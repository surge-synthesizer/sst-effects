/*
 * sst-effects - an open source library of audio effects
 * built by Surge Synth Team.
 *
 * Copyright 2018-2023, various authors, as described in the GitHub
 * transaction log.
 *
 * sst-effects is released under the GNU General Public Licence v3
 * or later (GPL-3.0-or-later). The license is found in the "LICENSE"
 * file in the root of this repository, or at
 * https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * The majority of these effects at initiation were factored from
 * Surge XT, and so git history prior to April 2023 is found in the
 * surge repo, https://github.com/surge-synthesizer/surge
 *
 * All source in sst-effects available at
 * https://github.com/surge-synthesizer/sst-effects
 */

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_TREMOLO_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_TREMOLO_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <iostream>
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/basic-blocks/dsp/RNG.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

// An sst voice effect should do the following:

// Be part of a namespace like this one
namespace sst::voice_effects::modulation
{
// Inherit core::VoiceEffectTemplateBase<VFXConfig>
template <typename VFXConfig> struct Tremolo : core::VoiceEffectTemplateBase<VFXConfig>
{
    // Have these constexpr members
    static constexpr const char *displayName{"Tremolo"};
    static constexpr const char *streamingName{"tremolo"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{3};

    basic_blocks::dsp::RNG &rng;

    // Enumerate its params
    enum FloatParams
    {
        fpVolume,
        fpRate,
        fpDepth,
        fpCrossover,
    };

    enum IntParams
    {
        ipHarmonic,
        ipShape,
        ipStereo
    };

    // And use functions such as these to define behavior of those params
    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpVolume:
            return pmd().asLinearDecibel(-60.f, 12.f).withName("Volume");
        case fpRate:
            return pmd().asLfoRate(-3, 4).withName("Rate");
        case fpDepth:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(.5f)
                .withLinearScaleFormatting("%", 100.f)
                .withName("Depth");
        case fpCrossover:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-24, 48)
                    .withName("Crossover")
                    .withDefault(0)
                    .withSemitoneFormatting();
            }
            return pmd().asAudibleFrequency().withName("Crossover");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipShape:
            return pmd()
                .asInt()
                .withRange(0, 6)
                .withUnorderedMapFormatting({
                    {0, "Sine"},
                    {1, "Triangle"},
                    {2, "Ramp Up"},
                    {3, "Ramp Down"},
                    {4, "Square"},
                    {5, "Noise"},
                    {6, "S&H"},
                })
                .withDefault(0)
                .withName("LFO shape");
        case ipHarmonic:
            return pmd().asOnOffBool().withDefault(false).withName("Harmonic");
        }
        return pmd().asInt().withName("Error");
    }

    // And these constructors and init functions
    Tremolo(basic_blocks::dsp::RNG &extrng)
        : core::VoiceEffectTemplateBase<VFXConfig>(), rng(extrng), LFO(this, rng)
    {
    }

    ~Tremolo() {}

    void initVoiceEffect()
    {
        filters[0].init();
        filters[1].init();
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    // Here comes stuff specific to this effect. For this tremolo effect we want a simple LFO,
    // which we have a simpleLFO class for. Let's introduce that here with an alias.
    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<Tremolo, VFXConfig::blockSize>;
    // create one...
    lfo_t LFO;
    // ...set up a variable for lfo shape.
    typename lfo_t::Shape lfoShape = lfo_t::Shape::SINE;

    // ...which we set in this little function that checks the shape parameter.
    void shapeCheck()
    {
        switch (this->getIntParam(ipShape))
        {
        case 0:
            lfoShape = lfo_t::SINE;
            break;
        case 1:
            lfoShape = lfo_t::TRI;
            break;
        case 2:
            lfoShape = lfo_t::DOWN_RAMP; // the ramps are in the other order in the param
            break;
        case 3:
            lfoShape =
                lfo_t::RAMP; // that's because we will subtract the lfo for reasons explained below
            break;
        case 4:
            lfoShape = lfo_t::PULSE;
            break;
        case 5:
            lfoShape = lfo_t::SMOOTH_NOISE;
            break;
        case 6:
            lfoShape = lfo_t::SH_NOISE;
            break;
        }
    }

    /*
     Next is the actual DSP.
     Tremolo is a really simple effect: Multiply each audio sample (or block of samples in our case)
     by one minus the LFO value on that sample.
     Harmonic tremolo is only a tiny bit more involved (ignoring the filter design...): Make
     highpassed and lowpassed copies of the signal, apply tremolo to those in opposite polarity.
     Meaning the highs will be loud when the lows are quiet and vice versa.
     There's 4 functions, for stereo+mono*harmonic+standard.
     I'll comment the stereo harmonic case. All the others have the same concepts,
     without the filters or the stereo or both.
    */

    void harmonicStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                        float *dataoutR, float pitch)
    {
        // Bring in the various params.
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto crossParam = this->getFloatParam(fpCrossover);

        // On a stereo signal, we might still want mono modulation (same on both sides, that is).
        // this param lets the user decide which one to use.
        bool stereo = this->getIntParam(ipStereo);

        // Keytrack is enabled via a function defined at the bottom
        if (keytrackOn)
        {
            crossParam += pitch; // if it's on, add the pitch value to the crossover freq
        }
        // Our pitch and frequency members are usually in units of (equal-tempered) semitones,
        // this is how we convert from that to a frequency in Hz.
        auto crossover = 440 * this->note_to_pitch_ignoring_tuning(crossParam);

        shapeCheck(); // Sets the lfo shape.

        // PhaseSet is false by default so this will run at note-on.
        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                // In the stereo modes, random phase works great for the smooth shapes
                auto phase = rng.unif01();   // so get a random number
                LFO.applyPhaseOffset(phase); // and initialize the LFO phase with it.
            }
            else if (lfoShape == lfo_t::PULSE)
            {
                // For the square we can make it start up or down at random,
                LFO.applyPhaseOffset(rng.boolean() ? 0.f : .5f);
            }
            else
            {
                // for the jagged shapes, start at 0 for more consistent note starts (random was
                // annoying here)
                LFO.applyPhaseOffset(0.f);
            }
            phaseSet = true; // last, set this true so it doesn't run next block.
        }

        // Run the LFO. (the 0.f is for the deform param which we don't use here).
        LFO.process_block(lfoRate, 0.f, lfoShape);

        // We want to eventually multiply our audio by the LFO value to modulate its amplitude.
        // But the LFO is bipolar, and if you multiply the signal by a negative value you just get
        // the same amplitude but polarity inverted. Not useful.
        // So make a variable for the LFO and its inverse, make it unipolar, and scale by depth:
        float lfoValueL = (LFO.lastTarget * .5f + .5f) * lfoDepth;
        float lfoValueInvL = (-LFO.lastTarget * .5f + .5f) * lfoDepth;

        // We also want to these to transition smoothly across the block.
        // We have these handy linear interpolators for that (defined in protected at the bottom).
        lfoLerp[0].set_target(1 - lfoValueL);
        lfoLerp[1].set_target(1 - lfoValueInvL);
        // 1 - value because we want low depth to mean no change (signal * 1).

        /*
         Ok, now to setup the filter coefficients. Though this filter is extremely efficient,
         it's still more expensive than an if statement. So we do the nice and easy optimisation:
         check if the crossover frequency changed since last block, recalculate the coefficients if
         so, else just keep them from last block. We initialize priorCrossover to a nonsense value
         below, so this will always run on the first block.
         */
        if (crossover != priorCrossover)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Lowpass, crossover, crossover, .5f, .5f,
                VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Highpass, crossover, crossover, .5f, .5f,
                VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            priorCrossover = crossover;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        // The datain and dataout args to process are pointers to well-aligned blocks of audio
        // samples. We could still process per-sample if we want, most procs do. Just loop over i
        // from 0 to block size. But here we don't need to, because we have all these handy
        // per-block functions.

        // So, define some temporary buffers at the block size.
        float leftLP alignas(16)[VFXConfig::blockSize];
        float leftHP alignas(16)[VFXConfig::blockSize];
        float rightLP alignas(16)[VFXConfig::blockSize];
        float rightHP alignas(16)[VFXConfig::blockSize];

        // Feed those into the filters
        filters[0].template processBlock<VFXConfig::blockSize>(datainL, datainR, leftLP, rightLP);
        filters[1].template processBlock<VFXConfig::blockSize>(datainL, datainR, leftHP, rightHP);

        // Then do a smoothed multiply with the lfo values
        lfoLerp[0].multiply_2_blocks(leftLP, (stereo ? rightHP : rightLP));
        lfoLerp[1].multiply_2_blocks(leftHP, (stereo ? rightLP : rightHP));
        // If the user asked for stereo modulation, right gets opposite LFOs to left, else it gets
        // the same

        // Sum the high and low passes together onto the output blocks
        sst::basic_blocks::mechanics::add_block<VFXConfig::blockSize>(leftLP, leftHP, dataoutL);
        sst::basic_blocks::mechanics::add_block<VFXConfig::blockSize>(rightLP, rightHP, dataoutR);
        // done!
    }

    /*
     That's it! Once a voice has been called, the above runs for the duration of that voice.
     The next voice runs another copy etc.
     At first I wrote one big function with lots of ifs, but it was a messy and seemed inefficient
     so I separated it. Now the branches are in the process functions further down instead.
     */

    void standardStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                        float *dataoutR, float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        bool stereo = this->getIntParam(ipStereo);

        shapeCheck();

        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                auto phase = rng.unif01();
                LFO.applyPhaseOffset(phase);
            }
            else if (lfoShape == lfo_t::PULSE)
            {
                LFO.applyPhaseOffset(rng.boolean() ? 0.f : .5f);
            }
            else
            {
                LFO.applyPhaseOffset(0.f);
            }
            phaseSet = true;
        }

        LFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (LFO.lastTarget * .5f + .5f) * lfoDepth;
        float lfoValueInv = (-LFO.lastTarget * .5f + .5f) * lfoDepth;
        lfoLerp[0].set_target(1 - lfoValue);
        lfoLerp[1].set_target(1 - (stereo ? lfoValueInv : lfoValue));

        // In harmonicStereo we made temporaries (cause we needed copies), and used
        // the filter function call to copy from input into them. Here we don't need to,
        // we can just copy to output directly and multiply output by the (smoothed) LFO values.

        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        lfoLerp[0].multiply_block(dataoutL);
        lfoLerp[1].multiply_block(dataoutR);
    }

    void harmonicMono(const float *const datain, float *dataout, float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto crossParam = this->getFloatParam(fpCrossover);
        if (keytrackOn)
        {
            crossParam += pitch;
        }
        auto crossover = 440 * this->note_to_pitch_ignoring_tuning(crossParam);

        shapeCheck();

        // In the Mono modes, random start phase was annoying since it makes note starts super
        // inconsistent. Let's make them all start in a sensible place instead.
        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                LFO.applyPhaseOffset(.75f); // loudest point
            }
            else if (lfoShape == lfo_t::PULSE)
            {
                LFO.applyPhaseOffset(.5f); // start high
            }
            else
            {
                LFO.applyPhaseOffset(.0f); // start at a transition
            }
            phaseSet = true;
        }

        LFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (LFO.lastTarget * .5f + .5f) * lfoDepth;
        float lfoValueInv = (-LFO.lastTarget * .5f + .5f) * lfoDepth;
        lfoLerp[0].set_target(1 - lfoValue);
        lfoLerp[1].set_target(1 - lfoValueInv);

        if (crossover != priorCrossover)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Lowpass, crossover, .5f,
                VFXConfig::getSampleRateInv(this), 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Highpass, crossover, .5f,
                VFXConfig::getSampleRateInv(this), 0.f);
            priorCrossover = crossover;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        float LP alignas(16)[VFXConfig::blockSize];
        float HP alignas(16)[VFXConfig::blockSize];

        filters[0].template processBlock<VFXConfig::blockSize>(datain, LP);
        filters[1].template processBlock<VFXConfig::blockSize>(datain, HP);

        lfoLerp[0].multiply_block(LP);
        lfoLerp[1].multiply_block(HP);

        sst::basic_blocks::mechanics::add_block<VFXConfig::blockSize>(LP, HP, dataout);
    }

    void standardMono(const float *const datain, float *dataout, float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);

        shapeCheck();

        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                LFO.applyPhaseOffset(.75f);
            }
            else if (lfoShape == lfo_t::PULSE)
            {
                LFO.applyPhaseOffset(.5f);
            }
            else
            {
                LFO.applyPhaseOffset(.0f);
            }
            phaseSet = true;
        }

        LFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (LFO.lastTarget * .5f + .5f) * lfoDepth;
        lfoLerp[0].set_target(1 - lfoValue);

        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datain, dataout);
        lfoLerp[0].multiply_block(dataout);
    }

    /*
     When starting a voice, the host will call one of the following functions.
     This first one gets called if incoming audio is Stereo...
    */
    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipHarmonic))
        {
            harmonicStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
        else
        {
            standardStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
        // Smooth the volume control
        volLerp.set_target(this->dbToLinear(this->getFloatParam(fpVolume)));
        volLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    // ...this second one if incoming audio is Mono...
    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        if (this->getIntParam(ipHarmonic))
        {
            harmonicMono(datainL, dataoutL, pitch);
        }
        else
        {
            standardMono(datainL, dataoutL, pitch);
        }
        volLerp.set_target(this->dbToLinear(this->getFloatParam(fpVolume)));
        volLerp.multiply_block(dataoutL);
    }

    // ...and this last one if the input is Mono, but the user asked for stereo modulation.
    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        // which in turn simply copies the mono audio and calls the stereo one with the copies.
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    // How does it know which MonoTo... function to choose? By first calling this.
    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

    // Shortcircuit et al can query this to set keytrack...
    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    // ...or this to check if it's enabled
    bool getKeytrack() const { return keytrackOn; }

    // ...and this signals that this proc can change its parameter names/units/etc after config time
    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<sst::filters::CytomicSVF, 2> filters;
    float priorCrossover = -1234.5678f;
    bool phaseSet = false;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> volLerp;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> lfoLerp[2];

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_TREMOLO_H
