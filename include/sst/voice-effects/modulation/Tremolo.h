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

#include <random>
#include <chrono>

#include <iostream>
#include "sst/basic-blocks/modulators/SimpleLFO.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct Tremolo : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Tremolo"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{3};

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

    Tremolo() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~Tremolo() {}

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
                .withDefault(1.f)
                .withLinearScaleFormatting("%", 100.f)
                .withName("Depth");
        case fpCrossover:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-24, 48)
                    .withName("Crossover Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
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
            return pmd().asBool().withDefault(false).withName("Stereo");
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
            return pmd().asBool().withDefault(false).withName("Harmonic");
        }
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect()
    {
        filters[0].init();
        filters[1].init();
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    // Ok, so let's introduce the simpleLFO with an alias.
    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<Tremolo, VFXConfig::blockSize>;
    // create one...
    lfo_t actualLFO{this, 1};
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
            lfoShape = lfo_t::DOWN_RAMP; // reverse the ramps because we subtract the lfo here.
            break;
        case 3:
            lfoShape = lfo_t::RAMP;
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

    // Here's a little RNG engine we'll use to randomize start phase
    float randUniZeroToOne()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        return static_cast<float>(dis(gen));
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

    void harmonicStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                        float pitch)
    {
        // Bring in the various params.
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto crossParam = this->getFloatParam(fpCrossover);
        if (keytrackOn)
        {
            crossParam += pitch;
        }
        auto crossover = 440 * this->note_to_pitch_ignoring_tuning(crossParam);

        // PhaseSet is false by default so this will run at note-on.
        if (!phaseSet)
        {
            auto phase = randUniZeroToOne();   // get a random number
            actualLFO.applyPhaseOffset(phase); // and initialize the LFO phase with it.
            phaseSet = true;                   // then set this true so it doesn't run next block.
        }

        shapeCheck(); // Sets the lfoshape.
        // Run the LFO. (the 0.f is for the deform param which we don't use here).
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        // Make a couple variables for the LFO value and its inverse,
        // both normalized into unipolar 0...1 range.
        float lfoValueL = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInvL = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;

        // If the user asked for stereo modulation, right gets opposite LFOs to left, else it gets
        // the same.
        float lfoValueR = (this->getIntParam(ipStereo) == true ? lfoValueInvL : lfoValueL);
        float lfoValueInvR = (this->getIntParam(ipStereo) == true ? lfoValueL : lfoValueInvL);

        /*
         Ok, now to setup the filter coefficients. Though this filter is extremely efficient,
         it's still more expensive than an if statement. So we do the nice and easy optimisation:
         check if the crossover frequency changed since last block, recalculate the coefficients if
         so, else just keep them from last block. We initialized priorCrossover to a nonsense value,
         so this will always run on the first block.
         */
        if (crossover != priorCrossover)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::LP, crossover, crossover, .5f, .5f,
                VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::HP, crossover, crossover, .5f, .5f,
                VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            priorCrossover = crossover;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        // Ok, here's where the action happens.
        // Process is per-block, but we want to calculate per-sample.
        // Hence the for loop from 0 to block size. Each i is a sample.
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            // First make two copies of the L and R inputs.
            auto inputLeftLP = datainL[i];
            auto inputLeftHP = datainL[i];
            auto inputRightLP = datainR[i];
            auto inputRightHP = datainR[i];

            // Feed those into the filters.
            filters[0].processBlockStep(inputLeftLP, inputRightLP);
            filters[1].processBlockStep(inputLeftHP, inputRightHP);

            // Now we have our high-and lowpassed versions of left and right.
            // Let's multiply them by the LFO values.
            inputLeftLP *= 1 - lfoValueL;
            inputLeftHP *= 1 - lfoValueInvL;
            inputRightLP *= 1 - lfoValueR;
            inputRightHP *= 1 - lfoValueInvR;
            // Remember the right LFO values will either be identical to the left ones or
            // opposite, depending on the stereo setting.

            // Now add the highpassed and lowpassed signals together for each side.
            inputLeftLP += inputLeftHP;
            inputRightLP += inputRightHP;
            // Normally when adding copies you have to divide down so it doens't get louder,
            // but not here because of the filtering.

            // lastly, pass each sample (scaled by the volume control) to the corresponding
            // output index to construct a block of audio to output.
            dataoutL[i] = inputLeftLP * outputVolume;
            dataoutR[i] = inputRightLP * outputVolume;
        }
    }

    /*
     That's it! Once a voice has been called, the above runs for the duration of that voice.
     The next voice runs another copy etc.
     At first I wrote one big function with lots of ifs, but it was a messy and seemed inefficient
     so I separated it. Now the branches are in the process functions further down instead.
     */

    void standardStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                        float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);

        if (!phaseSet)
        {
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }

        shapeCheck();
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInv = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inputL = datainL[i];
            auto inputR = datainR[i];

            inputL *= 1 - lfoValue;
            inputR *= 1 - (this->getIntParam(ipStereo) == true ? lfoValueInv : lfoValue);

            dataoutL[i] = inputL * outputVolume;
            dataoutR[i] = inputR * outputVolume;
        }
    }

    void harmonicMono(float *datainL, float *dataoutL, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto crossover = this->getFloatParam(fpCrossover);
        if (keytrackOn)
        {
            crossover += pitch;
        }

        if (!phaseSet)
        {
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }

        shapeCheck();
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInv = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;

        if (crossover != priorCrossover)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::LP, crossover, .5f, VFXConfig::getSampleRateInv(this), 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::HP, crossover, .5f, VFXConfig::getSampleRateInv(this), 0.f);
            priorCrossover = crossover;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inputLP = datainL[i];
            auto inputHP = datainL[i];

            filters[0].processBlockStep(inputLP);
            filters[1].processBlockStep(inputHP);

            inputLP *= 1 - lfoValue;
            inputHP *= 1 - lfoValueInv;

            inputLP += inputHP;

            dataoutL[i] = inputLP * outputVolume;
        }
    }

    void standardMono(float *datainL, float *dataoutL, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);

        if (!phaseSet)
        {
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }

        shapeCheck();
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataoutL[i] = (datainL[i] * (1 - lfoValue)) * outputVolume;
        }
    }

    /*
     When starting a voice, the host will call one of the following functions.
     This first one gets called if incoming audio is Stereo...
    */
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        if (this->getIntParam(ipHarmonic) == true)
        {
            harmonicStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
        else
        {
            standardStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
    }

    // ...this second one if incoming audio is Mono...
    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        if (this->getIntParam(ipHarmonic) == true)
        {
            harmonicMono(datainL, dataoutL, pitch);
        }
        else
        {
            standardMono(datainL, dataoutL, pitch);
        }
    }

    // ...and this last one if the input is Mono, but the user asked for stereo modulation.
    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        // which in turn simply copies the mono audio and calls the stereo one with the copies.
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    // How does it know which MonoTo... function to choose? By first calling this.
    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    bool keytrackOn{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<sst::filters::CytomicSVF, 2> filters;
    float priorCrossover = -1234.5678f;
    bool phaseSet = false;
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_TREMOLO_H
