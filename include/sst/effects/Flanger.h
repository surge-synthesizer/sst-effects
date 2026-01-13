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
#ifndef INCLUDE_SST_EFFECTS_FLANGER2_H
#define INCLUDE_SST_EFFECTS_FLANGER2_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/modulators/FXModControl.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::effects::flanger2
{
namespace sdsp = sst::basic_blocks::dsp;

template <typename FXConfig> struct Flanger2 : core::EffectTemplateBase<FXConfig>
{
    static constexpr double MIDI_0_FREQ = 8.17579891564371; // or 440.0 * pow( 2.0, - (69.0/12.0 ) )

    enum fl_modes
    {
        flm_classic = 0,
        flm_doppler,
        flm_arp_mix,
        flm_arp_solo,
    };

    enum fl_params
    {
        // Basic Control
        fl_mode = 0, // flange, phase-inverse-flange, arpeggio, vibrato
        fl_wave,     // what's the wave shape
        fl_rate,     // How quickly the oscillations happen
        fl_depth,    // How extreme the modulation of the delay is

        // Voices
        fl_voices,          // how many delay lines
        fl_voice_basepitch, // tune the first max delay line to this (M = sr / f)
        fl_voice_spacing,   // How far apart are the combs in pitch space

        // Feedback and EQ
        fl_feedback, // how much the output feeds back into the filters
        fl_damping,  // how much low pass damping in the feedback mechanism
        fl_width,    // how much to pan the delay lines ( 0 -> all even; 1 -> full spread)
        fl_mix,      // how much we add the comb into the mix
        fl_stereo,   // how much stereo-ness the LFO adds

        fl_num_params,
    };

    static constexpr int numParams{fl_num_params};
    static constexpr const char *streamingName{"flanger"};
    static constexpr const char *displayName{"Flanger"};

    sst::basic_blocks::tables::SurgeSincTableProvider sSincTable;

    Flanger2(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
             typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
    }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return ringout_value; }
    size_t silentSamplesLength() const { return 10; }
    void onSampleRateChanged() { initialize(); }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numParams);
        using pmd = basic_blocks::params::ParamMetaData;
        auto result = pmd().withName("Unknown " + std::to_string(idx));

        switch ((fl_params)idx)
        {
        case fl_mode:
            return result.withType(pmd::INT)
                .withName("Mode")
                .withDefault(0)
                .withRange(flm_classic, flm_arp_solo)
                .withUnorderedMapFormatting({{flm_classic, "Dry + Combs"},
                                             {flm_doppler, "Combs Only"},
                                             {flm_arp_mix, "Dry + Arp Combs"},
                                             {flm_arp_solo, "Arp Combs Only"}});
        case fl_wave:
            return result.withType(pmd::INT)
                .withName("Waveform")
                .withRange(0, 6)
                .withUnorderedMapFormatting({
                    {0, "Sine"},
                    {1, "Triangle"},
                    {2, "Ramp"},
                    {3, "Saw"},
                    {4, "Square"},
                    {5, "Noise"},
                    {6, "Sample & Hold"},
                })
                .withDefault(0);
        case fl_rate:
            return result.withName("Rate")
                .withRange(-7, 9)
                .withDefault(-2.f)
                .temposyncable()
                .withTemposyncMultiplier(-1)
                .withATwoToTheBFormatting(1, 1, "Hz")
                .withDecimalPlaces(3);
        case fl_depth:
            return result.withName("Depth").asPercent().withDefault(1.f);
        case fl_voices:
            return result.withName("Count")
                .withRange(1.f, 4.f)
                .withDefault(4.f)
                .withLinearScaleFormatting("Voices");
        case fl_voice_basepitch:
            return result.withName("Base Pitch").asMIDIPitch();
        case fl_voice_spacing:
            return result.withName("Spacing")
                .withRange(0.f, 12.f)
                .withDefault(0.f)
                .withSemitoneFormatting();

        case fl_feedback:
            return result.withName("Feedback").asPercent().withDefault(0.f);
        case fl_damping:
            return result.withName("HF Damping").asPercent().withDefault(0.1f);
        case fl_width:
            return this->getWidthParam();
        case fl_mix:
            return result.withName("Mix").asPercentBipolar().withDefault(0.8f);
        case fl_stereo:
            return pmd().asPercent().withName("Stereo").withDefault(1);
        case fl_num_params:
            throw std::logic_error("getParam called with num_params");
        }
        return result;
    }

  protected:
    static constexpr int COMBS_PER_CHANNEL = 4;
    struct InterpDelay
    {
        // OK so lets say we want lowest tunable frequency to be 23.5hz at 96k
        // 96000/23.5 = 4084
        // And lets future proof a bit and make it a power of 2 so we can use & properly
        static constexpr int DELAY_SIZE = 32768, DELAY_SIZE_MASK = DELAY_SIZE - 1;
        float line[DELAY_SIZE];
        int k = 0;
        InterpDelay() { clear(); }
        void clear()
        {
            memset(line, 0, DELAY_SIZE * sizeof(float));
            k = 0;
        }
        float read(float delayBy);
        void write(float nv)
        {
            k = (k + 1) & DELAY_SIZE_MASK;
            line[k] = nv;
        }
    };
    InterpDelay idels[2];

    int ringout_value = -1;
    float lpaL = 0.f, lpaR = 0.f; // state for the onepole LP filter

    sst::basic_blocks::modulators::FXModControl<FXConfig::blockSize,
                                                sst::basic_blocks::modulators::rnd_single>
        LFOs[2];

    sdsp::lipol<float, FXConfig::blockSize, true> delayBaseLerp[COMBS_PER_CHANNEL];
    sdsp::lipol<float, FXConfig::blockSize, true> mixLerp, voicesLerp, feedbackLerp, gainAdjLerp,
        fb_hf_dampingLerp;
    sdsp::SurgeLag<float> vzeropitchLag;
    float vweights[2][COMBS_PER_CHANNEL];

    sdsp::lipol_sse<FXConfig::blockSize, false> widthS, widthM;
    bool haveProcessed{false};

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

template <typename FXConfig> inline void Flanger2<FXConfig>::initialize()
{
    for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    {
        LFOs[i].setSampleRate(this->sampleRate());
    }
    idels[0].clear();
    idels[1].clear();
    haveProcessed = false;
}

template <typename FXConfig> inline float Flanger2<FXConfig>::InterpDelay::read(float delayBy)
{
    // so if delayBy is 19.2
    int itap = (int)std::min(delayBy, (float)(DELAY_SIZE - 2)); // this is 19
    float fractap = delayBy - itap;                             // this is .2
    int k0 = (k + DELAY_SIZE - itap - 1) & DELAY_SIZE_MASK;     // this is 20 back
    int k1 = (k + DELAY_SIZE - itap) & DELAY_SIZE_MASK;         // this is 19 back
    float result =
        line[k0] * fractap + line[k1] * (1.0 - fractap); // FIXME move to the one mul form

    return result;
}

template <typename FXConfig>
inline void Flanger2<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    /*
     * Ok soooo
     * 1: wtf is wrong with feedback
     * 3: don't take on any new tasks until I can null the new thing against the old
     *
     * once I'm confident in that:
     * 4: Break out the per-sample stuff into a template function
     * 5: Move to the other interpolation thingie
     * 6: Make a __m128 read function from the delay lines
     * 7: Profit????
     *
     * It's eeeh maybe not great?
     * Make a legacy template arg?
     * Anyuway....
     */

    const float oneoverFreq0 = 1.0f / MIDI_0_FREQ;

    if (!haveProcessed)
    {
        float v0 = this->floatValue(fl_voice_basepitch);
        if (v0 > 0)
            haveProcessed = true;
        vzeropitchLag.startValue(v0);
    }

    float v0 = this->floatValue(fl_voice_basepitch);
    vzeropitchLag.newValue(v0);
    vzeropitchLag.process();
    v0 = vzeropitchLag.v;
    float averageDelayBase = 0.0;

    float rate = this->envelopeRateLinear(-std::clamp(this->floatValue(fl_rate), -8.f, 10.f)) *
                 this->temposyncRatio(fl_rate);
    int mode = this->intValue(fl_mode);
    int mwave = this->intValue(fl_wave);
    float depth = std::clamp(this->floatValue(fl_depth), 0.f, 2.f);
    auto combspace = this->floatValue(fl_voice_spacing);


    float stereo = this->floatValue(fl_stereo);
    float combPhaseSpread{1.f};

    if (mode == flm_arp_mix || mode == flm_arp_solo)
    {
        rate *= .25f;
        stereo *= .5f;
        combPhaseSpread = 0.f;
    }
    else
    {
        stereo *= .125f;
        combPhaseSpread = 1.f;
    }

    std::array<std::array<float, COMBS_PER_CHANNEL>, 2> lfoVals;

    LFOs[0].processStartOfBlock(mwave, rate, depth, stereo, combPhaseSpread);
    lfoVals[0] = LFOs[0].valueQuad();

    LFOs[1].processStartOfBlock(mwave, rate, depth, stereo, combPhaseSpread);
    lfoVals[1] = LFOs[1].valueQuad();

    for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    {
        float nv = this->sampleRate() * oneoverFreq0 * this->noteToPitchInv(v0 + combspace * i);

        // OK so biggest tap = delayBase[i].v * (1.0 + lfoVal[c][i].v) + 1;
        // Assume lfoval is [-1,1] and depth is known
        float maxtap = nv * (1.0 + depth) + 1;
        if (maxtap >= InterpDelay::DELAY_SIZE)
        {
            nv = nv * 0.999f * InterpDelay::DELAY_SIZE / maxtap;
        }
        delayBaseLerp[i].newValue(nv);
        averageDelayBase += nv;
    }

    vzeropitchLag.process();

    averageDelayBase /= COMBS_PER_CHANNEL;
    float dApprox = rate * this->sampleRate() / FXConfig::blockSize * averageDelayBase * depth;
    float feedbackScale = 0.4 * sqrt((std::clamp(dApprox, 2.f, 60.f) + 30) / 100.0);

    const auto voices = std::clamp(this->floatValue(fl_voices), 1.f, (float)COMBS_PER_CHANNEL);
    auto gainadj{0.f};
    // Feedback adjust based on mode, plus some gain heuristics
    switch (mode)
    {
    case flm_classic:
    {
        gainadj = -1 / sqrt(7 - voices);
        float dv = (voicesLerp.v - 1);
        feedbackScale += (3.0 - dv) * 0.45 / 3.0;
        break;
    }
    case flm_doppler:
    {
        gainadj = -1 / sqrt(8 - voices);
        float dv = (voicesLerp.v - 1);
        feedbackScale += (3.0 - dv) * 0.45 / 3.0;
        break;
    }
    case flm_arp_solo:
    {
        // this is one voice doppler basically
        feedbackScale += 0.2;
        gainadj = -1 / sqrt(7);
    }
    case flm_arp_mix:
    {
        // this is one voice classic basically and the steady signal clamps away feedback more
        feedbackScale += 0.3;
        gainadj = -1 / sqrt(6);
    }
    default:
        break;
    }

    auto mix = this->floatValue(fl_mix);
    gainadj -= 0.07 * mix;
    mixLerp.newValue(mix);
    gainAdjLerp.newValue(gainadj);

    float fbv = this->floatValue(fl_feedback);
    if (fbv > 0)
        ringout_value = this->sampleRate() * 32.0;
    else
        ringout_value = 1024;

    if (mwave == 2 || mwave == 3 || mwave == 6)
    {
        feedbackScale *= 0.7;
    }

    if (fbv > 0 && fbv < 1)
    {
        fbv = sqrt(fbv);
    }

    feedbackLerp.newValue(feedbackScale * fbv);
    fb_hf_dampingLerp.newValue(0.4 * this->floatValue(fl_damping));

    auto longphase = LFOs[0].getLastPhase();
    longphase[0] *= COMBS_PER_CHANNEL;
    longphase[1] *= COMBS_PER_CHANNEL;

    for (int c = 0; c < 2; ++c)
    {
        for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
            vweights[c][i] = 0;

        if (mode == flm_arp_mix || mode == flm_arp_solo)
        {
            int ilp = (int)longphase[c];
            float flp = longphase[c] - ilp;

            if (ilp == COMBS_PER_CHANNEL)
                ilp = 0;

            if (flp > 0.9)
            {
                float dt = (flp - 0.9) * 10; // this will be between 0,1
                float nxt = sqrt(dt);
                float prr = sqrt(1.f - dt);
                vweights[c][ilp] = prr;
                if (ilp == COMBS_PER_CHANNEL - 1)
                    vweights[c][0] = nxt;
                else
                    vweights[c][ilp + 1] = nxt;
            }
            else
            {
                vweights[c][ilp] = 1.f;
            }
        }
        else
        {
            for (int i = 0; i < voices; i++)
                vweights[c][i] = 1.0;

            int li = (int)voices;
            float fi = voices - li;
            if (li < 4)
                vweights[c][li] = fi;
        }
    }

    float combs alignas(16)[2][FXConfig::blockSize];
    sst::basic_blocks::mechanics::clear_block<FXConfig::blockSize>(combs[0]);
    sst::basic_blocks::mechanics::clear_block<FXConfig::blockSize>(combs[1]);

    auto drythrough = 1.f;
    if (mode == flm_doppler || mode == flm_arp_solo)
    {
        drythrough = 0.f;
    }

    for (int b = 0; b < FXConfig::blockSize; ++b)
    {
        for (int c = 0; c < 2; ++c)
        {
            for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
            {
                auto tap = delayBaseLerp[i].v * (1.0 + lfoVals[c][i]) + 1;
                auto v = idels[c].read(tap);
                combs[c][b] += vweights[c][i] * v;
            }
            lfoVals[c] = LFOs[c].nextQuadValueInBlock();
        }

        for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
        {
            delayBaseLerp[i].process();
        }

        // hard- and soft-clip the feedback to avoid explosive runaways
        float fbl = 0.f;
        float fbr = 0.f;
        if (feedbackLerp.v > 0)
        {
            fbl = std::clamp(feedbackLerp.v * combs[0][b], -1.f, 1.f);
            fbr = std::clamp(feedbackLerp.v * combs[1][b], -1.f, 1.f);

            fbl = 1.5 * fbl - 0.5 * fbl * fbl * fbl;
            fbr = 1.5 * fbr - 0.5 * fbr * fbr * fbr;

            // and now we have clipped, apply the damping. FIXME - move to one mul form
            float df = std::clamp(fb_hf_dampingLerp.v, 0.01f, 0.99f);
            lpaL = lpaL * (1.0 - df) + fbl * df;
            fbl = fbl - lpaL;

            lpaR = lpaR * (1.0 - df) + fbr * df;
            fbr = fbr - lpaR;
        }

        auto vl = dataL[b] - fbl;
        auto vr = dataR[b] - fbr;
        idels[0].write(vl);
        idels[1].write(vr);

        float outl = drythrough * dataL[b] + mixLerp.v * combs[0][b];
        float outr = drythrough * dataR[b] + mixLerp.v * combs[1][b];

        outl = std::clamp((1.0f + gainAdjLerp.v) * outl, -1.f, 1.f);
        outr = std::clamp((1.0f + gainAdjLerp.v) * outr, -1.f, 1.f);

        outl = 1.5 * outl - 0.5 * outl * outl * outl;
        outr = 1.5 * outr - 0.5 * outr * outr * outr;

        dataL[b] = outl;
        dataR[b] = outr;

        mixLerp.process();
        feedbackLerp.process();
        gainAdjLerp.process();
        fb_hf_dampingLerp.process();
        voicesLerp.process();
    }

    this->setWidthTarget(widthS, widthM, fl_width, 1.0 / 3.0);
    this->applyWidth(dataL, dataR, widthS, widthM);
}

} // namespace sst::effects::flanger2

#endif
