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
#ifndef INCLUDE_SST_EFFECTS_FLANGER_H
#define INCLUDE_SST_EFFECTS_FLANGER_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"

namespace sst::effects::flanger
{
namespace sdsp = sst::basic_blocks::dsp;

template <typename FXConfig> struct Flanger : core::EffectTemplateBase<FXConfig>
{
    static constexpr double MIDI_0_FREQ = 8.17579891564371; // or 440.0 * pow( 2.0, - (69.0/12.0 ) )

    enum fl_modes
    {
        flm_classic = 0,
        flm_doppler,
        flm_arp_mix,
        flm_arp_solo,
    };

    enum fl_waves
    {
        flw_sine = 0,
        flw_tri,
        flw_saw,
        flw_sng,
        flw_snh,
        flw_square,
    };

    enum fl_params
    {
        // Basic Control
        fl_mode = 0, // flange, phase-inverse-flange, arepeggio, vibrato
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

        fl_num_params,
    };

    static constexpr int numParams{fl_num_params};
    static constexpr const char *effectName{"flanger"};

    Flanger(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
            typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
    }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return ringout_value; }
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
                .withDefault(0)
                .withRange(flw_sine, flw_square)
                .withUnorderedMapFormatting({{flw_sine, "Sine"},
                                             {flw_tri, "Triangle"},
                                             {flw_saw, "Sawtooth"},
                                             {flw_sng, "Noise"},
                                             {flw_snh, "Sample & Hold"},
                                             {flw_square, "Square"}});
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
                .withLinearScaleFormatting("semitones");

        case fl_feedback:
            return result.withName("Feedback").asPercent().withDefault(0.f);
        case fl_damping:
            return result.withName("HF Damping").asPercent().withDefault(0.1f);
        case fl_width:
            return this->getWidthParam();
        case fl_mix:
            return result.withName("Mix").asPercentBipolar().withDefault(0.8f);
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
        InterpDelay() { reset(); }
        void reset()
        {
            memset(line, 0, DELAY_SIZE * sizeof(float));
            k = 0;
        }
        float value(float delayBy);
        void push(float nv)
        {
            k = (k + 1) & DELAY_SIZE_MASK;
            line[k] = nv;
        }
    };

    int ringout_value = -1;
    InterpDelay idels[2];

    float lfophase[2][COMBS_PER_CHANNEL], longphase[2];
    float lpaL = 0.f, lpaR = 0.f; // state for the onepole LP filter

    sdsp::lipol<float, FXConfig::blockSize, true> lfoval[2][COMBS_PER_CHANNEL],
        delaybase[2][COMBS_PER_CHANNEL];
    sdsp::lipol<float, FXConfig::blockSize, true> depth, mix;
    sdsp::lipol<float, FXConfig::blockSize, true> voices, voice_detune, voice_chord;
    sdsp::lipol<float, FXConfig::blockSize, true> feedback, fb_hf_damping;
    sdsp::SurgeLag<float> vzeropitch;
    float lfosandhtarget[2][COMBS_PER_CHANNEL];
    float vweights[2][COMBS_PER_CHANNEL];

    sdsp::lipol_sse<FXConfig::blockSize, false> widthS, widthM;
    bool haveProcessed{false};

    const static int LFO_TABLE_SIZE = 8192;
    const static int LFO_TABLE_MASK = LFO_TABLE_SIZE - 1;
    float sin_lfo_table[LFO_TABLE_SIZE];
    float saw_lfo_table[LFO_TABLE_SIZE]; // don't make it analytic since I want to smooth the edges
  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

template <typename FXConfig> inline void Flanger<FXConfig>::initialize()
{
    for (int c = 0; c < 2; ++c)
        for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
        {
            lfophase[c][i] = 1.f * (i + 0.5 * c) / COMBS_PER_CHANNEL;
            lfosandhtarget[c][i] = 0.0;
        }
    longphase[0] = 0;
    longphase[1] = 0.5;

    for (int i = 0; i < LFO_TABLE_SIZE; ++i)
    {
        sin_lfo_table[i] = sin(2.0 * M_PI * i / LFO_TABLE_SIZE);

        saw_lfo_table[i] = 0;
    }
    haveProcessed = false;
}

template <typename FXConfig> inline float Flanger<FXConfig>::InterpDelay::value(float delayBy)
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
inline void Flanger<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    if (!haveProcessed)
    {
        float v0 = this->floatValue(fl_voice_basepitch);
        if (v0 > 0)
            haveProcessed = true;
        vzeropitch.startValue(v0);
    }
    // So here is a flanger with everything fixed

    float rate = this->envelopeRateLinear(-std::clamp(this->floatValue(fl_rate), -8.f, 10.f)) *
                 this->temposyncRatio(fl_rate);

    for (int c = 0; c < 2; ++c)
    {
        longphase[c] += rate;
        if (longphase[c] >= COMBS_PER_CHANNEL)
            longphase[c] -= COMBS_PER_CHANNEL;
    }

    const float oneoverFreq0 = 1.0f / MIDI_0_FREQ;

    int mode = this->intValue(fl_mode);
    int mwave = this->intValue(fl_wave);
    float depth_val = std::clamp(this->floatValue(fl_depth), 0.f, 2.f);

    float v0 = this->floatValue(fl_voice_basepitch);
    vzeropitch.newValue(v0);
    vzeropitch.process();
    v0 = vzeropitch.v;
    float averageDelayBase = 0.0;

    for (int c = 0; c < 2; ++c)
        for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
        {
            bool lforeset = false;

            lfophase[c][i] += rate;

            if (lfophase[c][i] > 1)
            {
                lforeset = true;
                lfophase[c][i] -= 1;
            }

            float lfoout = lfoval[c][i].v;
            float thisphase = lfophase[c][i];

            if (mode == flm_arp_mix || mode == flm_arp_solo)
            {
                // arpeggio - everyone needs to use the same phase with the voice swap
                thisphase = longphase[c] - (int)longphase[c];
            }

            switch (mwave)
            {
            case flw_sine:
            {
                float ps = thisphase * LFO_TABLE_SIZE;
                int psi = (int)ps;
                float psf = ps - psi;
                int psn = (psi + 1) & LFO_TABLE_MASK;

                lfoout = sin_lfo_table[psi] * (1.0 - psf) + psf * sin_lfo_table[psn];

                lfoval[c][i].newValue(lfoout);

                break;
            }
            case flw_tri:
                lfoout = (2.f * fabs(2.f * thisphase - 1.f) - 1.f);
                lfoval[c][i].newValue(lfoout);
                break;
            case flw_saw: // Gentler than a pure saw, more like a heavily skewed triangle
            {
                float cutAt = 0.98;
                float usephase;

                if (thisphase < cutAt)
                {
                    usephase = thisphase / cutAt;
                    lfoout = usephase * 2.0f - 1.f;
                }
                else
                {
                    usephase = (thisphase - cutAt) / (1.0 - cutAt);
                    lfoout = (1.0 - usephase) * 2.f - 1.f;
                }

                lfoval[c][i].newValue(lfoout);

                break;
            }
            case flw_square:
            {
                auto cutOffset = 0.02f;
                auto m = 2.f / cutOffset;
                auto c2 = cutOffset / 2.f;

                if (thisphase < 0.5f - c2)
                {
                    lfoout = 1.f;
                }
                else if ((thisphase >= 0.5 + c2) && (thisphase <= 1.f - cutOffset))
                {
                    lfoout = -1.f;
                }
                else if ((thisphase > 0.5 - c2) && (thisphase < 0.5 + c2))
                {
                    lfoout = -m * thisphase + (m / 2);
                }
                else
                {
                    lfoout = (m * thisphase) - (2 * m) + m + 1;
                }

                lfoval[c][i].newValue(lfoout);

                break;
            }
            case flw_sng: // Sample & Hold random
            case flw_snh: // Sample & Glide smoothed random
            {
                if (lforeset)
                {
                    lfosandhtarget[c][i] = this->storageRand01() - 1.f;
                }

                if (mwave == flw_sng)
                {
                    // FIXME exponential creep up. We want to get there in time related to our rate
                    auto cv = lfoval[c][i].v;
                    auto diff = (lfosandhtarget[c][i] - cv) * rate * 2;
                    lfoval[c][i].newValue(cv + diff);
                }
                else
                {
                    lfoval[c][i].newValue(lfosandhtarget[c][i]);
                }
            }
            break;
            }

            auto combspace = this->floatValue(fl_voice_spacing);
            float pitch = v0 + combspace * i;
            float nv = this->sampleRate() * oneoverFreq0 * this->noteToPitchInv((float)(pitch));

            // OK so biggest tap = delaybase[c][i].v * ( 1.0 + lfoval[c][i].v * depth.v ) + 1;
            // Assume lfoval is [-1,1] and depth is known
            float maxtap = nv * (1.0 + depth_val) + 1;
            if (maxtap >= InterpDelay::DELAY_SIZE)
            {
                nv = nv * 0.999 * InterpDelay::DELAY_SIZE / maxtap;
            }
            delaybase[c][i].newValue(nv);

            averageDelayBase += delaybase[c][i].new_v;
        }
    averageDelayBase /= (2 * COMBS_PER_CHANNEL);
    vzeropitch.process();

    float dApprox = rate * this->sampleRate() / FXConfig::blockSize * averageDelayBase * depth_val;

    depth.newValue(depth_val);
    mix.newValue(this->floatValue(fl_mix));
    voices.newValue(std::clamp(this->floatValue(fl_voices), 1.f, 4.f));
    float feedbackScale = 0.4 * sqrt((std::clamp(dApprox, 2.f, 60.f) + 30) / 100.0);

    // Feedback adjust based on mode
    switch (mode)
    {
    case flm_classic:
    {
        float dv = (voices.v - 1);
        feedbackScale += (3.0 - dv) * 0.45 / 3.0;
        break;
    }
    case flm_doppler:
    {
        float dv = (voices.v - 1);
        feedbackScale += (3.0 - dv) * 0.45 / 3.0;
        break;
    }
    case flm_arp_solo:
    {
        // this is one voice doppler basically
        feedbackScale += 0.2;
    }
    case flm_arp_mix:
    {
        // this is one voice classic basically and the steady signal clamps away feedback more
        feedbackScale += 0.3;
    }
    default:
        break;
    }

    float fbv = this->floatValue(fl_feedback);
    if (fbv > 0)
        ringout_value = this->sampleRate() * 32.0;
    else
        ringout_value = 1024;

    if (mwave == flw_saw || mwave == flw_snh)
    {
        feedbackScale *= 0.7;
    }

    if (fbv > 0 && fbv < 1)
    {
        fbv = sqrt(fbv);
    }

    feedback.newValue(feedbackScale * fbv);
    fb_hf_damping.newValue(0.4 * this->floatValue(fl_damping));
    float combs alignas(16)[2][FXConfig::blockSize];

    // Obviously when we implement stereo spread this will be different
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
                // std::cout << _D(longphase) << _D(dt) << _D(nxt) << _D(prr) << _D(ilp) << _D(flp)
                // << std::endl;
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
            float voices = std::clamp(this->floatValue(fl_voices), 1.f, COMBS_PER_CHANNEL * 1.f);
            vweights[c][0] = 1.0;

            for (int i = 0; i < voices && i < 4; ++i)
                vweights[c][i] = 1.0;

            int li = (int)voices;
            float fi = voices - li;
            if (li < 4)
                vweights[c][li] = fi;
        }
    }

    for (int b = 0; b < FXConfig::blockSize; ++b)
    {
        for (int c = 0; c < 2; ++c)
        {
            combs[c][b] = 0;
            for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
            {
                if (vweights[c][i] > 0)
                {
                    auto tap = delaybase[c][i].v * (1.0 + lfoval[c][i].v * depth.v) + 1;
                    auto v = idels[c].value(tap);
                    combs[c][b] += vweights[c][i] * v;
                }

                lfoval[c][i].process();
                delaybase[c][i].process();
            }
        }
        // softclip the feedback to avoid explosive runaways
        float fbl = 0.f;
        float fbr = 0.f;
        if (feedback.v > 0)
        {
            fbl = std::clamp(feedback.v * combs[0][b], -1.f, 1.f);
            fbr = std::clamp(feedback.v * combs[1][b], -1.f, 1.f);

            fbl = 1.5 * fbl - 0.5 * fbl * fbl * fbl;
            fbr = 1.5 * fbr - 0.5 * fbr * fbr * fbr;

            // and now we have clipped, apply the damping. FIXME - move to one mul form
            float df = std::clamp(fb_hf_damping.v, 0.01f, 0.99f);
            lpaL = lpaL * (1.0 - df) + fbl * df;
            fbl = fbl - lpaL;

            lpaR = lpaR * (1.0 - df) + fbr * df;
            fbr = fbr - lpaR;
        }

        auto vl = dataL[b] - fbl;
        auto vr = dataR[b] - fbr;
        idels[0].push(vl);
        idels[1].push(vr);

        auto origw = 1.f;
        if (mode == flm_doppler || mode == flm_arp_solo)
        {
            // doppler modes
            origw = 0.f;
        }

        float outl = origw * dataL[b] + mix.v * combs[0][b];
        float outr = origw * dataR[b] + mix.v * combs[1][b];

        // Some gain heueirstics
        float gainadj = 0.0;
        switch (mode)
        {
        case flm_classic:
            gainadj = -1 / sqrt(7 - voices.v);
            break;
        case flm_doppler:
            gainadj = -1 / sqrt(8 - voices.v);
            break;
        case flm_arp_mix:
            gainadj = -1 / sqrt(6);
            break;
        case flm_arp_solo:
            gainadj = -1 / sqrt(7);
            break;
        }

        gainadj -= 0.07 * mix.v;

        outl = std::clamp((1.0f + gainadj) * outl, -1.f, 1.f);
        outr = std::clamp((1.0f + gainadj) * outr, -1.f, 1.f);

        outl = 1.5 * outl - 0.5 * outl * outl * outl;
        outr = 1.5 * outr - 0.5 * outr * outr * outr;

        dataL[b] = outl;
        dataR[b] = outr;

        depth.process();
        mix.process();
        feedback.process();
        fb_hf_damping.process();
        voices.process();
    }

    this->setWidthTarget(widthS, widthM, fl_width, 1.0 / 3.0);
    this->applyWidth(dataL, dataR, widthS, widthM);
}

} // namespace sst::effects::flanger

#endif
