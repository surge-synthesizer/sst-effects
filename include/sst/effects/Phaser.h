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

#ifndef INCLUDE_SST_EFFECTS_PHASER_H
#define INCLUDE_SST_EFFECTS_PHASER_H

#include <cstring>
#include <cmath>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/modulators/FXModControl.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/filters/BiquadFilter.h"

namespace sst::effects::phaser
{
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct Phaser : core::EffectTemplateBase<FXConfig>
{
    enum phaser_params
    {
        ph_center = 0,
        ph_feedback,
        ph_sharpness,
        ph_mod_rate,
        ph_mod_depth,
        ph_stereo,
        ph_mix,
        ph_width,
        ph_stages,
        ph_spread,
        ph_mod_wave,
        ph_tone,

        ph_num_params,
    };

    using BiquadFilter = typename core::EffectTemplateBase<FXConfig>::BiquadFilterType;

    static constexpr int numParams{ph_num_params};
    static constexpr const char *streamingName{"phaser"};
    static constexpr const char *displayName{"Phaser"};

    Phaser(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
           typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p), lp(s), hp(s)
    {
        static_assert(core::ValidEffect<Phaser>);
        for (int i = 0; i < n_bq_units; i++)
        {
            biquad[i] = new BiquadFilter(this->globalStorage);
        }

        n_bq_units_initialised = n_bq_units;
        feedback.setBlockSize(FXConfig::blockSize * this->slowrate);
        tone.setBlockSize(FXConfig::blockSize);
        widthS.set_blocksize(FXConfig::blockSize);
        widthM.set_blocksize(FXConfig::blockSize);
        lp.setBlockSize(FXConfig::blockSize);
        hp.setBlockSize(FXConfig::blockSize);
        mix.set_blocksize(FXConfig::blockSize);
        bi = 0;
    }

    void initialize()
    {
        bi = 0;
        dL = 0;
        dR = 0;

        for (int i = 0; i < n_bq_units_initialised; i++)
        {
            biquad[i]->suspend();
        }

        mech::clear_block<FXConfig::blockSize>(L);
        mech::clear_block<FXConfig::blockSize>(R);

        lp.suspend();
        hp.suspend();

        lp.coeff_instantize();
        hp.coeff_instantize();

        mix.set_target(1.f);

        tone.instantize();
        widthS.instantize();
        widthM.instantize();
        mix.instantize();

        modLFO.setSampleRate(this->sampleRate());
    }

    ~Phaser()
    {
        for (int i = 0; i < n_bq_units_initialised; ++i)
            delete biquad[i];
    }

    void onSampleRateChanged()
    {
        modLFO.setSampleRate(this->sampleRate());
        initialize();
    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const
    {
        auto fb = this->floatValue(ph_feedback);

        // The ringout is longer at higher feedback. This is just a heuristic based on
        // testing with the patch in #2663. Note that at feedback above 1 (from
        // modulation or control pushes) you can get infinite self-oscillation
        // so run forever then

        if (fb > 1 || fb < -1)
        {
            return -1;
        }

        if (fb > 0.9 || fb < -0.9)
        {
            return 5000;
        }

        if (fb > 0.5 || fb < -0.5)
        {
            return 3000;
        }

        return 1000;
    }
    size_t silentSamplesLength() const { return 10; }

    void init_stages()
    {
        n_stages = this->intValue(ph_stages);
        n_bq_units = n_stages * 2;

        if (n_bq_units_initialised < n_bq_units)
        {
            // we need to increase the number of stages
            for (int k = n_bq_units_initialised; k < n_bq_units; k++)
            {
                biquad[k] = new BiquadFilter(this->globalStorage);
            }

            n_bq_units_initialised = n_bq_units;
        }
    }
    void setvars()
    {
        init_stages();

        double rate = this->envelopeRateLinear(-this->floatValue(ph_mod_rate)) *
                      this->temposyncRatio(ph_mod_rate);

        rate *= (float)this->slowrate;

        int mwave = this->intValue(ph_mod_wave);
        float depth = std::clamp(this->floatValue(ph_mod_depth), 0.f, 2.f);
        float stereo = std::clamp(this->floatValue(ph_stereo), 0.f, 1.f);

        if (this->isDeactivated(ph_mod_rate))
        {
            // auto rmin = fxdata->p[ph_mod_rate].val_min.f;
            // auto rmax = fxdata->p[ph_mod_rate].val_max.f;
            auto rmin = -7.f; // we cant query this at release runtime on the audio thread
            auto rmax = 9.f;
            auto phase =
                std::clamp((this->floatValue(ph_mod_rate) - rmin) / (rmax - rmin), 0.f, 1.f);

            modLFO.processStartOfBlock(mwave, 0.f, depth, phase, stereo);
        }
        else
        {
            modLFO.processStartOfBlock(mwave, rate, depth, 0.f, stereo);
        }

        auto lfoVals = modLFO.valueStereo();

        // if stages is set to 1 to indicate we are in legacy mode, use legacy freqs and spans
        if (n_stages < 2)
        {
            // 4 stages in original phaser mode
            for (int i = 0; i < 2; i++)
            {
                double omega = biquad[2 * i]->calc_omega(
                    2 * this->floatValue(ph_center) + legacy_freq[i] + legacy_span[i] * lfoVals[0]);
                biquad[2 * i]->coeff_APF(omega, 1.0 + 0.8 * this->floatValue(ph_sharpness));
                omega = biquad[2 * i + 1]->calc_omega(2 * this->floatValue(ph_center) +
                                                      legacy_freq[i] + legacy_span[i] * lfoVals[1]);
                biquad[2 * i + 1]->coeff_APF(omega, 1.0 + 0.8 * this->floatValue(ph_sharpness));
            }
        }
        else
        {
            for (int i = 0; i < n_stages; i++)
            {
                double center = powf(2, (i + 1.0) * 2 / n_stages);
                double omega = biquad[2 * i]->calc_omega(2 * this->floatValue(ph_center) +
                                                         this->floatValue(ph_spread) * center +
                                                         2.0 / (i + 1) * lfoVals[0]);
                biquad[2 * i]->coeff_APF(omega, 1.0 + 0.8 * this->floatValue(ph_sharpness));
                omega = biquad[2 * i + 1]->calc_omega(2 * this->floatValue(ph_center) +
                                                      this->floatValue(ph_spread) * center +
                                                      2.0 / (i + 1) * lfoVals[1]);
                biquad[2 * i + 1]->coeff_APF(omega, 1.0 + 0.8 * this->floatValue(ph_sharpness));
            }
        }

        feedback.newValue(0.95f * this->floatValue(ph_feedback));
        tone.newValue(std::clamp(this->floatValue(ph_tone), -1.f, 1.f));

        this->setWidthTarget(widthS, widthM, ph_width);

        // lowpass range is from MIDI note 136 down to 57 (~21.1 kHz down to 220 Hz)
        // highpass range is from MIDI note 34 to 136(~61 Hz to ~21.1 kHz)
        float clo = -12, cmid = 67, chi = -33;
        float hpCutoff = chi;
        float lpCutoff = cmid;

        if (tone.v > 0)
        {
            // OK so cool scale the hp cutoff
            auto tv = tone.v;
            hpCutoff = tv * (cmid - chi) + chi;
        }
        else
        {
            auto tv = -tone.v;
            lpCutoff = tv * (clo - cmid) + cmid;
        }

        lp.coeff_LP(lp.calc_omega((lpCutoff / 12.0) - 2.f), 0.707);
        hp.coeff_HP(hp.calc_omega((hpCutoff / 12.0) - 2.f), 0.707);
    }

    void processBlock(float *__restrict dataL, float *__restrict dataR)
    {
        if (bi == 0)
        {
            setvars();
        }

        bi = (bi + 1) & this->slowrate_m1;

        for (int i = 0; i < FXConfig::blockSize; i++)
        {
            feedback.process();
            tone.process();

            dL = dataL[i] + dL * feedback.v;
            dR = dataR[i] + dR * feedback.v;
            dL = std::clamp(dL, -32.f, 32.f);
            dR = std::clamp(dR, -32.f, 32.f);

            for (int curr_stage = 0; curr_stage < n_stages; curr_stage++)
            {
                dL = biquad[2 * curr_stage]->process_sample(dL);
                dR = biquad[2 * curr_stage + 1]->process_sample(dR);
            }

            L[i] = dL;
            R[i] = dR;
        }

        if (!this->isDeactivated(ph_tone))
        {
            lp.process_block(L, R);
            hp.process_block(L, R);
        }

        this->applyWidth(L, R, widthS, widthM);

        mix.set_target_smoothed(std::clamp(this->floatValue(ph_mix), 0.f, 1.f));
        mix.fade_2_blocks_inplace(dataL, L, dataR, R, this->blockSize_quad);
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numParams);
        using pmd = basic_blocks::params::ParamMetaData;

        /*
         * fxdata->p[ph_stages].val.i = 4;
fxdata->p[ph_width].val.f = 0.f;
fxdata->p[ph_spread].val.f = 0.f;
fxdata->p[ph_mod_wave].val.i = 1;
fxdata->p[ph_tone].val.f = 0.f;
fxdata->p[ph_tone].deactivated = false;
fxdata->p[ph_mod_rate].deactivated = false;
         */

        switch ((phaser_params)idx)
        {
        case ph_center:
            return pmd().asPercentBipolar().withName("Center").withDefault(0);
        case ph_feedback:
            return pmd().asPercentBipolar().withName("Feedback").withDefault(0);
        case ph_sharpness:
            return pmd().asPercentBipolar().withName("Sharpness").withDefault(0);
        case ph_mod_depth:
            return pmd().asPercent().withName("Depth").withDefault(0);
        case ph_stereo:
            return pmd().asPercent().withName("Stereo").withDefault(0);
        case ph_mix:
            return pmd().asPercent().withName("Mix").withDefault(0.5);
        case ph_width:
            return this->getWidthParam();
        case ph_tone:
            return pmd().withName("Tone").asPercentBipolar().deactivatable(true).withDefault(0.f);
        case ph_mod_rate:
            return pmd().withName("Rate").asLfoRate().deactivatable(true);
        case ph_stages:
            return pmd()
                .asInt()
                .withName("Count")
                .withRange(1, 16)
                .withDefault(4)
                .withLinearScaleFormatting("stages")
                .withCustomMaxDisplay("Legacy (4 Stages)");
        case ph_spread:
            return pmd().asPercent().withName("Spread");
        case ph_mod_wave:
            return pmd()
                .asInt()
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
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    sst::basic_blocks::dsp::lipol_sse<FXConfig::blockSize, false> widthS, widthM, mix;

    float L alignas(16)[FXConfig::blockSize], R alignas(16)[FXConfig::blockSize];

    sst::basic_blocks::dsp::lipol<float, FXConfig::blockSize, true> feedback, tone;
    static constexpr int max_stages = 16;
    static constexpr int default_stages = 4;
    int n_stages = default_stages;
    int n_bq_units = default_stages << 1;
    int n_bq_units_initialised = 0;
    float dL, dR;
    BiquadFilter *biquad[max_stages * 2], lp, hp;
    int bi; // block increment (to keep track of events not occurring every n blocks)

    // before stages/spread added parameters we had 4 stages at fixed frequencies and modulation
    // depth span
    float legacy_freq[4] = {1.5 / 12, 19.5 / 12, 35 / 12, 50 / 12};
    float legacy_span[4] = {2.0, 1.5, 1.0, 0.5};

    sst::basic_blocks::modulators::FXModControl<FXConfig::blockSize> modLFO;

  public:
    static constexpr int16_t streamingVersion{2};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        assert(streamedFrom <= streamingVersion);
        // 1->2 reordered mod waves and added actual saw (renamed "saw" to ramp)
        if (streamedFrom < 2)
        {
            if (param[ph_mod_wave] == 3 || param[ph_mod_wave] == 4)
            {
                // noise and snh were 3/4, are now 5/6
                param[ph_mod_wave] += 2;
                // and stereo could not be reduced, but now can
                param[ph_stereo] = 1.f;
            }
            else if (param[ph_mod_wave] == 5)
            {
                // square was 5, is now 4
                param[ph_mod_wave] -= 1;
            }
        }
    }
};
} // namespace sst::effects::phaser

#endif // SURGE_PHASER_H
