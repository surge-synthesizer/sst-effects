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

#ifndef INCLUDE_SST_EFFECTS_DELAY_H
#define INCLUDE_SST_EFFECTS_DELAY_H

#include <cstring>
#include <cmath>
#include <utility>

#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"

#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/Clippers.h"

#include "sst/basic-blocks/mechanics/simd-ops.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

#include "sst/basic-blocks/tables/SincTableProvider.h"

namespace sst::effects::delay
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct Delay : core::EffectTemplateBase<FXConfig>
{
    enum delay_params
    {
        dly_time_left = 0,
        dly_time_right,
        dly_feedback,
        dly_crossfeed,
        dly_lowcut,
        dly_highcut,
        dly_mod_rate,
        dly_mod_depth,
        dly_input_channel,
        dly_reserved, // looks like this one got removed at one point
        dly_mix,
        dly_width,

        dly_num_params,
    };

    enum delay_clipping_modes
    {
        dly_clipping_off,
        dly_clipping_soft,
        dly_clipping_tanh,
        dly_clipping_hard,
        dly_clipping_hard18,

        num_dly_clipping_modes,
    };

    static constexpr int numParams{dly_num_params};
    static constexpr const char *streamingName{"delay"};
    static constexpr const char *displayName{"Dual Delay"};

    Delay(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
          typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p), lp(s), hp(s)
    {
        // We should no longer need this
        mix.set_blocksize(FXConfig::blockSize);
        pan.set_blocksize(FXConfig::blockSize);
        feedback.set_blocksize(FXConfig::blockSize);
        crossfeed.set_blocksize(FXConfig::blockSize);
    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return -1; }
    float silentSamplesLastCheck{-100000.f};
    size_t silentSamplesVal{0};
    size_t silentSamplesLength()
    {
        auto t1 = std::max(this->floatValue(dly_time_left) * this->temposyncRatio(dly_time_left),
                           this->floatValue(dly_time_right) * this->temposyncRatio(dly_time_right));
        if (t1 != silentSamplesLastCheck)
        {
            auto t = pow(2.f, t1);
            this->silentSamplesVal = (size_t)(1.1 * t * this->sampleRate());
            this->silentSamplesLastCheck = t1;
        }
        return silentSamplesVal;
    }
    void onSampleRateChanged() { initialize(); }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case dly_time_left:
            return pmd().withName("Left").asEnvelopeTime();

        case dly_time_right:
            return pmd().withName("Right").asEnvelopeTime().deactivatable();

        case dly_feedback:
            return pmd()
                .asPercentExtendableToBipolar()
                .withName("Feedback")
                .deformable()
                .withDeformationCount(num_dly_clipping_modes);

        case dly_crossfeed:
            return pmd()
                .asPercent()
                .withName("Crossfeed")
                .extendable()
                .withExtendFactors(2.f, -1.f)
                .withDefault(0.f);

        case dly_mod_rate:
            return pmd().asLfoRate().withName("Rate");

        case dly_mod_depth:
            return pmd()
                .withName("Depth")
                .withLinearScaleFormatting("cents", 100.f)
                .withType(pmd::FLOAT)
                .withRange(0.f, 2.f)
                .extendable()
                .withExtendFactors(6.f)
                .withDefault(0.f);

        case dly_lowcut:
            return pmd().asAudibleFrequency().withName("Low Cut").deactivatable().withDefault(
                -60.f);

        case dly_highcut:
            return pmd()
                .asAudibleFrequency()
                .withName("High Cut")
                .deactivatable()
                .withDefault(70.f);
        case dly_input_channel:
            return pmd()
                .asPercentBipolar()
                .withName("Channel")
                .withCustomMaxDisplay("100.00 % (Right)")
                .withCustomMinDisplay("-100.00 % (Left)")
                .withCustomDefaultDisplay("0.00 % (Stereo)");
        case dly_reserved:
            return pmd().withType(basic_blocks::params::ParamMetaData::NONE);

        case dly_mix:
            return pmd().withName("Mix").asPercent().withDefault(0.5f);
        case dly_width:
            return this->getWidthParam();
        }
        return {};
    }

  protected:
    inline float amp_to_linear(float x)
    {
        x = std::max(0.f, x);

        return x * x * x;
    }
    void setvars(bool b);

    /*
     * we use our own sinctable here. Since these effects
     * are rarely constructed and not used at a voice level it
     * is ok.
     */
    sst::basic_blocks::tables::SurgeSincTableProvider sincTable;

    // TODO - we've wanted a sample rate adjustable max for a while.
    // Still don't have it here.
    static constexpr int max_delay_length{1 << 18};
    typename core::EffectTemplateBase<FXConfig>::lipol_ps_blocksz feedback, crossfeed, aligpan, pan,
        mix, widthS, widthM;
    float buffer alignas(
        16)[2][max_delay_length + sst::basic_blocks::tables::SurgeSincTableProvider::FIRipol_N];

    sst::basic_blocks::dsp::SurgeLag<float, true> timeL{0.0001}, timeR{0.0001};
    bool inithadtempo;
    float envf;
    int wpos;
    typename core::EffectTemplateBase<FXConfig>::BiquadFilterType lp, hp;
    double lfophase;
    float LFOval;
    bool LFOdirection, FBsign;
    // int ringout_time;
  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

template <typename FXConfig> inline void Delay<FXConfig>::initialize()
{
    memset(buffer[0], 0, (max_delay_length + this->sincTable.FIRipol_N) * sizeof(float));
    memset(buffer[1], 0, (max_delay_length + this->sincTable.FIRipol_N) * sizeof(float));
    wpos = 0;
    lfophase = 0.0;
    // ringout_time = 100000;
    envf = 0.f;
    LFOval = 0.f;
    LFOdirection = true;
    FBsign = false;
    lp.suspend();
    hp.suspend();
    // See issue #1444 and the fix for this stuff
    inithadtempo = (this->temposyncInitialized());
    setvars(true);
    inithadtempo = (this->temposyncInitialized());
}

template <typename FXConfig> inline void Delay<FXConfig>::setvars(bool init)
{
    if (!inithadtempo && this->temposyncInitialized())
    {
        init = true;
        inithadtempo = true;
    }

    auto fbp = this->floatValue(dly_feedback);
    FBsign = false;

    if (this->isExtended(dly_feedback))
    {
        fbp = 2.f * fbp - 1.f;

        if (fbp < 0.f)
        {
            FBsign = true;
        }
    }

    float fb = amp_to_linear(std::fabs(fbp));
    float cf = amp_to_linear(this->floatValueExtended(dly_crossfeed));

    feedback.set_target_smoothed(fb);
    crossfeed.set_target_smoothed(cf);

    float lforate = this->envelopeRateLinear(-this->floatValue(dly_mod_rate)) *
                    this->temposyncRatio(dly_mod_rate);
    lfophase += lforate;

    if (lfophase > 0.5)
    {
        lfophase -= 1;
        LFOdirection = !LFOdirection;
    }

    float lfo_increment =
        (0.00000000001f + powf(2, this->floatValueExtended(dly_mod_depth) * (1.f / 12.f)) - 1.f) *
        FXConfig::blockSize;
    // small bias to avoid denormals

    const float ca = 0.99f;

    if (LFOdirection)
        LFOval = ca * LFOval + lfo_increment;
    else
        LFOval = ca * LFOval - lfo_increment;

    auto isLinked = this->isDeactivated(dly_time_right) ? dly_time_left : dly_time_right;

    constexpr int FIRoffset{sst::basic_blocks::tables::SurgeSincTableProvider::FIRipol_N >> 1};

    timeL.newValue(this->sampleRate() * this->temposyncRatioInv(dly_time_left) *
                       this->noteToPitchIgnoringTuning(12 * this->floatValue(dly_time_left)) +
                   LFOval - FIRoffset);
    timeR.newValue(this->sampleRate() * this->temposyncRatioInv(isLinked) *
                       this->noteToPitchIgnoringTuning(12 * this->floatValue(isLinked)) -
                   LFOval - FIRoffset);

    if (init)
    {
        timeL.instantize();
        timeR.instantize();
    }

    mix.set_target_smoothed(this->floatValue(dly_mix));
    this->setWidthTarget(widthS, widthM, dly_width);
    pan.set_target_smoothed(std::clamp(this->floatValue(dly_input_channel), -1.f, 1.f));

    lp.coeff_LP2B(lp.calc_omega(this->floatValue(dly_highcut) / 12.0), 0.707);
    hp.coeff_HP(hp.calc_omega(this->floatValue(dly_lowcut) / 12.0), 0.707);

    if (init)
    {
        timeL.instantize();
        timeR.instantize();
        feedback.instantize();
        crossfeed.instantize();
        mix.instantize();
        widthS.instantize();
        widthM.instantize();
        pan.instantize();
        lp.coeff_instantize();
        hp.coeff_instantize();
    }
}

template <typename FXConfig> inline void Delay<FXConfig>::processBlock(float *dataL, float *dataR)

{
    setvars(false);

    int k;
    float tbufferL alignas(16)[FXConfig::blockSize],
        wbL alignas(16)[FXConfig::blockSize]; // wb = write-buffer
    float tbufferR alignas(16)[FXConfig::blockSize], wbR alignas(16)[FXConfig::blockSize];

    for (k = 0; k < FXConfig::blockSize; k++)
    {
        timeL.process();
        timeR.process();

        int i_dtimeL =
            std::max((int)FXConfig::blockSize,
                     std::min((int)timeL.v, (int)(max_delay_length - sincTable.FIRipol_N - 1)));
        int i_dtimeR =
            std::max((int)FXConfig::blockSize,
                     std::min((int)timeR.v, (int)(max_delay_length - sincTable.FIRipol_N - 1)));

        int rpL = ((wpos - i_dtimeL + k) - sincTable.FIRipol_N) & (max_delay_length - 1);
        int rpR = ((wpos - i_dtimeR + k) - sincTable.FIRipol_N) & (max_delay_length - 1);

        int sincL = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeL + 1) - timeL.v)), 0,
                               sincTable.FIRipol_M - 1);
        int sincR = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeR + 1) - timeR.v)), 0,
                               sincTable.FIRipol_M - 1);

        SIMD_M128 L, R;
        L = SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincL]),
                            SIMD_MM(loadu_ps)(&buffer[0][rpL]));
        L = SIMD_MM(add_ps)(L, SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincL + 4]),
                                               SIMD_MM(loadu_ps)(&buffer[0][rpL + 4])));
        L = SIMD_MM(add_ps)(L, SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincL + 8]),
                                               SIMD_MM(loadu_ps)(&buffer[0][rpL + 8])));
        L = sst::basic_blocks::mechanics::sum_ps_to_ss(L);
        R = SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincR]),
                            SIMD_MM(loadu_ps)(&buffer[1][rpR]));
        R = SIMD_MM(add_ps)(R, SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincR + 4]),
                                               SIMD_MM(loadu_ps)(&buffer[1][rpR + 4])));
        R = SIMD_MM(add_ps)(R, SIMD_MM(mul_ps)(SIMD_MM(load_ps)(&sincTable.sinctable1X[sincR + 8]),
                                               SIMD_MM(loadu_ps)(&buffer[1][rpR + 8])));
        R = sst::basic_blocks::mechanics::sum_ps_to_ss(R);

        SIMD_MM(store_ss)(&tbufferL[k], L);
        SIMD_MM(store_ss)(&tbufferR[k], R);
    }

    // negative feedback
    if (FBsign)
    {
        mech::mul_block<FXConfig::blockSize>(tbufferL, -1.f);
        mech::mul_block<FXConfig::blockSize>(tbufferR, -1.f);
    }

    // feedback path clipping modes
    switch (this->deformType(dly_feedback))
    {
    case dly_clipping_soft:
        sdsp::softclip_block<FXConfig::blockSize>(tbufferL);
        sdsp::softclip_block<FXConfig::blockSize>(tbufferR);
        break;
    case dly_clipping_tanh:
        sdsp::tanh7_block<FXConfig::blockSize>(tbufferL);
        sdsp::tanh7_block<FXConfig::blockSize>(tbufferR);
        break;
    case dly_clipping_hard:
        sdsp::hardclip_block<FXConfig::blockSize>(tbufferL);
        sdsp::hardclip_block<FXConfig::blockSize>(tbufferR);
        break;
    case dly_clipping_hard18:
        sdsp::hardclip_block8<FXConfig::blockSize>(tbufferL);
        sdsp::hardclip_block8<FXConfig::blockSize>(tbufferR);
        break;
    case dly_clipping_off:
    default:
        break;
    }

    if (!this->isDeactivated(dly_highcut))
    {
        lp.process_block(tbufferL, tbufferR);
    }

    if (!this->isDeactivated(dly_lowcut))
    {
        hp.process_block(tbufferL, tbufferR);
    }

    pan.trixpan_blocks(dataL, dataR, wbL, wbR, this->blockSize_quad);

    feedback.MAC_2_blocks_to(tbufferL, tbufferR, wbL, wbR, this->blockSize_quad);
    crossfeed.MAC_2_blocks_to(tbufferL, tbufferR, wbR, wbL, this->blockSize_quad);

    if (wpos + FXConfig::blockSize >= max_delay_length)
    {
        for (k = 0; k < FXConfig::blockSize; k++)
        {
            buffer[0][(wpos + k) & (max_delay_length - 1)] = wbL[k];
            buffer[1][(wpos + k) & (max_delay_length - 1)] = wbR[k];
        }
    }
    else
    {
        mech::copy_from_to<FXConfig::blockSize>(wbL, &buffer[0][wpos]);
        mech::copy_from_to<FXConfig::blockSize>(wbR, &buffer[1][wpos]);
    }

    if (wpos == 0)
    {
        // copy buffer so FIR-core doesn't have to wrap
        for (k = 0; k < sincTable.FIRipol_N; k++)
        {
            buffer[0][k + max_delay_length] = buffer[0][k];
            buffer[1][k + max_delay_length] = buffer[1][k];
        }
    }

    // scale width
    this->applyWidth(tbufferL, tbufferR, widthS, widthM);

    mix.fade_2_blocks_inplace(dataL, tbufferL, dataR, tbufferR, this->blockSize_quad);

    wpos += FXConfig::blockSize;
    wpos = wpos & (max_delay_length - 1);
}
} // namespace sst::effects::delay
#endif // SURGE_DELAY_H
