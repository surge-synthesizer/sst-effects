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

#ifndef INCLUDE_SST_EFFECTS_FLOATY_DELAY_H
#define INCLUDE_SST_EFFECTS_FLOATY_DELAY_H

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

#include "sst/filters/CytomicSVF.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::effects::floatydelay
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct FloatyDelay : core::EffectTemplateBase<FXConfig>
{
    enum floaty_params
    {
        fld_mix = 0,
        fld_time,
        fld_offset,
        fld_feedback,
        fld_warp_rate,
        fld_warp_depth,
        fld_cutoff,
        fld_resonance,
        fld_playrate,
        fld_playdir,

        fld_num_params,
    };

    static constexpr int numParams{fld_num_params};
    static constexpr const char *effectName{"Floaty Delay"};

    basic_blocks::dsp::RNG rng;

    FloatyDelay(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return -1; }
    void onSampleRateChanged() { initialize(); }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fld_mix:
            return pmd().withName("Mix").asPercent().withDefault(0.3f);

        case fld_time:
            return pmd()
                .asFloat()
                .withRange(20.f, 8000.f)
                .withDefault(300.f)
                .withLinearScaleFormatting("ms", 1)
                .withName("Time");

        case fld_offset:
            return pmd()
                .asFloat()
                .withRange(-10.f, 10.f)
                .withDefault(0.f)
                .withLinearScaleFormatting("ms", 1.f)
                .withName("L/R Offset");

        case fld_feedback:
            return pmd().asPercent().withDefault(.5f).withName("Feedback");

        case fld_warp_rate:
            return pmd().asLfoRate().withName("Rate");

        case fld_warp_depth:
            return pmd()
                .asFloat()
                .withRange(0.f, 4.f)
                .withDefault(0.f)
                .withLinearScaleFormatting("%", 1.f)
                .withName("Depth");

        case fld_cutoff:
            return pmd().asAudibleFrequency().withDefault(0.f).withName("Cutoff");

        case fld_resonance:
            return pmd().asPercent().withName("Resonance").withDefault(.7f);

        case fld_playrate:
            return pmd()
                .asInt()
                .withRange(0, 3)
                .withName("Playrate")
                .withDefault(1)
                .withUnorderedMapFormatting({{0, "0.5"}, {1, "1"}, {2, "1.5"}, {3, "2"}});
        case fld_playdir:
            return pmd()
                .asBool()
                .withName("Direction")
                .withDefault(true)
                .withUnorderedMapFormatting({{false, "Reverse"}, {true, "Forwards"}});
        }
        return {};
    }
    int samplerate = this->sampleRate();
    float envelope_rate_linear_nowrap(float f)
    {
        return 1.f * FXConfig::blockSize / samplerate * std::pow(-2, f);
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
    static constexpr int max_delay_length{1 << 16};
    sst::basic_blocks::dsp::lipol_sse<FXConfig::blockSize, true> timeLerpL, timeLerpR;
    typename core::EffectTemplateBase<FXConfig>::lipol_ps_blocksz feedback, mix;
    float buffer alignas(
        16)[2][max_delay_length + sst::basic_blocks::tables::SurgeSincTableProvider::FIRipol_N];

    sst::filters::CytomicSVF filter;

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<FloatyDelay, FXConfig::blockSize>;
    lfo_t sineLFO{this, rng};
    lfo_t noiseLFO{this, rng};
    typename lfo_t::Shape sine = lfo_t::Shape::SINE;
    typename lfo_t::Shape noise = lfo_t::Shape::SMOOTH_NOISE;

    bool inithadtempo;
    float envf;
    int wpos;
    float sampleRateInv = 1 / this->sampleRate();

    float playbackRate{1.f};
    static constexpr int osw = 128; // overlap window
    // int ringout_time;
};

template <typename FXConfig> inline void FloatyDelay<FXConfig>::initialize()
{
    memset(buffer[0], 0, (max_delay_length + this->sincTable.FIRipol_N) * sizeof(float));
    memset(buffer[1], 0, (max_delay_length + this->sincTable.FIRipol_N) * sizeof(float));
    wpos = 0;
    // ringout_time = 100000;
    envf = 0.f;
    filter.init();
    // See issue #1444 and the fix for this stuff
    inithadtempo = (this->temposyncInitialized());
    setvars(true);
    inithadtempo = (this->temposyncInitialized());
}

template <typename FXConfig> inline void FloatyDelay<FXConfig>::setvars(bool init)
{
    if (!inithadtempo && this->temposyncInitialized())
    {
        init = true;
        inithadtempo = true;
    }
    if (init)
    {
        timeLerpL.instantize();
        timeLerpR.instantize();
        feedback.instantize();
        mix.instantize();
    }

    auto fbp = this->floatValue(fld_feedback);
    float fb = amp_to_linear(std::fabs(fbp));
    feedback.set_target_smoothed(fb);

    auto wr = this->floatValue(fld_warp_rate);
    auto wd = this->floatValue(fld_warp_depth) * (this->floatValue(fld_time) * .25);

    sineLFO.process_block(wr, 0.f, sine);
    noiseLFO.process_block(wr * 2.f, 0.f, noise);
    auto mod = wd * (sineLFO.lastTarget + noiseLFO.lastTarget * 0.1f);

    auto tL = this->floatValue(fld_time) - this->floatValue(fld_offset);
    auto tR = this->floatValue(fld_time) + this->floatValue(fld_offset);

    tL *= mod;
    tR *= mod;

    tL = std::clamp(tL, 10.f, 8000.f);
    tR = std::clamp(tR, 10.f, 8000.f);

    timeLerpL.set_target(tL * sampleRateInv);
    timeLerpR.set_target(tR * sampleRateInv);

    switch (this->intValue(fld_playrate))
    {
    case (0):
        playbackRate = 0.5f;
        break;
    case (1):
        playbackRate = 1.f;
        break;
    case (2):
        playbackRate = 1.5f;
        break;
    case (3):
        playbackRate = 2.f;
        break;
    default:
        playbackRate = 1.f;
        break;
    }

    filter.template setCoeffForBlock<FXConfig::blockSize>(
        sst::filters::CytomicSVF::LP, this->floatValue(fld_cutoff), this->floatValue(fld_resonance),
        1 / this->sampleRate(), 0.f);

    mix.set_target_smoothed(this->floatValue(fld_mix));
}

template <typename FXConfig>
inline void FloatyDelay<FXConfig>::processBlock(float *dataL, float *dataR)

{
    setvars(false);

    int k;
    // wb = write-buffer
    float tbufferL alignas(16)[FXConfig::blockSize], wbL alignas(16)[FXConfig::blockSize];
    float tbufferR alignas(16)[FXConfig::blockSize], wbR alignas(16)[FXConfig::blockSize];

    float timeL alignas(16)[FXConfig::blockSize];
    float timeR alignas(16)[FXConfig::blockSize];
    timeLerpL.store_block(timeL);
    timeLerpR.store_block(timeR);

    for (k = 0; k < FXConfig::blockSize; k++)
    {
        int i_dtimeL =
            std::max((int)FXConfig::blockSize,
                     std::min((int)timeL[k], (int)(max_delay_length - sincTable.FIRipol_N - 1)));
        int i_dtimeR =
            std::max((int)FXConfig::blockSize,
                     std::min((int)timeR[k], (int)(max_delay_length - sincTable.FIRipol_N - 1)));
        // Read position
        int rpL = (int)((wpos - i_dtimeL + k) - sincTable.FIRipol_N) & (max_delay_length - 1);
        int rpR = (int)((wpos - i_dtimeR + k) - sincTable.FIRipol_N) & (max_delay_length - 1);

        // ols = overlap smoothing, osw = overlap smooting window
        // In repeat and/or non-1 playrate, read and write will approach each other,
        // turn the delay output down gradually in a little window around the approach.
        // float olsL = (std::abs(rpL - wpos) > osw) ? 1 : std::abs(rpL - wpos) / osw;
        // float olsR = (std::abs(rpR - wpos) > osw) ? 1 : std::abs(rpL - wpos) / osw;

        int sincL = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeL + 1) - timeL[k])), 0,
                               sincTable.FIRipol_M - 1);
        int sincR = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeR + 1) - timeR[k])), 0,
                               sincTable.FIRipol_M - 1);

        //
        __m128 L, R;
        L = _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincL]), _mm_loadu_ps(&buffer[0][rpL]));
        L = _mm_add_ps(L, _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincL + 4]),
                                     _mm_loadu_ps(&buffer[0][rpL + 4])));
        L = _mm_add_ps(L, _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincL + 8]),
                                     _mm_loadu_ps(&buffer[0][rpL + 8])));
        L = sst::basic_blocks::mechanics::sum_ps_to_ss(L);
        R = _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincR]), _mm_loadu_ps(&buffer[1][rpR]));
        R = _mm_add_ps(R, _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincR + 4]),
                                     _mm_loadu_ps(&buffer[1][rpR + 4])));
        R = _mm_add_ps(R, _mm_mul_ps(_mm_load_ps(&sincTable.sinctable1X[sincR + 8]),
                                     _mm_loadu_ps(&buffer[1][rpR + 8])));
        R = sst::basic_blocks::mechanics::sum_ps_to_ss(R);

        _mm_store_ss(&tbufferL[k], L);
        _mm_store_ss(&tbufferR[k], R);

        // * olsL etc
    }

    filter.template processBlock<FXConfig::blockSize>(tbufferL, tbufferR, tbufferL, tbufferR);

    sdsp::tanh7_block<FXConfig::blockSize>(tbufferL);
    sdsp::tanh7_block<FXConfig::blockSize>(tbufferR);

    feedback.MAC_2_blocks_to(tbufferL, tbufferR, wbL, wbR, this->blockSize_quad);

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

    mix.fade_2_blocks_inplace(dataL, tbufferL, dataR, tbufferR, this->blockSize_quad);

    wpos += FXConfig::blockSize;
    wpos = wpos & (max_delay_length - 1);
}
} // namespace sst::effects::floatydelay
#endif // FLOATYDELAY_H
