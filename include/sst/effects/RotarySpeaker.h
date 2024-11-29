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

#ifndef INCLUDE_SST_EFFECTS_ROTARYSPEAKER_H
#define INCLUDE_SST_EFFECTS_ROTARYSPEAKER_H

#include "sst/waveshapers.h"
#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/LanczosResampler.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"

namespace sst::effects::rotaryspeaker
{
template <typename FXConfig> struct RotarySpeaker : core::EffectTemplateBase<FXConfig>
{
    using BiquadFilter = typename core::EffectTemplateBase<FXConfig>::BiquadFilterType;

    enum rotary_params
    {
        rot_horn_rate = 0,
        rot_doppler,
        rot_tremolo,
        rot_rotor_rate,
        rot_drive,
        rot_waveshape,
        rot_width,
        rot_mix,

        rot_num_params,
    };

    static constexpr int n_waveShapers = 8;
    static constexpr std::array<sst::waveshapers::WaveshaperType, n_waveShapers> waveShapers = {
        sst::waveshapers::WaveshaperType::wst_soft, sst::waveshapers::WaveshaperType::wst_hard,
        sst::waveshapers::WaveshaperType::wst_asym, sst::waveshapers::WaveshaperType::wst_sine,
        sst::waveshapers::WaveshaperType::wst_digital, // If you adjust this list above here, you
                                                       // break 1.9 patch compat
        sst::waveshapers::WaveshaperType::wst_ojd, sst::waveshapers::WaveshaperType::wst_fwrectify,
        sst::waveshapers::WaveshaperType::wst_fuzzsoft};

    static constexpr int numParams{rot_num_params};
    static constexpr const char *effectName{"rotaryspeaker"};

    typename core::EffectTemplateBase<FXConfig>::lipol_ps_blocksz width alignas(16), mix
        alignas(16);
    sst::waveshapers::QuadWaveshaperState wsState alignas(16);

    RotarySpeaker(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                  typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p), xover(s), lowbass(s)
    {
        mix.set_blocksize(FXConfig::blockSize);
        width.set_blocksize(FXConfig::blockSize);
    }
    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return 1000; }
    void onSampleRateChanged() { initialize(); }

    void setvars(bool init);
    void processOnlyControl();

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numParams);
        using pmd = basic_blocks::params::ParamMetaData;
        auto result = pmd().withName("Unknown " + std::to_string(idx));

        switch ((rotary_params)idx)
        {
        case rot_horn_rate:
            return result.asLfoRate(-7, 9).withName("Horn Rate");
        case rot_doppler:
            return result.asPercent().withName("Doppler");
        case rot_tremolo:
            return result.asPercent().withName("Tremolo");
        case rot_rotor_rate:
            return result.asPercent().withRange(0, 2).withName("Rotor Rate");
        case rot_drive:
            return result.deactivatable().asPercent().withDefault(0).withName("Drive");
        case rot_waveshape:
        {
            std::unordered_map<int, std::string> names;
            for (int i = 0; i < n_waveShapers; ++i)
            {
                names[i] = sst::waveshapers::wst_names[(int)waveShapers[i]];
            }
            return result.withType(pmd::INT)
                .withName("Model")
                .withDefault(0)
                .withRange(0, n_waveShapers - 1)
                .withUnorderedMapFormatting(names);
        }
        case rot_width:
            return this->getWidthParam();
        case rot_mix:
            return result.asPercent().withDefault(0.33f).withName("Mix");
        case rot_num_params:
            return result;
        }
        return result;
    };

  protected:
    static constexpr int maxDelayLength{1 << 18};
    float buffer[maxDelayLength];
    int wpos;
    // filter *lp[2],*hp[2];
    // biquadunit rotor_lpL,rotor_lpR;
    BiquadFilter xover, lowbass;
    // float
    // f_rotor_lp[2][n_filter_parameters],f_xover[n_filter_parameters],f_lowbass[n_filter_parameters];

    using quadr_osc = sst::basic_blocks::dsp::SurgeQuadrOsc<float>;
    quadr_osc lfo;
    quadr_osc lf_lfo;
    sst::basic_blocks::dsp::lipol<float, FXConfig::blockSize, true> dL, dR, hornamp[2];
    sst::basic_blocks::dsp::SurgeLag<float, true> drive;
    bool first_run;
    sst::basic_blocks::tables::SurgeSincTableProvider sincTable;

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

template <typename FXConfig> inline void RotarySpeaker<FXConfig>::initialize()
{
    memset(buffer, 0, maxDelayLength * sizeof(float));

    wpos = 0;

    xover.suspend();
    lowbass.suspend();

    xover.coeff_LP2B(xover.calc_omega(0.862496f), 0.707); // 800 Hz
    lowbass.coeff_LP2B(xover.calc_omega(-1.14f), 0.707);  // 200 Hz

    setvars(true);
}

template <typename FXConfig> inline void RotarySpeaker<FXConfig>::setvars(bool init)
{
    drive.newValue(this->floatValue(rot_drive));
    width.set_target_smoothed(this->dbToLinear(this->floatValue(rot_width)));
    mix.set_target_smoothed(this->floatValue(rot_mix));

    if (init)
    {
        drive.instantize();
        width.instantize();
        mix.instantize();

        for (int i = 0; i < sst::waveshapers::n_waveshaper_registers; ++i)
            wsState.R[i] = SIMD_MM(setzero_ps)();
    }
}

template <typename FXConfig> inline void RotarySpeaker<FXConfig>::processOnlyControl()
{
    double frate = this->floatValue(rot_horn_rate) * this->temposyncRatio(rot_horn_rate);

    lfo.set_rate(2 * M_PI * powf(2, frate) * 1 / this->sampleRate() * FXConfig::blockSize);
    lf_lfo.set_rate(this->floatValue(rot_rotor_rate) * 2 * M_PI * powf(2, frate) * 1 /
                    this->sampleRate() * FXConfig::blockSize);

    lfo.process();
    lf_lfo.process();
}

template <typename FXConfig>
inline void RotarySpeaker<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    setvars(false);

    double frate = this->floatValue(rot_horn_rate) * this->temposyncRatio(rot_horn_rate);
    /*
     ** lf_lfo.process drives the sub-frequency and processes inside the iteration over samples>
     ** lfo.process drives the speaker and processes outside the iteration
     ** therefore lf_lfo processes FXConfig::blockSize more times
     ** hence the lack of FXConfig::blockSize here
     */

    lfo.set_rate(2 * M_PI * powf(2, frate) * 1 / this->sampleRate() * FXConfig::blockSize);
    lf_lfo.set_rate(this->floatValue(rot_rotor_rate) * 2 * M_PI * powf(2, frate) * 1 /
                    this->sampleRate());

    float precalc0 = (-2 - (float)lfo.i);
    float precalc1 = (-1 - (float)lfo.r);
    float precalc2 = (+1 - (float)lfo.r);
    float lenL = sqrt(precalc0 * precalc0 + precalc1 * precalc1);
    float lenR = sqrt(precalc0 * precalc0 + precalc2 * precalc2);

    float delay = this->sampleRate() * 0.0018f * this->floatValue(rot_doppler);

    dL.newValue(delay * lenL);
    dR.newValue(delay * lenR);

    float dotp_L = (precalc1 * (float)lfo.r + precalc0 * (float)lfo.i) / lenL;
    float dotp_R = (precalc2 * (float)lfo.r + precalc0 * (float)lfo.i) / lenR;

    float a = this->floatValue(rot_tremolo) * 0.6f;

    hornamp[0].newValue((1.f - a) + a * dotp_L);
    hornamp[1].newValue((1.f - a) + a * dotp_R);

    lfo.process();

    float upper alignas(16)[FXConfig::blockSize];
    float lower alignas(16)[FXConfig::blockSize];
    float lower_sub alignas(16)[FXConfig::blockSize];
    float tbufferL alignas(16)[FXConfig::blockSize];
    float tbufferR alignas(16)[FXConfig::blockSize];
    float wbL alignas(16)[FXConfig::blockSize];
    float wbR alignas(16)[FXConfig::blockSize];

    int k;

    drive.newValue(this->floatValue(rot_drive));

    auto wsi = this->intValue(rot_waveshape);

    if (wsi < 0 || wsi >= n_waveShapers)
    {
        wsi = 0;
    }

    auto ws = waveShapers[wsi];

    /*
    ** This is a set of completely empirical scaling settings to offset gain being too crazy
    ** in the drive cycle. There's no science really, just us playing with it and listening
    */
    float gain_tweak{1.f}, compensate{1.f}, drive_factor{1.f}, gain_comp_factor{1.0};
    float compensateStartsAt = 0.18;
    bool square_drive_comp = false;

    switch (ws)
    {
    case sst::waveshapers::WaveshaperType::wst_hard:
    {
        gain_tweak = 1.4;
        compensate = 3.f;
    }
    case sst::waveshapers::WaveshaperType::wst_asym:
    {
        gain_tweak = 1.15;
        compensate = 9.f;
        compensateStartsAt = 0.05;
        break;
    }
    case sst::waveshapers::WaveshaperType::wst_sine:
    {
        gain_tweak = 4.4;
        compensate = 10.f;
        compensateStartsAt = 0.f;
        square_drive_comp = true;
        break;
    }
    case sst::waveshapers::WaveshaperType::wst_digital:
    {
        gain_tweak = 1.f;
        compensate = 4.f;
        compensateStartsAt = 0.f;
        break;
    }
    case sst::waveshapers::WaveshaperType::wst_fwrectify:
    case sst::waveshapers::WaveshaperType::wst_fuzzsoft:
    {
        gain_tweak = 1.f;
        compensate = 2.f;
        compensateStartsAt = 0.f;
        break;
    }
    default:
    {
        gain_tweak = 1.f;
        compensate = 4.f;
        break;
    }
    }

    if (!this->isDeactivated(rot_drive))
    {
        drive_factor = 1.f + (drive.v * drive.v * 15.f);

        if (drive.v < compensateStartsAt)
            gain_comp_factor = 1.0;
        else if (square_drive_comp)
            gain_comp_factor = 1.f + (((drive.v * drive.v) - compensateStartsAt) * compensate);
        else
            gain_comp_factor = 1.f + ((drive.v - compensateStartsAt) * compensate);
    }

    // FX waveshapers have value at wst_soft for 0; so don't add wst_soft here (like we did
    // in 1.9)
    bool useSSEShaper = (ws >= sst::waveshapers::WaveshaperType::wst_sine);
    auto wsop = sst::waveshapers::GetQuadWaveshaper(ws);

    for (k = 0; k < FXConfig::blockSize; k++)
    {
        float input;

        if (!this->isDeactivated(rot_drive))
        {
            drive_factor = 1.f + (drive.v * drive.v * 15.f);
            if (useSSEShaper)
            {
                auto inp = SIMD_MM(set1_ps)(0.5 * (dataL[k] + dataR[k]));
                auto wsres = wsop(&wsState, inp, SIMD_MM(set1_ps)(drive_factor));
                float r[4];
                SIMD_MM(store_ps)(r, wsres);
                input = r[0] * gain_tweak;
            }
            else
            {
                input = sst::waveshapers::lookup_waveshape(ws, 0.5f * (dataL[k] + dataR[k]) *
                                                                   drive_factor) *
                        gain_tweak;
            }
            input /= gain_comp_factor;

            drive.process();
        }
        else
        {
            input = 0.5f * (dataL[k] + dataR[k]);
        }

        upper[k] = input;
        lower[k] = input;
    }

    xover.process_block(lower);

    for (k = 0; k < FXConfig::blockSize; k++)
    {
        // feed delay input
        int wp = (wpos + k) & (maxDelayLength - 1);
        lower_sub[k] = lower[k];
        upper[k] -= lower[k];
        buffer[wp] = upper[k];

        int i_dtimeL = std::max<int>(FXConfig::blockSize,
                                     std::min((int)dL.v, maxDelayLength - sincTable.FIRipol_N - 1));
        int i_dtimeR = std::max<int>(FXConfig::blockSize,
                                     std::min((int)dR.v, maxDelayLength - sincTable.FIRipol_N - 1));

        int rpL = (wpos - i_dtimeL + k);
        int rpR = (wpos - i_dtimeR + k);

        assert(sincTable.FIRipol_M - 1 > 0);
        int sincL = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeL + 1) - dL.v)), 0,
                               sincTable.FIRipol_M - 1);
        int sincR = sincTable.FIRipol_N *
                    std::clamp((int)(sincTable.FIRipol_M * (float(i_dtimeR + 1) - dR.v)), 0,
                               sincTable.FIRipol_M - 1);

        // get delay output
        tbufferL[k] = 0;
        tbufferR[k] = 0;
        for (int i = 0; i < sincTable.FIRipol_N; i++)
        {
            tbufferL[k] += buffer[(rpL - i) & (maxDelayLength - 1)] *
                           sincTable.sinctable1X[sincL + sincTable.FIRipol_N - i];
            tbufferR[k] += buffer[(rpR - i) & (maxDelayLength - 1)] *
                           sincTable.sinctable1X[sincR + sincTable.FIRipol_N - i];
        }
        dL.process();
        dR.process();
    }

    lowbass.process_block(lower_sub);

    for (k = 0; k < FXConfig::blockSize; k++)
    {
        lower[k] -= lower_sub[k];

        float bass = lower_sub[k] + lower[k] * (lf_lfo.r * 0.6f + 0.3f);

        wbL[k] = hornamp[0].v * tbufferL[k] + bass;
        wbR[k] = hornamp[1].v * tbufferR[k] + bass;

        lf_lfo.process();
        hornamp[0].process();
        hornamp[1].process();
    }

    // scale width
    this->applyWidth(wbL, wbR, width);

    mix.fade_2_blocks_inplace(dataL, wbL, dataR, wbR, FXConfig::blockSize >> 2);

    wpos += FXConfig::blockSize;
    wpos = wpos & (maxDelayLength - 1);
}
} // namespace sst::effects::rotaryspeaker
#endif // INCLUDE_SST_EFFECTS_ROTARYSPEAKER_H
