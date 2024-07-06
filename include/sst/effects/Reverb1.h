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
#ifndef INCLUDE_SST_EFFECTS_REVERB1_H
#define INCLUDE_SST_EFFECTS_REVERB1_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/filters/BiquadFilter.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"

namespace sst::effects::reverb1
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct Reverb1 : core::EffectTemplateBase<FXConfig>
{
    enum rev1_params
    {
        rev1_predelay = 0,
        rev1_shape,
        rev1_roomsize,
        rev1_decaytime,
        rev1_damping,
        rev1_lowcut,
        rev1_freq1,
        rev1_gain1,
        rev1_highcut,
        rev1_mix,
        rev1_width,
        // rev1_variation,
    };

    static constexpr int numParams{rev1_width + 1};
    static constexpr const char *effectName{"reverb1"};

    Reverb1(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
            typename FXConfig::ValueStorage *p);

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return ringout_time; }
    void onSampleRateChanged() { initialize(); }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numParams);
        using pmd = basic_blocks::params::ParamMetaData;
        auto result = pmd().withName("Unknown " + std::to_string(idx));
        // Once this switch is off we can turn on the configureControlsFromFXMetadata in
        // Reverb1Effect.cpp::init_ctrltypes

        switch ((rev1_params)idx)
        {
        case rev1_predelay:
            return result.withName("Pre-Delay").asEnvelopeTime().withDefault(-4.f);
        case rev1_shape:
            return result.withName("Room Shape")
                .withType(pmd::INT)
                .withRange(0, 3)
                .withDefault(0)
                .withUnorderedMapFormatting(
                    {{0, "Type 1"}, {1, "Type 2"}, {2, "Type 3"}, {3, "Type 4"}});
        case rev1_roomsize:
            return result.withName("Size").asPercent().withDefault(0.5);
        case rev1_decaytime:
            return result.withName("Decay Time")
                .withRange(-4, 6)
                .withDefault(1.f)
                .withLog2SecondsFormatting();
        case rev1_damping:
            return result.withName("HF Damping").asPercent().withDefault(0.2f);
        case rev1_lowcut:
            return result.withName("Low Cut").asAudibleFrequency().deactivatable().withDefault(
                -24.f);
        case rev1_freq1:
            return result.withName("Peak Freq").asAudibleFrequency().withDefault(0.f);
        case rev1_gain1:
            return result.withName("Peak Gain").asDecibel().withDefault(0.f);
        case rev1_highcut:
            return result.withName("High Cut")
                .asAudibleFrequency()
                .deactivatable()
                .withDefault(72.f);
        case rev1_mix:
            return result.withName("Mix").asPercent().withDefault(0.5f);
        case rev1_width:
            return this->getWidthParam();
        default:
            break;
        }

        return result;
    }

  protected:
    static constexpr int revbits = 15;
    static constexpr int max_rev_dly = 1 << revbits;
    static constexpr int rev_tap_bits = 4;
    static constexpr int rev_taps = 1 << rev_tap_bits;

    float delay_pan_L alignas(16)[rev_taps], delay_pan_R alignas(16)[rev_taps];
    float delay_fb alignas(16)[rev_taps];
    float delay alignas(16)[rev_taps * max_rev_dly];
    float out_tap alignas(16)[rev_taps];
    float predelay alignas(16)[max_rev_dly];
    int delay_time alignas(16)[rev_taps];
    typename core::EffectTemplateBase<FXConfig>::lipol_ps_blocksz mix, width;

    void update_rtime();
    void update_rsize() { loadpreset(shape); }
    void clear_buffers()
    {
        mech::clear_block<max_rev_dly>(predelay);
        mech::clear_block<rev_taps * max_rev_dly>(delay);
    }

    void loadpreset(int id);
    /*int delay_time_mod[rev_taps];
    int delay_time_dv[rev_taps];*/
    int delay_pos{0};
    /*	float noise[rev_taps];
            float noise_target[rev_taps];*/
    double modphase{0.0};
    int shape{0};
    float lastf[numParams];
    typename core::EffectTemplateBase<FXConfig>::BiquadFilterType band1, locut, hicut;
    int ringout_time;
    int b{0};

    const float db60 = powf(10.f, 0.05f * -60.f);
};

template <typename FXConfig>
Reverb1<FXConfig>::Reverb1(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                           typename FXConfig::ValueStorage *p)
    : core::EffectTemplateBase<FXConfig>(s, e, p), band1(s), locut(s), hicut(s)
{
}

template <typename FXConfig> inline void Reverb1<FXConfig>::initialize()
{
    band1.coeff_peakEQ(band1.calc_omega(this->floatValue(rev1_freq1) / 12.f), 2,
                       this->floatValue(rev1_gain1));
    locut.coeff_HP(locut.calc_omega(this->floatValue(rev1_lowcut) / 12.f), 0.5);
    hicut.coeff_LP2B(hicut.calc_omega(this->floatValue(rev1_highcut) / 12.f), 0.5);
    band1.coeff_instantize();
    locut.coeff_instantize();
    hicut.coeff_instantize();
    band1.suspend();
    locut.suspend();
    hicut.suspend();

    ringout_time = 10000000;
    b = 0;

    loadpreset(0);
    modphase = 0;
    update_rsize();
    mix.set_target(1.f); // Should be the smoothest
    mix.instantize();

    width.set_target(1.f); // Should be the smoothest
    width.instantize();

    for (int t = 0; t < rev_taps; t++)
    {
        float x = (float)t / (rev_taps - 1.f);
        float xbp = -1.f + 2.f * x;

        out_tap[t] = 0;
        delay_pan_L[t] = sqrt(0.5 - 0.495 * xbp);
        delay_pan_R[t] = sqrt(0.5 + 0.495 * xbp);
    }
    delay_pos = 0;

    clear_buffers();
}

template <typename FXConfig>
inline void Reverb1<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    float wetL alignas(16)[FXConfig::blockSize], wetR alignas(16)[FXConfig::blockSize];

    if (this->intValue(rev1_shape) != shape)
        loadpreset(this->intValue(rev1_shape));
    if ((b == 0) && (fabs(this->floatValue(rev1_roomsize) - lastf[rev1_roomsize]) > 0.001f))
        loadpreset(shape);
    //	if(fabs(this->floatValue(rev1_variation) - lastf[rev1_variation]) > 0.001f) update_rsize();
    if (fabs(this->floatValue(rev1_decaytime) - lastf[rev1_decaytime]) > 0.001f)
        update_rtime();

    // do more seldom
    if (b == 0)
    {
        band1.coeff_peakEQ(band1.calc_omega(this->floatValue(rev1_freq1) * (1.f / 12.f)), 2,
                           this->floatValue(rev1_gain1));
        locut.coeff_HP(locut.calc_omega(this->floatValue(rev1_lowcut) * (1.f / 12.f)), 0.5);
        hicut.coeff_LP2B(hicut.calc_omega(this->floatValue(rev1_highcut) * (1.f / 12.f)), 0.5);
    }
    b = (b + 1) & 31;

    mix.set_target_smoothed(this->floatValue(rev1_mix));
    this->setWidthTarget(width, rev1_width);

    int pdtime = (int)(float)this->sampleRate() *
                 this->noteToPitchIgnoringTuning(12 * this->floatValue(rev1_predelay)) *
                 this->temposyncRatio(rev1_predelay);

    const __m128 one4 = _mm_set1_ps(1.f);
    float dv = this->floatValue(rev1_damping);

    dv = std::clamp(dv, 0.01f, 0.99f); // this is a simple one-pole damper, w * y[n] + ( 1-w )
                                       // y[n-1] so to be stable has to stay in range
    __m128 damp4 = _mm_load1_ps(&dv);
    __m128 damp4m1 = _mm_sub_ps(one4, damp4);

    for (int k = 0; k < FXConfig::blockSize; k++)
    {
        for (int t = 0; t < rev_taps; t += 4)
        {
            int dp = (delay_pos - (delay_time[t] >> 8));
            // float newa = delay[t + ((dp & (max_rev_dly-1))<<rev_tap_bits)];
            __m128 newa = _mm_load_ss(&delay[t + ((dp & (max_rev_dly - 1)) << rev_tap_bits)]);
            dp = (delay_pos - (delay_time[t + 1] >> 8));
            __m128 newb = _mm_load_ss(&delay[t + 1 + ((dp & (max_rev_dly - 1)) << rev_tap_bits)]);
            dp = (delay_pos - (delay_time[t + 2] >> 8));
            newa = _mm_unpacklo_ps(newa, newb); // a,b,0,0
            __m128 newc = _mm_load_ss(&delay[t + 2 + ((dp & (max_rev_dly - 1)) << rev_tap_bits)]);
            dp = (delay_pos - (delay_time[t + 3] >> 8));
            __m128 newd = _mm_load_ss(&delay[t + 3 + ((dp & (max_rev_dly - 1)) << rev_tap_bits)]);
            newc = _mm_unpacklo_ps(newc, newd);      // c,d,0,0
            __m128 new4 = _mm_movelh_ps(newa, newc); // a,b,c,d

            __m128 out_tap4 = _mm_load_ps(&out_tap[t]);
            out_tap4 = _mm_add_ps(_mm_mul_ps(out_tap4, damp4), _mm_mul_ps(new4, damp4m1));
            _mm_store_ps(&out_tap[t], out_tap4);
            // out_tap[t] = this->floatValue(rev1_damping]*out_tap[t) + (1-
            // this->floatValue(rev1_damping))*newa;
        }

        __m128 fb = _mm_add_ps(_mm_add_ps(_mm_load_ps(out_tap), _mm_load_ps(out_tap + 4)),
                               _mm_add_ps(_mm_load_ps(out_tap + 8), _mm_load_ps(out_tap + 12)));
        fb = mech::sum_ps_to_ss(fb);
        /*pd_floator(int t=0; t<rev_taps; t+=4)
        {
                fb += out_tap[t] + out_tap[t+1] + out_tap[t+2] + out_tap[t+3];
        }*/

        const __m128 ca = _mm_set_ss(((float)(-(2.f) / rev_taps)));
        // fb =  ca * fb + predelay[(delay_pos - pdtime)&(max_rev_dly-1)];
        fb = _mm_add_ss(_mm_mul_ss(ca, fb),
                        _mm_load_ss(&predelay[(delay_pos - pdtime) & (max_rev_dly - 1)]));

        delay_pos = (delay_pos + 1) & (max_rev_dly - 1);

        predelay[delay_pos] = 0.5f * (dataL[k] + dataR[k]);
        //__m128 fb4 = _mm_load1_ps(&fb);
        __m128 fb4 = _mm_shuffle_ps(fb, fb, 0);

        __m128 L = _mm_setzero_ps(), R = _mm_setzero_ps();
        for (int t = 0; t < rev_taps; t += 4)
        {
            __m128 ot = _mm_load_ps(&out_tap[t]);
            __m128 dfb = _mm_load_ps(&delay_fb[t]);
            __m128 a = _mm_mul_ps(dfb, _mm_add_ps(fb4, ot));
            _mm_store_ps(&delay[(delay_pos << rev_tap_bits) + t], a);
            L = _mm_add_ps(L, _mm_mul_ps(ot, _mm_load_ps(&delay_pan_L[t])));
            R = _mm_add_ps(R, _mm_mul_ps(ot, _mm_load_ps(&delay_pan_R[t])));
        }
        L = mech::sum_ps_to_ss(L);
        R = mech::sum_ps_to_ss(R);
        _mm_store_ss(&wetL[k], L);
        _mm_store_ss(&wetR[k], R);
    }

    if (!this->isDeactivated(rev1_lowcut))
    {
        locut.process_block(wetL, wetR);
    }

    band1.process_block(wetL, wetR);

    if (!this->isDeactivated(rev1_highcut))
    {
        hicut.process_block(wetL, wetR);
    }

    // scale width
    this->applyWidth(wetL, wetR, width);

    mix.fade_2_blocks_inplace(dataL, wetL, dataR, wetR, this->blockSize_quad);
}

template <typename FXConfig> inline void Reverb1<FXConfig>::loadpreset(int id)
{
    if (shape != id)
        clear_buffers();

    shape = id;

    switch (id)
    {
    case 0:
        delay_time[0] = 1339934;
        delay_time[1] = 962710;
        delay_time[2] = 1004427;
        delay_time[3] = 1103966;
        delay_time[4] = 1198575;
        delay_time[5] = 1743348;
        delay_time[6] = 1033425;
        delay_time[7] = 933313;
        delay_time[8] = 949407;
        delay_time[9] = 1402754;
        delay_time[10] = 1379894;
        delay_time[11] = 1225304;
        delay_time[12] = 1135598;
        delay_time[13] = 1402107;
        delay_time[14] = 956152;
        delay_time[15] = 1137737;
        break;
    case 1:
        delay_time[0] = 1265607;
        delay_time[1] = 844703;
        delay_time[2] = 856159;
        delay_time[3] = 1406425;
        delay_time[4] = 786608;
        delay_time[5] = 1163557;
        delay_time[6] = 1091206;
        delay_time[7] = 1129434;
        delay_time[8] = 1270379;
        delay_time[9] = 896997;
        delay_time[10] = 1415393;
        delay_time[11] = 782808;
        delay_time[12] = 868582;
        delay_time[13] = 1234463;
        delay_time[14] = 1000336;
        delay_time[15] = 968299;
        break;
    case 2:
        delay_time[0] = 1293101;
        delay_time[1] = 1334867;
        delay_time[2] = 1178781;
        delay_time[3] = 1850949;
        delay_time[4] = 1663760;
        delay_time[5] = 1982922;
        delay_time[6] = 1211021;
        delay_time[7] = 1824481;
        delay_time[8] = 1520266;
        delay_time[9] = 1351822;
        delay_time[10] = 1102711;
        delay_time[11] = 1513696;
        delay_time[12] = 1057618;
        delay_time[13] = 1671799;
        delay_time[14] = 1406360;
        delay_time[15] = 1170468;
        break;
    case 3:
        delay_time[0] = 1833435;
        delay_time[1] = 2462309;
        delay_time[2] = 2711583;
        delay_time[3] = 2219764;
        delay_time[4] = 1664194;
        delay_time[5] = 2109157;
        delay_time[6] = 1626137;
        delay_time[7] = 1434473;
        delay_time[8] = 2271242;
        delay_time[9] = 1621375;
        delay_time[10] = 1831218;
        delay_time[11] = 2640903;
        delay_time[12] = 1577737;
        delay_time[13] = 1871624;
        delay_time[14] = 2439164;
        delay_time[15] = 1427343;
        break;
    }

    for (int t = 0; t < rev_taps; t++)
    {
        // float r = storage->rand_01();
        // float rbp = storage->rand_pm1();
        // float a = 256.f * (3000.f * (1.f + rbp * rbp * this->floatValue(rev1_variation)))*(1.f
        // + 1.f * this->floatValue(rev1_roomsize)); delay_time[t] = (int)a;
        delay_time[t] = (int)((float)(2.f * this->floatValue(rev1_roomsize)) * delay_time[t]);
    }
    lastf[rev1_roomsize] = this->floatValue(rev1_roomsize);
    update_rtime();
}

template <typename FXConfig> inline void Reverb1<FXConfig>::update_rtime()
{
    int max_dt = 0;
    for (int t = 0; t < rev_taps; t++)
    {
        delay_fb[t] = powf(db60, delay_time[t] / (256.f * this->sampleRate() *
                                                  powf(2.f, this->floatValue(rev1_decaytime))));
        max_dt = std::max(max_dt, delay_time[t]);
    }
    lastf[rev1_decaytime] = this->floatValue(rev1_decaytime);
    float t =
        this->blockSize_inv *
        ((float)(max_dt >> 8) + this->sampleRate() * powf(2.f, this->floatValue(rev1_decaytime)) *
                                    2.f); // * 2.f is to get the db120 time
    ringout_time = (int)t;
}

} // namespace sst::effects::reverb1

#endif
