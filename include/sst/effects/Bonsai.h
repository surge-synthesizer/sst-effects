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
#ifndef INCLUDE_SST_EFFECTS_BONSAI_H
#define INCLUDE_SST_EFFECTS_BONSAI_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/FastMath.h"
#include "sst/basic-blocks/tables/DbToLinearProvider.h"

namespace sst::effects
{
namespace sdsp = sst::basic_blocks::dsp;

inline float freq_sr_to_alpha(float freq, float sr)
{
    const float freq2pi = 2.f * M_PI * freq;
    return freq2pi / (freq2pi + sr);
}
template <size_t blockSize>
inline void freq_sr_to_alpha_block(float *__restrict freq, float sr, float *__restrict coef)
{
    for (auto i = 1U; i < blockSize; ++i)
    {
        const float freq2pi = 2.f * M_PI * freq[i];
        coef[i] = freq2pi / (freq2pi + sr);
    }
}
inline float freq_sr2pi_to_alpha(float freq, float sr_div2pi) { return freq / (freq + sr_div2pi); }
template <size_t blockSize>
inline void freq_sr2pi_to_alpha_block(float *__restrict freq, float sr_div2pi,
                                      float *__restrict coef)
{
    for (auto i = 1U; i < blockSize; ++i)
    {
        coef[i] = freq[i] / (freq[i] + sr_div2pi);
    }
}

template <size_t blockSize>
inline void onepole_lp_block(float &last, const float *__restrict coef, float *__restrict src,
                             float *__restrict dst)
{
    dst[0] = last + coef[0] * (src[0] - last);
    for (auto i = 1U; i < blockSize; ++i)
    {
        dst[i] = dst[i - 1] + coef[i] * (src[i] - dst[i - 1]);
    }
    last = dst[blockSize - 1];
}
template <size_t blockSize>
inline void onepole_lp_block(float &last, const float coef, float *__restrict src,
                             float *__restrict dst)
{
    dst[0] = last + coef * (src[0] - last);
    for (auto i = 1U; i < blockSize; ++i)
    {
        dst[i] = dst[i - 1] + coef * (src[i] - dst[i - 1]);
    }
    last = dst[blockSize - 1];
}
template <size_t blockSize>
inline void onepole_lp_block(float &last, const float coef, float src, float *__restrict dst)
{
    dst[0] = last + coef * (src - last);
    for (auto i = 1U; i < blockSize; ++i)
    {
        dst[i] = dst[i - 1] + coef * (src - dst[i - 1]);
    }
    last = dst[blockSize - 1];
}

template <size_t blockSize>
inline void sum2_block(float *__restrict one, float *__restrict two, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = one[i] + two[i];
    }
}
template <size_t blockSize>
inline void sum3_block(float *__restrict one, float *__restrict two, float *__restrict three,
                       float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = one[i] + two[i] + three[i];
    }
}
template <size_t blockSize>
inline void minus2_block(float *__restrict one, float two, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = one[i] - two;
    }
}
template <size_t blockSize>
inline void minus2_block(float *__restrict one, float *__restrict two, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = one[i] - two[i];
    }
}

template <size_t blockSize>
inline void mul_block(float *__restrict src1, float src2, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = src1[i] * src2;
    }
}
template <size_t blockSize>
inline void mul_block(float *__restrict src1, float *__restrict src2, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = src1[i] * src2[i];
    }
}

template <size_t blockSize> inline void mul_block_inplace(float *__restrict src, float factor)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        src[i] = src[i] * factor;
    }
}
template <size_t blockSize>
inline void mul_block_inplace(float *__restrict src, float *__restrict factor)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        src[i] = src[i] * factor[i];
    }
}

template <size_t blockSize> inline void negate_block(float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = -src[i];
    }
}

template <size_t blockSize>
inline void onepole_hp_block(float &last, const float *__restrict coef, float *__restrict src,
                             float *__restrict dst)
{
    float lp alignas(16)[blockSize];
    onepole_lp_block<blockSize>(last, coef, src, lp);
    minus2_block<blockSize>(src, lp, dst);
}
template <size_t blockSize>
inline void onepole_hp_block(float &last, const float coef, float *__restrict src,
                             float *__restrict dst)
{
    float lp alignas(16)[blockSize];
    onepole_lp_block<blockSize>(last, coef, src, lp);
    minus2_block<blockSize>(src, lp, dst);
}

template <size_t blockSize>
inline void unit_delay_block(float &last, float *__restrict src, float *__restrict dst)
{
    dst[0] = last;
    for (auto i = 1U; i < blockSize; ++i)
    {
        dst[i] = src[i - 1];
    }
    last = src[blockSize - 1];
}

template <typename T> inline int sgn(T val) { return (T(0) < val) - (val < T(0)); }

// pade approximants tend either to infinity or 0. tanh and inv sinh have a horizontal asymptote and
// so they will always eventually deviate. if you replace this with some approximation, please check
// that it does not trend to infinity as increasing the gain enough will eventualy cause it to no
// longer sound distorted
inline float clip_inv_sinh(float src)
{
    const float abs2x = 2 * fabs(src);
    return logf(abs2x + sqrt(abs2x * abs2x + 1)) * 0.5 * sgn(src);
}
inline float clip_inv_sinh(float invlevel, float level, float src)
{
    const float scaledown = invlevel * src;
    const float abs2x = 2 * fabs(scaledown);
    return logf(abs2x + sqrt(abs2x * abs2x + 1)) * 0.5 * sgn(scaledown) * level;
}
template <size_t blockSize>
inline void clip_inv_sinh_block(float invlevel, float level, float *__restrict src,
                                float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        const float scaledown = invlevel * src[i];
        const float abs2x = 2 * fabs(scaledown);
        dst[i] = logf(abs2x + sqrt(abs2x * abs2x + 1)) * 0.5 * sgn(scaledown) * level;
    }
}
template <size_t blockSize>
inline void clip_inv_sinh_block(float level, float *__restrict src, float *__restrict dst)
{
    const float invlevel = 1 / level;
    for (auto i = 0U; i < blockSize; ++i)
    {
        const float scaledown = invlevel * src[i];
        const float abs2x = 2 * fabs(scaledown);
        dst[i] = logf(abs2x + sqrt(abs2x * abs2x + 1)) * 0.5 * sgn(scaledown) * level;
    }
}
template <size_t blockSize>
inline void clip_inv_sinh_block(float *__restrict level, float *__restrict src,
                                float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        const float scaledown = src[i] / level[i];
        const float abs2x = 2 * fabs(scaledown);
        dst[i] = logf(abs2x + sqrt(abs2x * abs2x + 1)) * 0.5 * sgn(scaledown) * level[i];
    }
}

// trends to infinity
inline float clip_tanh76(float x) { return sst::basic_blocks::dsp::fasttanh(x); }
inline float clip_tanh76(float x, float invlevel, float level)
{
    return sst::basic_blocks::dsp::fasttanh(invlevel * x) * level;
}
template <size_t blockSize>
inline void clip_tanh76_block(float invlevel, float level, float *__restrict src,
                              float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = sst::basic_blocks::dsp::fasttanh(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_tanh76_block(float *__restrict level, float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = sst::basic_blocks::dsp::fasttanh(src[i] / level[i]) * level[i];
    }
}

// trends to 0 decently slowly, 9,10 would be better
inline float fasttanh78(float x)
{
    auto x2 = x * x;
    auto numerator = x * (2027025 + x2 * (270270 + x2 * (6930 + x2 * 36)));
    auto denominator = 2027025 + x2 * (945945 + x2 * (51975 + x2 * (630 + x2)));
    return numerator / denominator;
}
inline float fasttanh78(float x, float invlevel, float level)
{
    return fasttanh78(x * invlevel) * level;
}
template <size_t blockSize>
inline void clip_tanh78_block(float invlevel, float level, float *__restrict src,
                              float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh78(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_tanh78_block(float level, float *__restrict src, float *__restrict dst)
{
    const float invlevel = 1 / level;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh78(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_tanh78_block(float *__restrict level, float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh78(src[i] / level[i]) * level[i];
    }
}

// trends to 0 quickly and only approximates tanh within around +-0.2
inline float fasttanh_foldback(float x)
{
    auto x2 = x * x;
    auto numerator = x * 5 * (21 + x2 * 2);
    auto denominator = 105 + 0.6825 * x2 * (45 + x2 * 6);
    return numerator / denominator;
}
inline float fasttanh_foldback(float x, float invlevel, float level)
{
    return fasttanh_foldback(x * invlevel) * level;
}
template <size_t blockSize>
inline void clip_tanh_foldback_block(float invlevel, float level, float *__restrict src,
                                     float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh_foldback(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_tanh_foldback_block(float level, float *__restrict src, float *__restrict dst)
{
    const float invlevel = 1 / level;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh_foldback(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_tanh_foldback_block(float *__restrict level, float *__restrict src,
                                     float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fasttanh_foldback(src[i] / level[i]) * level[i];
    }
}

inline float sech(float x) { return 2 / (exp(x) + exp(-x)); }

// oscillates a bit in the middle but eventually stays around 0.5 * tanh(x)
inline float clip_sine_tanh(float x)
{
    return (sech(x * 0.125) + 0.125) * sin(x) + 0.5 * fasttanh78(x * 0.125);
}
inline float clip_sine_tanh(float x, float invlevel, float level)
{
    return clip_sine_tanh(x * invlevel) * level;
}
template <size_t blockSize>
inline void clip_sine_tanh_block(float invlevel, float level, float *__restrict src,
                                 float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = clip_sine_tanh(invlevel * src[i]) * level;
    }
}
template <size_t blockSize>
inline void clip_sine_tanh_block(float level, float *__restrict src, float *__restrict dst)
{
    const float invlevel = 1 / level;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = clip_sine_tanh(src[i] * invlevel) * level;
    }
}
template <size_t blockSize>
inline void clip_sine_tanh_block(float *__restrict level, float *__restrict src,
                                 float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = clip_sine_tanh(src[i] / level[i]) * level[i];
    }
}

inline float rerange(float in, float l1, float h1, float l2, float h2)
{
    return (in - l1) * (h2 - l2) * (1.f / (h1 - l1)) + l2;
}
template <size_t blockSize>
inline void rerange_block(float *__restrict in, float l1, float h1, float l2, float h2,
                          float *__restrict src, float *__restrict dst)
{
    const float inv_h1_m_l1 = 1.f / (h1 - l1);
    const float h2_m_l2 = h2 - l2;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (in[i] - l1) * h2_m_l2 * inv_h1_m_l1 + l2;
    }
}
template <size_t blockSize>
inline void rerange_block(float *__restrict in, float l1, float h1, float *__restrict l2,
                          float *__restrict h2, float *__restrict src, float *__restrict dst)
{
    const float inv_h1_m_l1 = 1.f / (h1 - l1);
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (in[i] - l1) * (h2[i] - l2[i]) * inv_h1_m_l1 + l2[i];
    }
}
template <size_t blockSize>
inline void rerange_block(float *__restrict in, float *__restrict l1, float *__restrict h1,
                          float *__restrict l2, float *__restrict h2, float *__restrict src,
                          float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (in[i] - l1[i]) * (h2[i] - l2[i]) * (1.f / (h1[i] - l1[i])) + l2[i];
    }
}
inline float rerange01(float in, float l2, float h2) { return in * (h2 - l2) + l2; }
template <size_t blockSize>
inline void rerange01_block(float *__restrict in, float l2, float h2, float *__restrict src,
                            float *__restrict dst)
{
    const float h2_m_l2 = h2 - l2;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = in[i] * h2_m_l2 + l2;
    }
}
template <size_t blockSize>
inline float rerange01_block(float *__restrict in, float *__restrict l2, float *__restrict h2,
                             float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = in[i] * (h2[i] - l2[i]) + l2[i];
    }
}
inline float rerange1b(float in, float l2, float h2) { return (in - 1.f) * (l2 - h2) * 0.5 + l2; }
template <size_t blockSize>
inline void rerange1b_block(float *__restrict in, float l2, float h2, float *__restrict src,
                            float *__restrict dst)
{
    const float h2_m_l2 = h2 - l2;
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (in[i] - 1.f) * h2_m_l2 + l2;
    }
}
template <size_t blockSize>
inline float rerange1b_block(float *__restrict in, float *__restrict l2, float *__restrict h2,
                             float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (in[i] - 1.f) * (h2[i] - l2[i]) + l2[i];
    }
}

inline float invsq(float in)
{
    const float inv = 1.f - in;
    return 1 - (inv * inv);
}
template <size_t blockSize> inline void invsq_block(float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        const float inv = 1.f - src[i];
        dst[i] = 1 - (inv * inv);
    }
}

template <size_t blockSize>
inline void clampbi_block(float minmax, float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fmax(fmin(src[i], minmax), -minmax);
        // if (src[i] > minmax)
        // {
        //     dst[i] = minmax;
        // }
        // else if (src[i] < -minmax)
        // {
        //     dst[i] = -minmax;
        // }
        // else
        // {
        //     dst[i] = src[i];
        // }
    }
}
template <size_t blockSize>
inline void max_block(float *__restrict src, float value, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fmax(src[i], value);
    }
}

template <size_t blockSize>
inline void lerp_block(float *__restrict src1, float *__restrict src2, float mix,
                       float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (src2[i] - src1[i]) * mix + src1[i];
    }
}
template <size_t blockSize>
inline void lerp_block(float *__restrict src1, float *__restrict src2, float *__restrict mix,
                       float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = (src2[i] - src1[i]) * mix[i] + src1[i];
    }
}

template <size_t blockSize> inline void abs_block(float *__restrict src, float *__restrict dst)
{
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = fabs(src[i]);
    }
}

inline int32_t super_simple_noise(int32_t last) { return last * 1103515245 + 12345; }
template <size_t blockSize> inline void noise_block(float &last, float *__restrict dst)
{
    int32_t temp alignas(16)[blockSize] = {};
    temp[0] = super_simple_noise(last);
    for (auto i = 1U; i < blockSize; ++i)
    {
        temp[i] = super_simple_noise(temp[i - 1]);
    }
    last = temp[blockSize - 1];
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = 4.656700025584826e-10 * (float)temp[i];
    }
}
template <size_t blockSize> inline void noise_ds2_block(float &last, float *__restrict dst)
{
    int32_t temp alignas(16)[blockSize] = {};
    temp[0] = super_simple_noise(last);
    for (auto i = 1U; i < (blockSize >> 1); ++i)
    {
        const float noise = super_simple_noise(temp[2 * i - 2]);
        temp[2 * i - 1] = noise;
        temp[2 * i] = noise;
    }
    last = dst[blockSize - 1];
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = 4.656700025584826e-10 * (float)temp[i];
    }
}
template <size_t blockSize> inline void noise_ds4_block(float &last, float *__restrict dst)
{
    int32_t temp alignas(16)[blockSize] = {};
    temp[0] = super_simple_noise(last);
    for (auto i = 1U; i < (blockSize >> 2); ++i)
    {
        const float noise = super_simple_noise(temp[4 * i - 4]);
        temp[4 * i - 3] = noise;
        temp[4 * i - 2] = noise;
        temp[4 * i - 1] = noise;
        temp[4 * i] = noise;
    }
    last = temp[blockSize - 1];
    for (auto i = 0U; i < blockSize; ++i)
    {
        dst[i] = 4.656700025584826e-10 * (float)temp[i];
    }
}

template <typename FXConfig> struct Bonsai : EffectTemplateBase<FXConfig>
{
    // static constexpr double MIDI_0_FREQ = 8.17579891564371;
    // or 440.0 * pow( 2.0, - (69.0/12.0) )

    enum b_dist_modes
    {
        bdm_inv_sinh = 0,
        bdm_tanh = 1,
        bdm_tanh_approx_foldback = 2,
        bdm_sine = 3,
    };

    enum b_params
    {
        // Input
        b_gain_in = 0, // input level control

        // Bass Boost
        b_bass_boost,   // amount of bass boost
        b_bass_distort, // how much distortion should the bass boost have
                        // b_bass_st_mono, // should the bass boost be applied in mono or stereo
                        // maybe right click options need to be elsewhere

        // Tape(ish) Saturation
        b_tape_dist_mode, // what distortion should be used in the saturation
        b_tape_sat,       // tape sat meta control

        // Noise
        b_noise_sensitivity, // noise meta control, not volume compensated
        b_noise_gain,        // additional gain control of the noise

        // Misc
        b_dull,     // dull meta control
        b_gain_out, // output level control
        b_mix,      // mix

        b_num_params,
    };

    static constexpr int numParams{b_num_params};
    static constexpr const char *effectName{"bonsai"};

    Bonsai(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
           typename FXConfig::ValueStorage *p)
        : EffectTemplateBase<FXConfig>(s, e, p)
    {
    }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);
    void tape_sat_block(float last[], int lastmin, const float coef4690, const float coef1280,
                        const float coef160, const float coef99, const float coef3000,
                        const float coef8000, float sat, int mode, float *__restrict srcL,
                        float *__restrict srcR, float *__restrict dstL, float *__restrict dstR);
    void bass_boost_block(float last[], int lastmin, const float coef20, const float coef30,
                          const float coef50, const float coef200, const float coef500, float boost,
                          float dist, float *__restrict srcL, float *__restrict srcR,
                          float *__restrict dstL, float *__restrict dstR);
    void noise_channel_block(float last[], int lastmin, const float coef50, const float coef500,
                             const float coef1000, float *__restrict sens_lp_scale,
                             float *__restrict sens_lp_coef, float *__restrict threshold,
                             const float sr_scaled, float *__restrict src, float *__restrict noise,
                             float *__restrict dst);
    void tape_noise_block(float last[], int lastmin, const float coef20, const float coef50,
                          const float coef500, const float coef1000, const float coef2000,
                          float sens, float gain, float *__restrict srcL, float *__restrict srcR,
                          float *__restrict dstL, float *__restrict dstR);
    void age_block(float last[], int lastmin, const float coef0, const float coef100,
                   const float coef700, const float coef1200, const float coef2000,
                   const float coef3000, float dull, float *__restrict srcL, float *__restrict srcR,
                   float *__restrict dstL, float *__restrict dstR);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return ringout_value; }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numParams);
        using pmd = basic_blocks::params::ParamMetaData;
        auto result = pmd().withName("Unknown " + std::to_string(idx));

        switch ((b_params)idx)
        {
        case b_bass_boost:
            return result.withName("Amount").withRange(0.f, 1.f).withDefault(0.25f);
        case b_bass_distort:
            return result.withName("Distort").withRange(0.f, 3.f).withDefault(1.f);
        case b_tape_sat:
            return result.withName("Tape Sat").withRange(0.f, 1.f).withDefault(0.25f);
        case b_tape_dist_mode:
            return result.withType(pmd::INT)
                .withName("Tape Dist Mode")
                .withRange(bdm_inv_sinh, bdm_sine)
                .withDefault(bdm_tanh)
                .withUnorderedMapFormatting({{bdm_inv_sinh, "Inverse Sinh"},
                                             {bdm_tanh, "Tanh"},
                                             {bdm_tanh_approx_foldback, "Foldback"},
                                             {bdm_sine, "Sine-Tanh Combo"}});
        case b_noise_sensitivity:
            return result.withName("Sensitivity").withRange(0.f, 1.f).withDefault(0.25f);
        case b_noise_gain:
            return result.withName("Gain").asDecibelNarrow().withDefault(0.f);
        case b_dull:
            return result.withName("Dull").withRange(0.f, 1.f).withDefault(0.f);
        case b_gain_in:
            return result.withName("Input Gain").asDecibelNarrow().withDefault(0.f);
        case b_gain_out:
            return result.withName("Output Gain").asDecibelNarrow().withDefault(0.f);
        case b_mix:
            return result.withName("Mix").asPercent().withDefault(1.f);
        case b_num_params:
            throw std::logic_error("getParam called with num_params");
        }
        return result;
    }

    int ringout_value = -1;

    float last[89] = {};
    float sr_div2pi = Bonsai<FXConfig>::sampleRate() / (2 * M_PI);
    const float coef0 = freq_sr2pi_to_alpha(0, sr_div2pi);
    const float coef10 = freq_sr2pi_to_alpha(10, sr_div2pi);
    const float coef20 = freq_sr2pi_to_alpha(20, sr_div2pi);
    const float coef30 = freq_sr2pi_to_alpha(20, sr_div2pi);
    const float coef50 = freq_sr2pi_to_alpha(50, sr_div2pi);
    const float coef99 = freq_sr2pi_to_alpha(99, sr_div2pi);
    const float coef100 = freq_sr2pi_to_alpha(100, sr_div2pi);
    const float coef160 = freq_sr2pi_to_alpha(160, sr_div2pi);
    const float coef200 = freq_sr2pi_to_alpha(200, sr_div2pi);
    const float coef500 = freq_sr2pi_to_alpha(500, sr_div2pi);
    const float coef700 = freq_sr2pi_to_alpha(700, sr_div2pi);
    const float coef1000 = freq_sr2pi_to_alpha(1000, sr_div2pi);
    const float coef1200 = freq_sr2pi_to_alpha(1200, sr_div2pi);
    const float coef1280 = freq_sr2pi_to_alpha(1280, sr_div2pi);
    const float coef2000 = freq_sr2pi_to_alpha(2000, sr_div2pi);
    const float coef3000 = freq_sr2pi_to_alpha(3000, sr_div2pi);
    const float coef4690 = freq_sr2pi_to_alpha(4690, sr_div2pi);
    const float coef8000 = freq_sr2pi_to_alpha(8000, sr_div2pi);
};

template <typename FXConfig> inline void Bonsai<FXConfig>::initialize()
{
    last[47] = 1; // noise mid seed
    last[48] = 2; // noise side seed
}

template <size_t blockSize>
inline void tilt_highboost_block(float &last1, float &last2, float coef1, float coef2,
                                 float *__restrict src, float *__restrict dst)
{
    float tilt1_hp4690 alignas(16)[blockSize] = {};
    float tilt1_lp1280 alignas(16)[blockSize] = {};
    const float amp_db13_5 = 4.7315127124153629629634297;   // 1.122018456459045 ^ 13.5
    const float amp_mdbm1_7 = -0.8222426472597752553172857; // -(1.122018456459045 ^ -1.7)
    onepole_hp_block<blockSize>(last1, coef1, src, tilt1_hp4690);
    onepole_lp_block<blockSize>(last2, coef2, src, tilt1_lp1280);
    mul_block_inplace<blockSize>(tilt1_hp4690, amp_db13_5);
    mul_block_inplace<blockSize>(tilt1_lp1280, amp_mdbm1_7);
    sum3_block<blockSize>(tilt1_hp4690, tilt1_lp1280, src, dst);
}
template <size_t blockSize>
inline void tilt_lowboost_block(float &last1, float &last2, float coef1, float coef2,
                                float *__restrict src, float *__restrict dst)
{
    float tilt1_hp160 alignas(16)[blockSize] = {};
    float tilt1_lp99 alignas(16)[blockSize] = {};
    const float amp_db13_5 = 4.7315127124153629629634297;   // 1.122018456459045 ^ 13.5
    const float amp_mdbm1_4 = -0.8511380359115372899247841; // -(1.122018456459045 ^ -1.4)
    onepole_hp_block<blockSize>(last1, coef1, src, tilt1_hp160);
    onepole_lp_block<blockSize>(last2, coef2, src, tilt1_lp99);
    mul_block_inplace<blockSize>(tilt1_hp160, amp_mdbm1_4);
    mul_block_inplace<blockSize>(tilt1_lp99, amp_db13_5);
    sum3_block<blockSize>(tilt1_hp160, tilt1_lp99, src, dst);
}

template <size_t blockSize>
inline void high_shelf_block(float &last, float coef, float gainfactor, float *__restrict src,
                             float *__restrict dst)
{
    float highpass alignas(16)[blockSize] = {};
    onepole_hp_block<blockSize>(last, coef, src, highpass);
    mul_block_inplace<blockSize>(highpass, gainfactor);
    sum2_block<blockSize>(highpass, src, dst);
}
template <size_t blockSize>
inline void high_shelf_block(float &last, float coef, float *__restrict gainfactor,
                             float *__restrict src, float *__restrict dst)
{
    float highpass alignas(16)[blockSize] = {};
    onepole_hp_block<blockSize>(last, coef, src, highpass);
    mul_block_inplace<blockSize>(highpass, gainfactor);
    sum2_block<blockSize>(highpass, src, dst);
}
template <size_t blockSize>
inline void low_shelf_block(float &last, float coef, float gainfactor, float *__restrict src,
                            float *__restrict dst)
{
    float lowpass alignas(16)[blockSize] = {};
    onepole_lp_block<blockSize>(last, coef, src, lowpass);
    mul_block_inplace<blockSize>(lowpass, gainfactor);
    sum2_block<blockSize>(lowpass, src, dst);
}
template <size_t blockSize>
inline void low_shelf_block(float &last, float coef, float *__restrict gainfactor,
                            float *__restrict src, float *__restrict dst)
{
    float lowpass alignas(16)[blockSize] = {};
    onepole_lp_block<blockSize>(last, coef, src, lowpass);
    mul_block_inplace<blockSize>(lowpass, gainfactor);
    sum2_block<blockSize>(lowpass, src, dst);
}

template <size_t blockSize>
inline void high_shelf_nl_block(float &last, float coef, float gainfactor, float invlevel,
                                float level, float *__restrict src, float *__restrict dst)
{
    float highpass alignas(16)[blockSize] = {};
    onepole_hp_block<blockSize>(last, coef, src, highpass);
    mul_block_inplace<blockSize>(highpass, gainfactor);
    clip_inv_sinh_block<blockSize>(invlevel, level, highpass, highpass);
    sum2_block<blockSize>(highpass, src, dst);
}
template <size_t blockSize>
inline void high_shelf_nl_block(float &last, float coef, float *__restrict gainfactor,
                                float invlevel, float level, float *__restrict src,
                                float *__restrict dst)
{
    float highpass alignas(16)[blockSize] = {};
    onepole_hp_block<blockSize>(last, coef, src, highpass);
    mul_block_inplace<blockSize>(highpass, gainfactor);
    clip_inv_sinh_block<blockSize>(invlevel, level, highpass, highpass);
    sum2_block<blockSize>(highpass, src, dst);
}
template <size_t blockSize>
inline void low_shelf_nl_block(float &last, float coef, float gainfactor, float invlevel,
                               float level, float *__restrict src, float *__restrict dst)
{
    float lowpass alignas(16)[blockSize] = {};
    onepole_lp_block<blockSize>(last, coef, src, lowpass);
    mul_block_inplace<blockSize>(lowpass, gainfactor);
    clip_inv_sinh_block<blockSize>(invlevel, level, lowpass, lowpass);
    sum2_block<blockSize>(lowpass, src, dst);
}
template <size_t blockSize>
inline void low_shelf_nl_block(float &last, float coef, float *__restrict gainfactor,
                               float invlevel, float level, float *__restrict src,
                               float *__restrict dst)
{
    float lowpass alignas(16)[blockSize] = {};
    onepole_lp_block<blockSize>(last, coef, src, lowpass);
    mul_block_inplace<blockSize>(lowpass, gainfactor);
    clip_inv_sinh_block<blockSize>(invlevel, level, lowpass, lowpass);
    sum2_block<blockSize>(lowpass, src, dst);
}

// 16 last slots
template <typename FXConfig>
inline void Bonsai<FXConfig>::tape_sat_block(float last[], int lastmin, const float coef4690,
                                             const float coef1280, const float coef160,
                                             const float coef99, const float coef3000,
                                             const float coef8000, float sat, int mode,
                                             float *__restrict srcL, float *__restrict srcR,
                                             float *__restrict dstL, float *__restrict dstR)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};
    float bufC alignas(16)[FXConfig::blockSize] = {};

    const float sat_invsq = invsq(sat);
    float sat_invinvsq_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 0], coef20, -sat_invsq,
                                          sat_invinvsq_block);
    float sat_halfsq_db alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(
        last[lastmin + 1], coef20, this->dbToLinear(rerange01(0.5 * (sat * sat + sat), 1.5, 6.f)),
        sat_halfsq_db);
    float level alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef20,
                                          rerange01(sat_invsq, 0.15, 0.025), level);
    float pregain alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 3], coef20,
                                          this->dbToLinear(rerange01(sat, 0.f, 9.f)), pregain);

    tilt_highboost_block<FXConfig::blockSize>(last[lastmin + 4], last[lastmin + 5], coef4690,
                                              coef1280, srcL, bufA);
    mul_block<FXConfig::blockSize>(bufA, pregain, bufA);
    tilt_highboost_block<FXConfig::blockSize>(last[lastmin + 6], last[lastmin + 7], coef4690,
                                              coef1280, srcR, bufC);
    mul_block<FXConfig::blockSize>(bufC, pregain, bufC);

    switch ((b_dist_modes)mode)
    {
    case bdm_inv_sinh:
        clip_inv_sinh_block<FXConfig::blockSize>(level, bufA, bufA);
        clip_inv_sinh_block<FXConfig::blockSize>(level, bufC, bufC);
        break;
    case bdm_tanh:
        clip_tanh78_block<FXConfig::blockSize>(level, bufA, bufA);
        clip_tanh78_block<FXConfig::blockSize>(level, bufC, bufC);
        break;
    case bdm_tanh_approx_foldback:
        clip_tanh_foldback_block<FXConfig::blockSize>(level, bufA, bufA);
        clip_tanh_foldback_block<FXConfig::blockSize>(level, bufC, bufC);
        break;
    case bdm_sine:
        clip_sine_tanh_block<FXConfig::blockSize>(level, bufA, bufA);
        clip_sine_tanh_block<FXConfig::blockSize>(level, bufC, bufC);
        break;
    default:
        clip_tanh78_block<FXConfig::blockSize>(level, bufA, bufA);
        clip_tanh78_block<FXConfig::blockSize>(level, bufC, bufC);
        break;
    }
    tilt_lowboost_block<FXConfig::blockSize>(last[lastmin + 8], last[lastmin + 9], coef160, coef99,
                                             bufA, bufB);
    mul_block<FXConfig::blockSize>(bufB, sat_halfsq_db, bufB);
    clip_tanh78_block<FXConfig::blockSize>(6.6666666666666666, 0.15, bufB, bufB); // 1/0.15
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 10], coef3000, sat_invinvsq_block, bufB,
                                          bufA);
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 11], coef8000, sat_invinvsq_block, bufA,
                                          dstL);
    sum2_block<FXConfig::blockSize>(srcL, dstL, dstL);

    tilt_lowboost_block<FXConfig::blockSize>(last[lastmin + 12], last[lastmin + 13], coef160,
                                             coef99, bufC, bufB);
    mul_block<FXConfig::blockSize>(bufB, sat_halfsq_db, bufB);
    clip_tanh78_block<FXConfig::blockSize>(6.6666666666666666, 0.15, bufB, bufB); // 1/0.15
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 14], coef3000, sat_invinvsq_block, bufB,
                                          bufA);
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 15], coef8000, sat_invinvsq_block, bufA,
                                          dstR);
    sum2_block<FXConfig::blockSize>(srcR, dstR, dstR);
}

// 29 last slots
template <typename FXConfig>
inline void Bonsai<FXConfig>::bass_boost_block(float last[], int lastmin, const float coef20,
                                               const float coef30, const float coef50,
                                               const float coef200, const float coef500,
                                               float boost, float dist, float *__restrict srcL,
                                               float *__restrict srcR, float *__restrict dstL,
                                               float *__restrict dstR)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};
    float reused alignas(16)[FXConfig::blockSize] = {};
    float branch1 alignas(16)[FXConfig::blockSize] = {};
    float branch2 alignas(16)[FXConfig::blockSize] = {};
    float branch3 alignas(16)[FXConfig::blockSize] = {};

    const float dist01 = dist * 0.3333333333333333333333333;
    const float distsq = dist * dist; // this will extend past 0-1
    const float distinvsq = invsq(dist01);

    float lerp1_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 0], coef20,
                                          rerange01(distinvsq, 0.025, 0.01), lerp1_block);
    float lerp2_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 1], coef20, rerange01(distsq, 0.125, 1.f),
                                          lerp2_block);
    float lerp3_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef20, rerange01(distsq, 0.5, 10.f),
                                          lerp3_block);
    float lerp4_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 3], coef20, rerange01(distsq, 1.f, 5.f),
                                          lerp4_block);
    float lerp5_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 4], coef20,
                                          rerange01(dist01, 0.075, 0.025), lerp5_block);
    float lerp6_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 5], coef20, rerange01(dist01, 0.1, 0.075),
                                          lerp6_block);
    float boost_block alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 6], coef20, boost, boost_block);

    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 7], coef20, srcL, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 8], coef50, bufA, bufB);
    clip_tanh78_block<FXConfig::blockSize>(lerp1_block, bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 9], coef50, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 20.f, branch1);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 10], coef30, srcL, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 11], coef200, bufA, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 12], coef200, bufB, reused);
    mul_block<FXConfig::blockSize>(reused, 5.f, branch2);
    mul_block<FXConfig::blockSize>(reused, lerp2_block, bufA);
    clampbi_block<FXConfig::blockSize>(0.01, reused, bufA);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 13], coef200, bufA, branch3);
    mul_block<FXConfig::blockSize>(branch3, lerp3_block, branch3);
    mul_block<FXConfig::blockSize>(reused, lerp4_block, bufB);
    clip_tanh78_block<FXConfig::blockSize>(lerp5_block, bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 14], coef200, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 2.f, bufA);
    sum2_block<FXConfig::blockSize>(branch3, bufA, bufB);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 15], coef30, bufB, bufA);
    sum3_block<FXConfig::blockSize>(branch1, branch2, bufA, bufB);
    mul_block<FXConfig::blockSize>(bufB, 0.16666666666666666666666, bufB);
    // change the above constant to adjust the default gain, so the slider is negative less often
    // previous value: 0.3333333333333333333333333
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 16], coef500, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, boost_block, bufA);
    clip_tanh78_block<FXConfig::blockSize>(lerp6_block, bufA, bufA);
    // mul_block<FXConfig::blockSize>(bufA, rerange01(dist01, 1.25, 0.75), bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 17], coef500, bufA, dstL);
    sum2_block<FXConfig::blockSize>(srcL, dstL, dstL);

    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 18], coef20, srcR, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 19], coef50, bufA, bufB);
    clip_tanh78_block<FXConfig::blockSize>(lerp1_block, bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 20], coef50, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 20.f, branch1);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 21], coef30, srcR, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 22], coef200, bufA, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 23], coef200, bufB, reused);
    mul_block<FXConfig::blockSize>(reused, 5.f, branch2);
    mul_block<FXConfig::blockSize>(reused, lerp2_block, bufA);
    clampbi_block<FXConfig::blockSize>(0.01, reused, bufA);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 24], coef200, bufA, branch3);
    mul_block<FXConfig::blockSize>(branch3, lerp3_block, branch3);
    mul_block<FXConfig::blockSize>(reused, lerp4_block, bufB);
    clip_tanh78_block<FXConfig::blockSize>(lerp5_block, bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 25], coef200, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 2.f, bufA);
    sum2_block<FXConfig::blockSize>(branch3, bufA, bufB);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 26], coef30, bufB, bufA);
    sum3_block<FXConfig::blockSize>(branch1, branch2, bufA, bufB);
    mul_block<FXConfig::blockSize>(bufB, 0.16666666666666666666666, bufB);
    // change the above constant to adjust the default gain, so the slider is negative less often
    // previous value: 0.3333333333333333333333333
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 27], coef500, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, boost_block, bufA);
    clip_tanh78_block<FXConfig::blockSize>(lerp6_block, bufA, bufA);
    // mul_block<FXConfig::blockSize>(bufA, rerange01(dist01, 1.25, 0.75), bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 28], coef500, bufA, dstR);
    sum2_block<FXConfig::blockSize>(srcR, dstR, dstR);
}

// 8 last slots
template <typename FXConfig>
inline void Bonsai<FXConfig>::noise_channel_block(
    float last[], int lastmin, const float coef50, const float coef500, const float coef1000,
    float *__restrict sens_lp_scale, float *__restrict sens_lp_coef, float *__restrict threshold,
    const float sr_scaled, float *__restrict src, float *__restrict noise, float *__restrict dst)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};
    float bufC alignas(16)[FXConfig::blockSize] = {};
    float noise_filt alignas(16)[FXConfig::blockSize] = {};

    clampbi_block<FXConfig::blockSize>(1.f, noise, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 0], coef1000, bufA, noise_filt);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 1], coef50, bufA, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef50, bufB, bufC);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 3], coef50, src, bufA);
    sum3_block<FXConfig::blockSize>(src, bufA, bufC, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 4], coef1000, bufB, bufA);
    unit_delay_block<FXConfig::blockSize>(last[lastmin + 5], bufA, bufC);
    minus2_block<FXConfig::blockSize>(bufA, bufC, bufB);
    mul_block<FXConfig::blockSize>(bufB, sr_scaled, bufC);
    abs_block<FXConfig::blockSize>(bufC, bufB);
    minus2_block<FXConfig::blockSize>(bufB, threshold, bufB);
    max_block<FXConfig::blockSize>(bufB, 0.f, bufB);
    mul_block<FXConfig::blockSize>(bufB, bufB, bufB);
    mul_block<FXConfig::blockSize>(bufB, sens_lp_scale, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 6], sens_lp_coef, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, noise_filt, bufB);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 7], coef500, bufB, dst);
}
// 24 last slots, set first two to set noise seeds
template <typename FXConfig>
inline void Bonsai<FXConfig>::tape_noise_block(float last[], int lastmin, const float coef20,
                                               const float coef50, const float coef500,
                                               const float coef1000, const float coef2000,
                                               const float sens, const float gain,
                                               float *__restrict srcL, float *__restrict srcR,
                                               float *__restrict dstL, float *__restrict dstR)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};
    float noiseL alignas(16)[FXConfig::blockSize] = {};
    float noiseR alignas(16)[FXConfig::blockSize] = {};

    noise_block<FXConfig::blockSize>(last[lastmin + 0], bufA);
    noise_block<FXConfig::blockSize>(last[lastmin + 1], bufB);
    mul_block<FXConfig::blockSize>(bufB, 0.5, bufB);
    sum2_block<FXConfig::blockSize>(bufA, bufB, noiseL);
    minus2_block<FXConfig::blockSize>(bufA, bufB, noiseR);

    float gain_adj alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef20, gain * 0.25, gain_adj);
    const float sens_isq = invsq(sens);
    float sens_lp_coef alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 3], coef20,
                                          rerange01(sens_isq, coef500, coef50), sens_lp_coef);
    float threshold alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 4], coef20,
                                          rerange01(sens_isq, 0.125, 0.f), threshold);
    const float sr_scaled = 0.001 * this->sampleRate();
    float sens_lp_scale alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 5], coef20, rerange01(sens, 100.f, 50.f),
                                          sens_lp_scale);

    noise_channel_block(last, lastmin + 6, coef50, coef500, coef1000, sens_lp_scale, sens_lp_coef,
                        threshold, sr_scaled, srcL, noiseL, bufA);
    mul_block<FXConfig::blockSize>(bufA, gain_adj, bufA);
    clip_tanh78_block<FXConfig::blockSize>(10, 0.1, bufA, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 14], coef2000, bufA, bufB);
    sum2_block<FXConfig::blockSize>(srcL, bufB, dstL);

    noise_channel_block(last, lastmin + 15, coef50, coef500, coef1000, sens_lp_scale, sens_lp_coef,
                        threshold, sr_scaled, srcR, noiseR, bufB);
    mul_block<FXConfig::blockSize>(bufB, gain_adj, bufB);
    clip_tanh78_block<FXConfig::blockSize>(10, 0.1, bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 23], coef2000, bufB, bufA);
    sum2_block<FXConfig::blockSize>(srcR, bufA, dstR);
}

// 16 last slots
template <typename FXConfig>
inline void
Bonsai<FXConfig>::age_block(float last[], int lastmin, const float coef0, const float coef100,
                            const float coef700, const float coef1200, const float coef2000,
                            const float coef3000, const float dull, float *__restrict srcL,
                            float *__restrict srcR, float *__restrict dstL, float *__restrict dstR)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};

    const float dull_sq = dull * dull;
    const float dull_isq = invsq(dull);
    float gain alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 0], coef20,
                                          this->dbToLinear(rerange01(dull_isq, -36.f, 3.f)), gain);
    float gain_inv alignas(16)[FXConfig::blockSize] = {};
    negate_block<FXConfig::blockSize>(gain, gain_inv);
    float coef_highcut alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(
        last[lastmin + 1], coef20,
        freq_sr_to_alpha(rerange01(dull_isq, 20000.f, 2000.f), this->sampleRate()), coef_highcut);
    float coef_lowcut alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef20,
                                          rerange01(dull_sq, coef0, coef100), coef_lowcut);
    float cliplevel alignas(16)[FXConfig::blockSize] = {};
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 3], coef20,
                                          rerange01(dull_isq, 0.25, 0.075), cliplevel);

    high_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 4], coef3000, gain_inv, 50.f, 0.02,
                                             srcL, bufA);
    high_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 5], coef2000, gain, 20.f, 0.05, bufA,
                                             bufB);
    low_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 6], coef700, gain_inv,
                                            13.33333333333333333, 0.075, bufB, bufA);
    low_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 7], coef1200, gain, 25.f, 0.04, bufA,
                                            bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 8], coef_highcut, bufB, bufA);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 9], coef_lowcut, bufA, bufB);
    clip_tanh78_block<FXConfig::blockSize>(cliplevel, bufB, dstL);

    high_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 10], coef3000, gain_inv, 50.f, 0.02,
                                             srcR, bufA);
    high_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 11], coef2000, gain, 20.f, 0.05, bufA,
                                             bufB);
    low_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 12], coef700, gain_inv,
                                            13.33333333333333333, 0.075, bufB, bufA);
    low_shelf_nl_block<FXConfig::blockSize>(last[lastmin + 13], coef1200, gain, 25.f, 0.04, bufA,
                                            bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 14], coef_highcut, bufB, bufA);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 15], coef_lowcut, bufA, bufB);
    clip_tanh78_block<FXConfig::blockSize>(cliplevel, bufB, dstR);
}

template <typename FXConfig>
inline void Bonsai<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    float scaledL alignas(16)[FXConfig::blockSize] = {};
    float scaledR alignas(16)[FXConfig::blockSize] = {};
    float hpL alignas(16)[FXConfig::blockSize] = {};
    float hpR alignas(16)[FXConfig::blockSize] = {};
    float bassL alignas(16)[FXConfig::blockSize] = {};
    float bassR alignas(16)[FXConfig::blockSize] = {};
    // float bassboostedL alignas(16)[FXConfig::blockSize] = {};
    // float bassboostedR alignas(16)[FXConfig::blockSize] = {};
    float satL alignas(16)[FXConfig::blockSize] = {};
    float satR alignas(16)[FXConfig::blockSize] = {};
    float noiseL alignas(16)[FXConfig::blockSize] = {};
    float noiseR alignas(16)[FXConfig::blockSize] = {};
    float agedL alignas(16)[FXConfig::blockSize] = {};
    float agedR alignas(16)[FXConfig::blockSize] = {};
    float outL alignas(16)[FXConfig::blockSize] = {};
    float outR alignas(16)[FXConfig::blockSize] = {};
    mul_block<FXConfig::blockSize>(dataL, this->dbToLinear(this->floatValue(b_gain_in)), scaledL);
    mul_block<FXConfig::blockSize>(dataR, this->dbToLinear(this->floatValue(b_gain_in)), scaledR);
    onepole_hp_block<FXConfig::blockSize>(last[0], coef10, scaledL, hpL);
    onepole_hp_block<FXConfig::blockSize>(last[1], coef10, scaledR, hpR);
    bass_boost_block(last, 2, coef20, coef30, coef50, coef200, coef500,
                     this->dbToLinear(this->floatValue(b_bass_boost)),
                     this->floatValue(b_bass_distort), hpL, hpR, bassL, bassR);
    // sum2_block<FXConfig::blockSize>(hpL, bassL, bassboostedL);
    // sum2_block<FXConfig::blockSize>(hpR, bassR, bassboostedR);
    tape_sat_block(last, 31, coef4690, coef1280, coef160, coef99, coef3000, coef8000,
                   std::clamp(this->floatValue(b_tape_sat), 0.f, 1.f),
                   this->intValue(b_tape_dist_mode), bassL, bassR, satL, satR);
    tape_noise_block(last, 47, coef20, coef50, coef500, coef1000, coef2000,
                     this->floatValue(b_noise_sensitivity),
                     this->dbToLinear(this->floatValue(b_noise_gain)), satL, satR, noiseL, noiseR);
    age_block(last, 71, coef0, coef100, coef700, coef1200, coef2000, coef3000,
              this->floatValue(b_dull), noiseL, noiseR, agedL, agedR);
    onepole_hp_block<FXConfig::blockSize>(last[87], coef10, agedL, outL);
    onepole_hp_block<FXConfig::blockSize>(last[88], coef10, agedR, outR);
    mul_block<FXConfig::blockSize>(outL, this->dbToLinear(this->floatValue(b_gain_out)), outL);
    mul_block<FXConfig::blockSize>(outR, this->dbToLinear(this->floatValue(b_gain_out)), outR);
    lerp_block<FXConfig::blockSize>(dataL, outL, this->floatValue(b_mix), dataL);
    lerp_block<FXConfig::blockSize>(dataR, outR, this->floatValue(b_mix), dataR);
}

} // namespace sst::effects

#endif
