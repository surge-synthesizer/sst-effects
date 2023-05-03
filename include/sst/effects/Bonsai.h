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
    const float rc = 1.f / (2.f * M_PI * freq);
    // return delta / (rc + delta);
    // std::cout << delta << std::endl << rc << std::endl << (delta / (rc + delta)) << std::endl;
    return 1.f / (rc * sr + 1.f);
    // const auto temp = 2 * M_PI * delta * freq;
    // return temp / (temp + 1);
}
template <size_t blockSize>
inline void freq_sr_to_alpha_block(float *__restrict freq, float sr, float *__restrict coef)
{
    for (auto i = 1U; i < blockSize; ++i)
    {
        const float rc = 1.f / (2.f * M_PI * freq[i]);
        coef[i] = 1.f / (rc * sr + 1.f);
        // const auto temp = 2 * M_PI * delta * freq[i];
        // coef[i] = temp / (temp + 1);
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
        b_noise_mode,   // vinyl inspired or tape inspired
        b_noise_amount, // noise meta control, not volume compensated
        b_noise_gain,   // additional gain control of the noise

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
    void tape_sat_block(float last[], int lastmin, const float coef1, const float coef2,
                        const float coef3, const float coef4, const float coef5, const float coef6,
                        float sat, int mode, float *__restrict src, float *__restrict dst);
    void bass_boost_block(float last[], int lastmin, const float coef20, const float coef30,
                          const float coef50, const float coef200, const float coef500, float boost,
                          float dist, float *__restrict src, float *__restrict dst);

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
        // case b_bass_st_mono:
        //     return result.withType(pmd::INT).withName("Mono/Stereo").withRange(0,
        //     1).withDefault(0);
        case b_tape_sat:
            return result.withName("Tape Sat").withRange(0.f, 1.f).withDefault(0.25f);
        case b_tape_dist_mode:
            return result.withType(pmd::INT)
                .withName("Tape Dist Mode")
                .withRange(bdm_inv_sinh, bdm_sine)
                .withDefault(bdm_tanh);
        case b_noise_amount:
            return result.withName("Sensitivity").withRange(0.f, 1.f).withDefault(0.25f);
        case b_noise_gain:
            return result.withName("Gain").asDecibelNarrow().withDefault(0.f);
        case b_noise_mode:
            return result.withType(pmd::INT).withName("Mode").withRange(0, 1).withDefault(0);
        case b_dull:
            return result.withName("Dull").withRange(0.f, 1.f).withDefault(0.f);
        case b_gain_in:
            return result.withName("Input Gain").asDecibelNarrow().withDefault(0.f);
        case b_gain_out:
            return result.withName("Output Gain").asDecibelNarrow().withDefault(0.f);
        case b_mix:
            return result.withName("Mix").asPercent().withDefault(1.f);
        // case fl_mode:
        //     return result.withType(pmd::INT)
        //         .withName("Mode")
        //         .withDefault(0)
        //         .withRange(flm_classic, flm_arp_solo)
        //         .withUnorderedMapFormatting({{flm_classic, "Dry + Combs"},
        //                                      {flm_doppler, "Combs Only"},
        //                                      {flm_arp_mix, "Dry + Arp Combs"},
        //                                      {flm_arp_solo, "Arp Combs Only"}});
        // case fl_wave:
        //     return result.withType(pmd::INT)
        //         .withName("Waveform")
        //         .withDefault(0)
        //         .withRange(flw_sine, flw_square)
        //         .withUnorderedMapFormatting({{flw_sine, "Sine"},
        //                                      {flw_tri, "Triangle"},
        //                                      {flw_saw, "Sawtooth"},
        //                                      {flw_sng, "Noise"},
        //                                      {flw_snh, "Sample & Hold"},
        //                                      {flw_square, "Square"}});
        // case fl_rate:
        //     return result.withName("Rate")
        //         .withRange(-7, 9)
        //         .withDefault(-2.f)
        //         .temposyncable()
        //         .withATwoToTheBFormatting(1, 1, "Hz")
        //         .withDecimalPlaces(3);
        // case fl_depth:
        //     return result.withName("Depth").asPercent().withDefault(1.f);
        // case fl_voices:
        //     return result.withName("Count")
        //         .withRange(1.f, 4.f)
        //         .withDefault(4.f)
        //         .withLinearScaleFormatting("Voices");
        // case fl_voice_basepitch:
        //     return result.withName("Base Pitch").asMIDIPitch();
        // case fl_voice_spacing:
        //     return result.withName("Spacing")
        //         .withRange(0.f, 12.f)
        //         .withDefault(0.f)
        //         .withLinearScaleFormatting("semitones");

        // case fl_feedback:
        //     return result.withName("Feedback").asPercent().withDefault(0.f);
        // case fl_damping:
        //     return result.withName("HF Damping").asPercent().withDefault(0.1f);
        // case fl_width:
        //     return result.withName("Width").asDecibelNarrow().withDefault(0.f);
        // case fl_mix:
        //     return result.withName("Mix").asPercentBipolar().withDefault(0.8f);
        case b_num_params:
            throw std::logic_error("getParam called with num_params");
        }
        return result;
    }

    //   protected:
    //     static constexpr int COMBS_PER_CHANNEL = 4;
    //     struct InterpDelay
    //     {
    //         // OK so lets say we want lowest tunable frequency to be 23.5hz at 96k
    //         // 96000/23.5 = 4084
    //         // And lets future proof a bit and make it a power of 2 so we can use & properly
    //         static constexpr int DELAY_SIZE = 32768, DELAY_SIZE_MASK = DELAY_SIZE - 1;
    //         float line[DELAY_SIZE];
    //         int k = 0;
    //         InterpDelay() { reset(); }
    //         void reset()
    //         {
    //             memset(line, 0, DELAY_SIZE * sizeof(float));
    //             k = 0;
    //         }
    //         float value(float delayBy);
    //         void push(float nv)
    //         {
    //             k = (k + 1) & DELAY_SIZE_MASK;
    //             line[k] = nv;
    //         }
    //     };

    int ringout_value = -1;
    // InterpDelay idels[2];

    // float lfophase[2][COMBS_PER_CHANNEL], longphase[2];
    // float lpaL = 0.f, lpaR = 0.f; // state for the onepole LP filter

    // sdsp::lipol<float, FXConfig::blockSize, true> lfoval[2][COMBS_PER_CHANNEL],
    //     delaybase[2][COMBS_PER_CHANNEL];
    sdsp::lipol<float, FXConfig::blockSize, true> depth, mix;
    // sdsp::lipol<float, FXConfig::blockSize, true> voices, voice_detune, voice_chord;
    // sdsp::lipol<float, FXConfig::blockSize, true> feedback, fb_hf_damping;
    sdsp::SurgeLag<float> vzeropitch;
    // float lfosandhtarget[2][COMBS_PER_CHANNEL];
    // float vweights[2][COMBS_PER_CHANNEL];

    // sdsp::lipol_sse<FXConfig::blockSize, false> width;
    // bool haveProcessed{false};

    float last[35] = {};
    float sr = Bonsai<FXConfig>::sampleRate();
    const float coef_hb_hp = freq_sr_to_alpha(4690, sr);
    const float coef_hb_lp = freq_sr_to_alpha(1280, sr);
    const float coef_lb_hp = freq_sr_to_alpha(160, sr);
    const float coef_lb_lp = freq_sr_to_alpha(99, sr);
    const float coef_dist_hs1 = freq_sr_to_alpha(3000, sr);
    const float coef_dist_hs2 = freq_sr_to_alpha(8000, sr);
    const float coef10 = freq_sr_to_alpha(10, sr);
    const float coef20 = freq_sr_to_alpha(20, sr);
    const float coef30 = freq_sr_to_alpha(20, sr);
    const float coef50 = freq_sr_to_alpha(50, sr);
    const float coef200 = freq_sr_to_alpha(200, sr);
    const float coef500 = freq_sr_to_alpha(500, sr);

    // const static int LFO_TABLE_SIZE = 8192;
    // const static int LFO_TABLE_MASK = LFO_TABLE_SIZE - 1;
    // float sin_lfo_table[LFO_TABLE_SIZE];
    // float saw_lfo_table[LFO_TABLE_SIZE]; // don't make it analytic since I want to smooth the
    // edges
};

template <typename FXConfig> inline void Bonsai<FXConfig>::initialize()
{
    // for (int c = 0; c < 2; ++c)
    //     for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    //     {
    //         lfophase[c][i] = 1.f * (i + 0.5 * c) / COMBS_PER_CHANNEL;
    //         lfosandhtarget[c][i] = 0.0;
    //     }
    // longphase[0] = 0;
    // longphase[1] = 0.5;

    // for (int i = 0; i < LFO_TABLE_SIZE; ++i)
    // {
    //     sin_lfo_table[i] = sin(2.0 * M_PI * i / LFO_TABLE_SIZE);

    //     saw_lfo_table[i] = 0;

    //     // http://www.cs.cmu.edu/~music/icm-online/readings/panlaws/
    //     double panAngle = 1.0 * i / (LFO_TABLE_SIZE - 1) * M_PI / 2.0;
    //     auto piby2 = M_PI / 2.0;
    //     auto lW = sqrt((piby2 - panAngle) / piby2 * cos(panAngle));
    //     auto rW = sqrt(panAngle * sin(panAngle) / piby2);
    // }
    // haveProcessed = false;
}

// template <typename FXConfig> inline float Bonsai<FXConfig>::InterpDelay::value(float delayBy)
// {
//     // so if delayBy is 19.2
//     int itap = (int)std::min(delayBy, (float)(DELAY_SIZE - 2)); // this is 19
//     float fractap = delayBy - itap;                             // this is .2
//     int k0 = (k + DELAY_SIZE - itap - 1) & DELAY_SIZE_MASK;     // this is 20 back
//     int k1 = (k + DELAY_SIZE - itap) & DELAY_SIZE_MASK;         // this is 19 back
//     float result =
//         line[k0] * fractap + line[k1] * (1.0 - fractap); // FIXME move to the one mul form

//     return result;
// }

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
template <typename FXConfig>
inline void Bonsai<FXConfig>::tape_sat_block(float last[], int lastmin, const float coef_hb_hp,
                                             const float coef_hb_lp, const float coef_lb_hp,
                                             const float coef_lb_lp, const float coef_dist_hs1,
                                             const float coef_dist_hs2, float sat, int mode,
                                             float *__restrict src, float *__restrict dst)
{
    float bufA alignas(16)[FXConfig::blockSize] = {};
    float bufB alignas(16)[FXConfig::blockSize] = {};
    // float predist alignas(16)[FXConfig::blockSize] = {};
    // float predist_scaled alignas(16)[FXConfig::blockSize] = {};
    // float dist alignas(16)[FXConfig::blockSize] = {};
    // float untilt alignas(16)[FXConfig::blockSize] = {};
    // float untilt_scaled alignas(16)[FXConfig::blockSize] = {};
    // float dist2 alignas(16)[FXConfig::blockSize] = {};
    // float high_shelf alignas(16)[FXConfig::blockSize] = {};
    const float sat_invsq = invsq(sat);
    const float sat_halfsq = 0.5 * (sat * sat + sat);
    const float level = rerange01(sat_invsq, 0.15, 0.025);
    tilt_highboost_block<FXConfig::blockSize>(last[lastmin + 0], last[lastmin + 1], coef_hb_hp,
                                              coef_hb_lp, src, bufA);
    mul_block<FXConfig::blockSize>(bufA, this->dbToLinear(rerange01(sat, 0.f, 9.f)), bufA);
    std::cout << mode << (b_dist_modes)mode << std::endl;
    switch ((b_dist_modes)mode)
    {
    case bdm_inv_sinh:
        // std::cout << "bdm_inv_sinh" << std::endl;
        clip_inv_sinh_block<FXConfig::blockSize>(1 / level, level, bufA, bufA);
        break;
    case bdm_tanh:
        // std::cout << "bdm_tanh" << std::endl;
        clip_tanh78_block<FXConfig::blockSize>(1 / level, level, bufA, bufA);
        break;
    case bdm_tanh_approx_foldback:
        // std::cout << "bdm_tanh_approx_foldback" << std::endl;
        clip_tanh_foldback_block<FXConfig::blockSize>(1 / level, level, bufA, bufA);
        break;
    case bdm_sine:
        // std::cout << "bdm_sine" << std::endl;
        clip_sine_tanh_block<FXConfig::blockSize>(1 / level, level, bufA, bufA);
        break;
    default:
        // std::cout << "default" << std::endl;
        clip_tanh78_block<FXConfig::blockSize>(1 / level, level, bufA, bufA);
        break;
    }
    tilt_lowboost_block<FXConfig::blockSize>(last[lastmin + 2], last[lastmin + 3], coef_lb_hp,
                                             coef_lb_lp, bufA, bufB);
    mul_block<FXConfig::blockSize>(bufB, this->dbToLinear(rerange01(sat_halfsq, 1.5, 6.f)), bufB);
    clip_tanh78_block<FXConfig::blockSize>(6.6666666666666666, 0.15, bufB, bufB); // 1/0.15
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 4], coef_dist_hs1, -sat_invsq, bufB, bufA);
    high_shelf_block<FXConfig::blockSize>(last[lastmin + 5], coef_dist_hs2, -sat_invsq, bufA, dst);
}
template <typename FXConfig>
inline void Bonsai<FXConfig>::bass_boost_block(float last[], int lastmin, const float coef20,
                                               const float coef30, const float coef50,
                                               const float coef200, const float coef500,
                                               float boost, float dist, float *__restrict src,
                                               float *__restrict dst)
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
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 0], coef20, src, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 1], coef50, bufA, bufB);
    clip_tanh78_block<FXConfig::blockSize>(rerange01(distinvsq, 0.025, 0.01), bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 2], coef50, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 20.f, branch1);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 3], coef30, src, bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 4], coef200, bufA, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 5], coef200, bufB, reused);
    mul_block<FXConfig::blockSize>(reused, 5.f, branch2);
    mul_block<FXConfig::blockSize>(reused, rerange01(distsq, 0.125, 1.f), bufA);
    clampbi_block<FXConfig::blockSize>(0.01, reused, bufA);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 6], coef200, bufA, branch3);
    mul_block<FXConfig::blockSize>(branch3, rerange01(distsq, 0.5, 10.f), branch3);
    mul_block<FXConfig::blockSize>(reused, rerange01(distsq, 1.f, 5.f), bufB);
    clip_tanh78_block<FXConfig::blockSize>(rerange01(dist01, 0.075, 0.025), bufB, bufB);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 7], coef200, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, 2.f, bufA);
    sum2_block<FXConfig::blockSize>(branch3, bufA, bufB);
    onepole_hp_block<FXConfig::blockSize>(last[lastmin + 8], coef30, bufB, bufA);
    sum3_block<FXConfig::blockSize>(branch1, branch2, bufA, bufB);
    mul_block<FXConfig::blockSize>(bufB, 0.16666666666666666666666, bufB);
    // change the above constant to adjust the default gain, so the slider is negative less often
    // previous value: 0.3333333333333333333333333
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 9], coef500, bufB, bufA);
    mul_block<FXConfig::blockSize>(bufA, boost, bufA);
    clip_tanh78_block<FXConfig::blockSize>(rerange01(dist01, 0.1, 0.075), bufA, bufA);
    // mul_block<FXConfig::blockSize>(bufA, rerange01(dist01, 1.25, 0.75), bufA);
    onepole_lp_block<FXConfig::blockSize>(last[lastmin + 10], coef500, bufA, dst);
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
    float bassboostedL alignas(16)[FXConfig::blockSize] = {};
    float bassboostedR alignas(16)[FXConfig::blockSize] = {};
    float outL alignas(16)[FXConfig::blockSize] = {};
    float outR alignas(16)[FXConfig::blockSize] = {};
    mul_block<FXConfig::blockSize>(dataL, this->dbToLinear(this->floatValue(b_gain_in)), scaledL);
    mul_block<FXConfig::blockSize>(dataR, this->dbToLinear(this->floatValue(b_gain_in)), scaledR);
    onepole_hp_block<FXConfig::blockSize>(last[0], coef10, scaledL, hpL);
    onepole_hp_block<FXConfig::blockSize>(last[1], coef10, scaledR, hpR);
    bass_boost_block(last, 2, coef20, coef30, coef50, coef200, coef500,
                     this->dbToLinear(this->floatValue(b_bass_boost)),
                     this->floatValue(b_bass_distort), hpL, bassL);
    bass_boost_block(last, 13, coef20, coef30, coef50, coef200, coef500,
                     this->dbToLinear(this->floatValue(b_bass_boost)),
                     this->floatValue(b_bass_distort), hpR, bassR);
    sum2_block<FXConfig::blockSize>(hpL, bassL, bassboostedL);
    sum2_block<FXConfig::blockSize>(hpR, bassR, bassboostedR);
    tape_sat_block(last, 24, coef_hb_hp, coef_hb_lp, coef_lb_hp, coef_lb_lp, coef_dist_hs1,
                   coef_dist_hs2, std::clamp(this->floatValue(b_tape_sat), 0.f, 1.f),
                   this->intValue(b_tape_dist_mode), bassboostedL, outL);
    tape_sat_block(last, 30, coef_hb_hp, coef_hb_lp, coef_lb_hp, coef_lb_lp, coef_dist_hs1,
                   coef_dist_hs2, std::clamp(this->floatValue(b_tape_sat), 0.f, 1.f),
                   this->intValue(b_tape_dist_mode), bassboostedR, outR);
    mul_block<FXConfig::blockSize>(outL, this->dbToLinear(this->floatValue(b_gain_out)), outL);
    mul_block<FXConfig::blockSize>(outR, this->dbToLinear(this->floatValue(b_gain_out)), outR);
    lerp_block<FXConfig::blockSize>(dataL, outL, this->floatValue(b_mix), dataL);
    lerp_block<FXConfig::blockSize>(dataR, outR, this->floatValue(b_mix), dataR);
    // for (int i = 0; i < FXConfig::blockSize; ++i)
    // {
    //     dataL[i] = outL[i];
    //     dataR[i] = outR[i];
    // }
    // if (!haveProcessed)
    // {
    //     float v0 = this->floatValue(fl_voice_basepitch);
    //     if (v0 > 0)
    //         haveProcessed = true;
    //     vzeropitch.startValue(v0);
    // }
    // // So here is a flanger with everything fixed

    // float rate = this->envelopeRateLinear(-std::clamp(this->floatValue(fl_rate), -8.f, 10.f))
    // *
    //              this->temposyncRatio(fl_rate);

    // for (int c = 0; c < 2; ++c)
    // {
    //     longphase[c] += rate;
    //     if (longphase[c] >= COMBS_PER_CHANNEL)
    //         longphase[c] -= COMBS_PER_CHANNEL;
    // }

    // const float oneoverFreq0 = 1.0f / MIDI_0_FREQ;

    // int mode = this->intValue(fl_mode);
    // int mwave = this->intValue(fl_wave);
    // float depth_val = std::clamp(this->floatValue(fl_depth), 0.f, 2.f);

    // float v0 = this->floatValue(fl_voice_basepitch);
    // vzeropitch.newValue(v0);
    // vzeropitch.process();
    // v0 = vzeropitch.v;
    // float averageDelayBase = 0.0;

    // for (int c = 0; c < 2; ++c)
    //     for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    //     {
    //         bool lforeset = false;

    //         lfophase[c][i] += rate;

    //         if (lfophase[c][i] > 1)
    //         {
    //             lforeset = true;
    //             lfophase[c][i] -= 1;
    //         }

    //         float lfoout = lfoval[c][i].v;
    //         float thisphase = lfophase[c][i];

    //         if (mode == flm_arp_mix || mode == flm_arp_solo)
    //         {
    //             // arpeggio - everyone needs to use the same phase with the voice swap
    //             thisphase = longphase[c] - (int)longphase[c];
    //         }

    //         switch (mwave)
    //         {
    //         case flw_sine:
    //         {
    //             float ps = thisphase * LFO_TABLE_SIZE;
    //             int psi = (int)ps;
    //             float psf = ps - psi;
    //             int psn = (psi + 1) & LFO_TABLE_MASK;

    //             lfoout = sin_lfo_table[psi] * (1.0 - psf) + psf * sin_lfo_table[psn];

    //             lfoval[c][i].newValue(lfoout);

    //             break;
    //         }
    //         case flw_tri:
    //             lfoout = (2.f * fabs(2.f * thisphase - 1.f) - 1.f);
    //             lfoval[c][i].newValue(lfoout);
    //             break;
    //         case flw_saw: // Gentler than a pure saw, more like a heavily skewed triangle
    //         {
    //             float cutAt = 0.98;
    //             float usephase;

    //             if (thisphase < cutAt)
    //             {
    //                 usephase = thisphase / cutAt;
    //                 lfoout = usephase * 2.0f - 1.f;
    //             }
    //             else
    //             {
    //                 usephase = (thisphase - cutAt) / (1.0 - cutAt);
    //                 lfoout = (1.0 - usephase) * 2.f - 1.f;
    //             }

    //             lfoval[c][i].newValue(lfoout);

    //             break;
    //         }
    //         case flw_square:
    //         {
    //             auto cutOffset = 0.02f;
    //             auto m = 2.f / cutOffset;
    //             auto c2 = cutOffset / 2.f;

    //             if (thisphase < 0.5f - c2)
    //             {
    //                 lfoout = 1.f;
    //             }
    //             else if ((thisphase >= 0.5 + c2) && (thisphase <= 1.f - cutOffset))
    //             {
    //                 lfoout = -1.f;
    //             }
    //             else if ((thisphase > 0.5 - c2) && (thisphase < 0.5 + c2))
    //             {
    //                 lfoout = -m * thisphase + (m / 2);
    //             }
    //             else
    //             {
    //                 lfoout = (m * thisphase) - (2 * m) + m + 1;
    //             }

    //             lfoval[c][i].newValue(lfoout);

    //             break;
    //         }
    //         case flw_sng: // Sample & Hold random
    //         case flw_snh: // Sample & Glide smoothed random
    //         {
    //             if (lforeset)
    //             {
    //                 lfosandhtarget[c][i] = this->storageRand01() - 1.f;
    //             }

    //             if (mwave == flw_sng)
    //             {
    //                 // FIXME exponential creep up. We want to get there in time related to
    //                 our rate auto cv = lfoval[c][i].v; auto diff = (lfosandhtarget[c][i] -
    //                 cv) * rate
    //                 * 2; lfoval[c][i].newValue(cv + diff);
    //             }
    //             else
    //             {
    //                 lfoval[c][i].newValue(lfosandhtarget[c][i]);
    //             }
    //         }
    //         break;
    //         }

    //         auto combspace = this->floatValue(fl_voice_spacing);
    //         float pitch = v0 + combspace * i;
    //         float nv = this->sampleRate() * oneoverFreq0 *
    //         this->noteToPitchInv((float)(pitch));

    //         // OK so biggest tap = delaybase[c][i].v * ( 1.0 + lfoval[c][i].v * depth.v ) +
    //         1;
    //         // Assume lfoval is [-1,1] and depth is known
    //         float maxtap = nv * (1.0 + depth_val) + 1;
    //         if (maxtap >= InterpDelay::DELAY_SIZE)
    //         {
    //             nv = nv * 0.999 * InterpDelay::DELAY_SIZE / maxtap;
    //         }
    //         delaybase[c][i].newValue(nv);

    //         averageDelayBase += delaybase[c][i].new_v;
    //     }
    // averageDelayBase /= (2 * COMBS_PER_CHANNEL);
    // vzeropitch.process();

    // float dApprox = rate * this->sampleRate() / FXConfig::blockSize * averageDelayBase *
    // depth_val;

    // depth.newValue(depth_val);
    // mix.newValue(this->floatValue(fl_mix));
    // voices.newValue(std::clamp(this->floatValue(fl_voices), 1.f, 4.f));
    // float feedbackScale = 0.4 * sqrt((std::clamp(dApprox, 2.f, 60.f) + 30) / 100.0);

    // // Feedback adjust based on mode
    // switch (mode)
    // {
    // case flm_classic:
    // {
    //     float dv = (voices.v - 1);
    //     feedbackScale += (3.0 - dv) * 0.45 / 3.0;
    //     break;
    // }
    // case flm_doppler:
    // {
    //     float dv = (voices.v - 1);
    //     feedbackScale += (3.0 - dv) * 0.45 / 3.0;
    //     break;
    // }
    // case flm_arp_solo:
    // {
    //     // this is one voice doppler basically
    //     feedbackScale += 0.2;
    // }
    // case flm_arp_mix:
    // {
    //     // this is one voice classic basically and the steady signal clamps away feedback
    //     more feedbackScale += 0.3;
    // }
    // default:
    //     break;
    // }

    // float fbv = this->floatValue(fl_feedback);
    // if (fbv > 0)
    //     ringout_value = this->sampleRate() * 32.0;
    // else
    //     ringout_value = 1024;

    // if (mwave == flw_saw || mwave == flw_snh)
    // {
    //     feedbackScale *= 0.7;
    // }

    // if (fbv < 0)
    //     fbv = fbv;
    // else if (fbv > 1)
    //     fbv = fbv;
    // else
    //     fbv = sqrt(fbv);

    // feedback.newValue(feedbackScale * fbv);
    // fb_hf_damping.newValue(0.4 * this->floatValue(fl_damping));
    // float combs alignas(16)[2][FXConfig::blockSize];

    // // Obviously when we implement stereo spread this will be different
    // for (int c = 0; c < 2; ++c)
    // {
    //     for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    //         vweights[c][i] = 0;

    //     if (mode == flm_arp_mix || mode == flm_arp_solo)
    //     {
    //         int ilp = (int)longphase[c];
    //         float flp = longphase[c] - ilp;

    //         if (ilp == COMBS_PER_CHANNEL)
    //             ilp = 0;

    //         if (flp > 0.9)
    //         {
    //             float dt = (flp - 0.9) * 10; // this will be between 0,1
    //             float nxt = sqrt(dt);
    //             float prr = sqrt(1.f - dt);
    //             // std::cout << _D(longphase) << _D(dt) << _D(nxt) << _D(prr) << _D(ilp) <<
    //             _D(flp)
    //             // << std::endl;
    //             vweights[c][ilp] = prr;
    //             if (ilp == COMBS_PER_CHANNEL - 1)
    //                 vweights[c][0] = nxt;
    //             else
    //                 vweights[c][ilp + 1] = nxt;
    //         }
    //         else
    //         {
    //             vweights[c][ilp] = 1.f;
    //         }
    //     }
    //     else
    //     {
    //         float voices = std::clamp(this->floatValue(fl_voices), 1.f, COMBS_PER_CHANNEL
    //         * 1.f); vweights[c][0] = 1.0;

    //         for (int i = 0; i < voices && i < 4; ++i)
    //             vweights[c][i] = 1.0;

    //         int li = (int)voices;
    //         float fi = voices - li;
    //         if (li < 4)
    //             vweights[c][li] = fi;
    //     }
    // }

    // for (int b = 0; b < FXConfig::blockSize; ++b)
    // {
    //     for (int c = 0; c < 2; ++c)
    //     {
    //         combs[c][b] = 0;
    //         for (int i = 0; i < COMBS_PER_CHANNEL; ++i)
    //         {
    //             if (vweights[c][i] > 0)
    //             {
    //                 auto tap = delaybase[c][i].v * (1.0 + lfoval[c][i].v * depth.v) + 1;
    //                 auto v = idels[c].value(tap);
    //                 combs[c][b] += vweights[c][i] * v;
    //             }

    //             lfoval[c][i].process();
    //             delaybase[c][i].process();
    //         }
    //     }
    //     // softclip the feedback to avoid explosive runaways
    //     float fbl = 0.f;
    //     float fbr = 0.f;
    //     if (feedback.v > 0)
    //     {
    //         fbl = std::clamp(feedback.v * combs[0][b], -1.f, 1.f);
    //         fbr = std::clamp(feedback.v * combs[1][b], -1.f, 1.f);

    //         fbl = 1.5 * fbl - 0.5 * fbl * fbl * fbl;
    //         fbr = 1.5 * fbr - 0.5 * fbr * fbr * fbr;

    //         // and now we have clipped, apply the damping. FIXME - move to one mul form
    //         float df = std::clamp(fb_hf_damping.v, 0.01f, 0.99f);
    //         lpaL = lpaL * (1.0 - df) + fbl * df;
    //         fbl = fbl - lpaL;

    //         lpaR = lpaR * (1.0 - df) + fbr * df;
    //         fbr = fbr - lpaR;
    //     }

    //     auto vl = dataL[b] - fbl;
    //     auto vr = dataR[b] - fbr;
    //     idels[0].push(vl);
    //     idels[1].push(vr);

    //     auto origw = 1.f;
    //     if (mode == flm_doppler || mode == flm_arp_solo)
    //     {
    //         // doppler modes
    //         origw = 0.f;
    //     }

    //     float outl = origw * dataL[b] + mix.v * combs[0][b];
    //     float outr = origw * dataR[b] + mix.v * combs[1][b];

    //     // Some gain heueirstics
    //     float gainadj = 0.0;
    //     switch (mode)
    //     {
    //     case flm_classic:
    //         gainadj = -1 / sqrt(7 - voices.v);
    //         break;
    //     case flm_doppler:
    //         gainadj = -1 / sqrt(8 - voices.v);
    //         break;
    //     case flm_arp_mix:
    //         gainadj = -1 / sqrt(6);
    //         break;
    //     case flm_arp_solo:
    //         gainadj = -1 / sqrt(7);
    //         break;
    //     }

    //     gainadj -= 0.07 * mix.v;

    //     outl = std::clamp((1.0f + gainadj) * outl, -1.f, 1.f);
    //     outr = std::clamp((1.0f + gainadj) * outr, -1.f, 1.f);

    //     outl = 1.5 * outl - 0.5 * outl * outl * outl;
    //     outr = 1.5 * outr - 0.5 * outr * outr * outr;

    //     dataL[b] = outl;
    //     dataR[b] = outr;

    //     depth.process();
    //     mix.process();
    //     feedback.process();
    //     fb_hf_damping.process();
    //     voices.process();
    // }

    // width.set_target_smoothed(this->dbToLinear(this->floatValue(fl_width)) / 3);

    // this->applyWidth(dataL, dataR, width);
}

} // namespace sst::effects

#endif
