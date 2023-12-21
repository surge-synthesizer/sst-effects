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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENPHASEMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENPHASEMOD_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FastMath.h"
#include "sst/filters/HalfRateFilter.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenPhaseMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"GenPhaseMod"};

    enum struct GenPhaseModFloatParams : uint32_t
    {
        transpose,
        depth,
        num_params
    };

    enum struct GenPhaseModIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)GenPhaseModFloatParams::num_params};
    static constexpr int numIntParams{(int)GenPhaseModIntParams::num_params};

    GenPhaseMod() : core::VoiceEffectTemplateBase<VFXConfig>(), pre(6, true), post(6, true) {}

    ~GenPhaseMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)GenPhaseModFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((GenPhaseModFloatParams)idx)
        {
        case GenPhaseModFloatParams::transpose:
            return pmd()
                .asFloat()
                .withRange(-96, 96)
                .withLinearScaleFormatting("semitones")
                .withDefault(0)
                .withName("Transpose");
        case GenPhaseModFloatParams::depth:
            return pmd().asDecibel().withName("Depth").withDefault(0);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;

        omega.set_target(0.5 * 440 *
                         this->note_to_pitch_ignoring_tuning(
                             pitch + this->getFloatParam((int)GenPhaseModFloatParams::transpose)) *
                         M_PI_2 * this->getSampleRateInv());
        pregain.set_target(
            3.1415 * this->dbToLinear(this->getFloatParam((int)GenPhaseModFloatParams::depth)));

        constexpr int bs2 = VFXConfig::blockSize << 1;
        float OS alignas(16)[2][bs2];
        float omInterp alignas(16)[bs2];
        float phVals alignas(16)[bs2];

        pregain.multiply_2_blocks_to(datainL, datainR, OS[0], OS[1]);
        pre.process_block_U2(OS[0], OS[1], OS[0], OS[1], bs2);

        omega.store_block(omInterp);
        phVals[0] = phase + omInterp[0];
        for (int k = 1; k < bs2; ++k)
        {
            phVals[k] = phVals[k - 1] + omInterp[k];
        }
        phase = phVals[bs2 - 1];
        if (phase > M_PI)
            phase -= 2 * M_PI;

        namespace bd = sst::basic_blocks::dsp;
        const auto half = _mm_set1_ps(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = _mm_load_ps(phVals + k);
            auto s0 = _mm_load_ps(OS[0] + k);
            auto s1 = _mm_load_ps(OS[1] + k);
            auto w0 = sdsp::clampToPiRangeSSE(_mm_add_ps(s0, ph));
            auto w1 = sdsp::clampToPiRangeSSE(_mm_add_ps(s1, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = _mm_mul_ps(half, _mm_sub_ps(sdsp::fastsinSSE(w0), sph));
            auto r1 = _mm_mul_ps(half, _mm_sub_ps(sdsp::fastsinSSE(w1), sph));
            _mm_store_ps(OS[0] + k, r0);
            _mm_store_ps(OS[1] + k, r1);
        }

        post.process_block_D2(OS[0], OS[1], bs2, dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;

        omega.set_target(0.5 * 440 *
                         this->note_to_pitch_ignoring_tuning(
                             pitch + this->getFloatParam((int)GenPhaseModFloatParams::transpose)) *
                         M_PI_2 * this->getSampleRateInv());
        pregain.set_target(
            3.1415 * this->dbToLinear(this->getFloatParam((int)GenPhaseModFloatParams::depth)));

        constexpr int bs2 = VFXConfig::blockSize << 1;
        float OS alignas(16)[2][bs2];
        float omInterp alignas(16)[bs2];
        float phVals alignas(16)[bs2];

        pregain.multiply_block_to(datainL, OS[0]);
        pre.process_block_U2(OS[0], OS[0], OS[0], OS[1], bs2);

        omega.store_block(omInterp);
        phVals[0] = phase + omInterp[0];
        for (int k = 1; k < bs2; ++k)
        {
            phVals[k] = phVals[k - 1] + omInterp[k];
        }
        phase = phVals[bs2 - 1];
        if (phase > M_PI)
            phase -= 2 * M_PI;

        namespace bd = sst::basic_blocks::dsp;
        const auto half = _mm_set1_ps(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = _mm_load_ps(phVals + k);
            auto s0 = _mm_load_ps(OS[0] + k);
            auto w0 = sdsp::clampToPiRangeSSE(_mm_add_ps(s0, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = _mm_mul_ps(half, _mm_sub_ps(sdsp::fastsinSSE(w0), sph));
            _mm_store_ps(OS[0] + k, r0);
        }

        post.process_block_D2(OS[0], OS[0], bs2, dataoutL, 0);
    }

  protected:
    double phase{0.0};
    sst::filters::HalfRate::HalfRateFilter pre, post;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> pregain;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize << 1, true> omega;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenPhaseMod_H
