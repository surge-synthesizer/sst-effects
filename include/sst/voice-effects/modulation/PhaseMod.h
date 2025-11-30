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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASEMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASEMOD_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FastMath.h"
#include "sst/filters/HalfRateFilter.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct PhaseMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"PhaseMod"};
    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{2};

    enum FloatParams
    {
        fpTranspose,
        fpDepth,
    };

    enum IntParams
    {
        ipNum,
        ipDenom,
    };

    PhaseMod() : core::VoiceEffectTemplateBase<VFXConfig>(), pre(6, true), post(6, true) {}

    ~PhaseMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpTranspose:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-96, 96)
                    .withSemitoneFormatting()
                    .withDefault(0)
                    .withName("Offset");
            }
            else
            {
                return pmd().asAudibleFrequency().withDefault(0).withName("Frequency");
            }
        case fpDepth:
            return pmd().asDecibel().withName("Depth").withDefault(0);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipNum:
            return pmd()
                .asInt()
                .withRange(1, 16)
                .withDefault(1)
                .withName("Numerator")
                .withDimensionlessFormatting();
        case ipDenom:
            return pmd()
                .asInt()
                .withRange(1, 16)
                .withDefault(1)
                .withName("Denominator")
                .withDimensionlessFormatting();
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float getRatio()
    {
        auto num = (float)(this->getIntParam(ipNum));
        auto denom = (float)(this->getIntParam(ipDenom));
        if (num == denom)
        {
            return 0.f;
        }

        auto ratio = num / denom;
        return 12 * std::log2(ratio);
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;

        auto freq = this->getFloatParam(fpTranspose) + getRatio();
        if (keytrackOn)
        {
            freq += pitch;
        }

        omega.set_target(0.5 * 440 * this->note_to_pitch_ignoring_tuning(freq) * M_PI_2 *
                         this->getSampleRateInv());
        pregain.set_target(3.1415 * this->dbToLinear(this->getFloatParam(fpDepth)));

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

        const auto half = SIMD_MM(set1_ps)(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = SIMD_MM(load_ps)(phVals + k);
            auto s0 = SIMD_MM(load_ps)(OS[0] + k);
            auto s1 = SIMD_MM(load_ps)(OS[1] + k);
            auto w0 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s0, ph));
            auto w1 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s1, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w0), sph));
            auto r1 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w1), sph));
            SIMD_MM(store_ps)(OS[0] + k, r0);
            SIMD_MM(store_ps)(OS[1] + k, r1);
        }

        post.process_block_D2(OS[0], OS[1], bs2, dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;

        auto freq = this->getFloatParam(fpTranspose) + getRatio();
        if (keytrackOn)
        {
            freq += pitch;
        }

        omega.set_target(0.5 * 440 * this->note_to_pitch_ignoring_tuning(freq) * M_PI_2 *
                         this->getSampleRateInv());
        pregain.set_target(3.1415 * this->dbToLinear(this->getFloatParam(fpDepth)));

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

        const auto half = SIMD_MM(set1_ps)(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = SIMD_MM(load_ps)(phVals + k);
            auto s0 = SIMD_MM(load_ps)(OS[0] + k);
            auto w0 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s0, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w0), sph));
            SIMD_MM(store_ps)(OS[0] + k, r0);
        }

        post.process_block_D2(OS[0], OS[0], bs2, dataoutL, 0);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }

  protected:
    bool keytrackOn{true};
    double phase{0.0};
    sst::filters::HalfRate::HalfRateFilter pre, post;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> pregain;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize << 1, true> omega;

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};
} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_PhaseMod_H
