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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENCORRELATEDNOISE_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENCORRELATEDNOISE_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/CorrelatedNoise.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <random>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenCorrelatedNoise : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Generate Sin"};

    enum struct GenCorrelatedNoiseFloatParams : uint32_t
    {
        color,
        level,
        num_params
    };

    enum struct GenCorrelatedNoiseIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)GenCorrelatedNoiseFloatParams::num_params};
    static constexpr int numIntParams{(int)GenCorrelatedNoiseIntParams::num_params};

    // provide a function which is uniform bipolar float -1.f .. 1.f random values
    GenCorrelatedNoise()
        : core::VoiceEffectTemplateBase<VFXConfig>(), mGenerator((size_t)this), mDistro(-1.f, 1.f)
    {
        // Warm up
        for (int i = 0; i < 7; ++i)
        {
            sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(mPrior[0], mPrior[1], 0,
                                                                          mDistro(mGenerator));
        }
    }

    ~GenCorrelatedNoise() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)GenCorrelatedNoiseFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((GenCorrelatedNoiseFloatParams)idx)
        {
        case GenCorrelatedNoiseFloatParams::color:
            return pmd().asPercentBipolar().withName("Color");
        case GenCorrelatedNoiseFloatParams::level:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
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
        auto levT =
            std::clamp(this->getFloatParam((int)GenCorrelatedNoiseFloatParams::level), 0.f, 1.f);
        levT = levT * levT * levT;
        auto col =
            std::clamp(this->getFloatParam((int)GenCorrelatedNoiseFloatParams::color), -1.f, 1.f);
        mLevelLerp.set_target(levT);
        mColorLerp.newValue(col);
        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0], mPrior[1], mColorLerp.v, mDistro(mGenerator));
            dataoutR[k] = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0], mPrior[1], mColorLerp.v, mDistro(mGenerator));
            mColorLerp.process();
        }
        mLevelLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        auto levT =
            std::clamp(this->getFloatParam((int)GenCorrelatedNoiseFloatParams::level), 0.f, 1.f);
        levT = levT * levT * levT;
        mLevelLerp.set_target(levT);
        auto col =
            std::clamp(this->getFloatParam((int)GenCorrelatedNoiseFloatParams::color), -1.f, 1.f);
        mColorLerp.newValue(col);

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0], mPrior[1], mColorLerp.v, mDistro(mGenerator));
            mColorLerp.process();
        }
        mLevelLerp.multiply_block(dataoutL);
    }

  protected:
    float mPrior[2]{0.f, 0.f};

    std::minstd_rand mGenerator;
    std::uniform_real_distribution<float> mDistro;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> mLevelLerp;
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mColorLerp;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenCorrelatedNoise_H
