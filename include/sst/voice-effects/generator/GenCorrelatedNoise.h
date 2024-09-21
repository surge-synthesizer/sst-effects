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
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenCorrelatedNoise : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Correlated Noise"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{1};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpColor,
        fpLevel,
        fpStereoWidth,
    };

    enum IntParams
    {
        ipStereo,
    };

    // provide a function which is uniform bipolar float -1.f .. 1.f random values
    GenCorrelatedNoise() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        // Warm up
        for (int i = 0; i < 7; ++i)
        {
            sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0][0], mPrior[0][1], 0, rng.unifPM1());
            sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[1][0], mPrior[1][1], 0, rng.unifPM1());
        }
    }

    ~GenCorrelatedNoise() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case FloatParams::fpColor:
            return pmd().asPercentBipolar().withName("Color");
        case FloatParams::fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
        case FloatParams::fpStereoWidth:
            return pmd()
                .asFloat()
                .withRange(0.f, 2.f)
                .withDefault(1.f)
                .withLinearScaleFormatting("%", 100)
                .withName(!stereo ? std::string() : "Stereo Width");
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        return basic_blocks::params::ParamMetaData().asBool().withDefault(true).withName(
            "Stereo Noise");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void midSideAdjust(float width, float leftIn, float rightIn, float &leftOut, float &rightOut)
    {
        float midIn = 0.f, sideIn = 0.f, midOut = 0.f, sideOut = 0.f;

        midIn = (leftIn + rightIn) / 2;
        sideIn = (leftIn - rightIn) / 2;

        sideIn *= width;

        // TODO: loudness compensation...
        leftOut = midIn + sideIn;
        rightOut = midIn - sideIn;
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        auto isStereo = this->getIntParam(ipStereo) != 0;
        if (!isStereo)
        {
            processMonoToMono(datainL, dataoutL, pitch);
            basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
            return;
        }
        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;
        auto col = std::clamp(this->getFloatParam(fpColor), -1.f, 1.f);
        mLevelLerp.set_target(levT);
        mColorLerp.newValue(col);

        auto widthParam = std::clamp(this->getFloatParam(fpStereoWidth), 0.f, 2.f);

        // remember getOversamplingRatio is constexpr so the compiler
        // will get a good shot at this loop
        for (int k = 0; k < VFXConfig::blockSize; k += this->getOversamplingRatio())
        {
            auto runLeft = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0][0], mPrior[0][1], mColorLerp.v, rng.unifPM1());
            auto runRight = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[1][0], mPrior[1][1], mColorLerp.v, rng.unifPM1());

            if (isStereo)
            {
                midSideAdjust(widthParam, runLeft, runRight, dataoutL[k], dataoutR[k]);
            }

            for (auto kk = 1; kk < this->getOversamplingRatio(); ++kk)
            {
                dataoutL[k + kk] = dataoutL[k];
                dataoutR[k + kk] = dataoutR[k];

                mColorLerp.process(); // inside since LERP is at OS
            }
        }
        mLevelLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;
        mLevelLerp.set_target(levT);
        auto col = std::clamp(this->getFloatParam(fpColor), -1.f, 1.f);
        mColorLerp.newValue(col);

        for (int k = 0; k < VFXConfig::blockSize; k += this->getOversamplingRatio())
        {
            dataoutL[k] = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0][0], mPrior[0][1], mColorLerp.v, rng.unifPM1());
            for (auto kk = 1; kk < this->getOversamplingRatio(); ++kk)
            {
                dataoutL[k + kk] = dataoutL[k];

                mColorLerp.process(); // inside since this LERP is at OS
            }
        }
        mLevelLerp.multiply_block(dataoutL);
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool checkParameterConsistency() const { return true; }

  protected:
    float mPrior[2][2]{{0.f, 0.f}, {0.f, 0.f}};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> mLevelLerp;
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mColorLerp;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenCorrelatedNoise_H
