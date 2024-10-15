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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_TILTNOISE_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_TILTNOISE_H

#include "../VoiceEffectCore.h"

#include <iostream>
#include <math.h>
#include <algorithm>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct TiltNoise : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Tilt Noise"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{1};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpLevel,
        fpTilt,
        fpStereoWidth
    };

    enum IntParams
    {
        ipStereo,
    };

    TiltNoise() : core::VoiceEffectTemplateBase<VFXConfig>() {}
    ~TiltNoise() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
        case fpTilt:
            return pmd()
                .asDecibelNarrow()
                .withRange(-6.f, 6.f)
                .withName("Tilt")
                .withCustomMinDisplay("Red")
                .withCustomDefaultDisplay("White")
                .withCustomMaxDisplay("Violet")
                .withDefault(0);
        case fpStereoWidth:
            return pmd()
                .asFloat()
                .withRange(0.f, 2.f)
                .withDefault(1.f)
                .withLinearScaleFormatting("%", 100)
                .withName(!stereo ? std::string() : "Stereo Width");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd().asBool().withDefault(true).withName("Stereo");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void setCoeffs()
    {
        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;
        float posGain = this->dbToLinear(slope);
        float negGain = this->dbToLinear(-1 * slope);
        float res = .07f;

        if (slope != priorSlope)
        {
            for (int i = 0; i < 11; ++i)
            {
                float freq = powf(2, (i + 1.f)) * 10.f;
                if (i < 6)
                {
                    filters[i].template setCoeffForBlock<VFXConfig::blockSize>(
                        filters::CytomicSVF::Mode::LOW_SHELF, freq, res, this->getSampleRateInv(),
                        negGain);
                }
                else
                {
                    filters[i].template setCoeffForBlock<VFXConfig::blockSize>(
                        filters::CytomicSVF::Mode::HIGH_SHELF, freq, res, this->getSampleRateInv(),
                        posGain);
                }
            }
            priorSlope = slope;
        }
        else
        {
            for (int i = 0; i < 11; i++)
            {
                filters[i].template retainCoeffForBlock<VFXConfig::blockSize>();
            }
        }
    }

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
        bool stereo = this->getIntParam(ipStereo);

        float tilt = this->getFloatParam(fpTilt);
        if (tilt > 0)
        {
            tilt *= -4.f;
        }
        tilt = this->dbToLinear(tilt);
        attenLerp.set_target(tilt);
        float atten alignas(16)[VFXConfig::blockSize];
        attenLerp.store_block(atten);

        float level = this->getFloatParam(fpLevel);
        level = level * level * level;
        levelLerp.set_target(level);

        widthLerp.set_target(std::clamp(this->getFloatParam(fpStereoWidth), 0.f, 2.f));
        float width alignas(16)[VFXConfig::blockSize];
        widthLerp.store_block(width);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto noiseL = rng.unifPM1();
            auto noiseR = stereo ? rng.unifPM1() : noiseL;
            if (stereo)
            {
                midSideAdjust(width[i], noiseL, noiseR, dataoutL[i], dataoutR[i]);
            }
            else
            {
                dataoutL[i] = noiseL;
                dataoutR[i] = noiseR;
            }
            dataoutL[i] *= atten[i];
            dataoutR[i] *= atten[i];
        }

        setCoeffs();
        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL,
                                                                   dataoutR);
        }
        levelLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        float level = this->getFloatParam(fpLevel);
        level = level * level * level;
        levelLerp.set_target(level);

        float tilt = this->getFloatParam(fpTilt);
        if (tilt > 0)
        {
            tilt *= -4.f;
        }
        tilt = this->dbToLinear(tilt);
        attenLerp.set_target(tilt);
        float atten alignas(16)[VFXConfig::blockSize];
        attenLerp.store_block(atten);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataout[i] = rng.unifPM1();
            dataout[i] *= atten[i];
        }

        setCoeffs();
        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(dataout, dataout);
        }
        levelLerp.multiply_block(dataout);
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    float priorSlope = -1324.f;
    std::array<sst::filters::CytomicSVF, 11> filters;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> levelLerp, widthLerp, attenLerp;

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
} // namespace sst::voice_effects::generator
#endif // SCXT_TILTNOISE_H
