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
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/filters/FastTiltNoiseFilter.h"
#include "sst/effects-shared/WidthProvider.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig>
struct TiltNoise : core::VoiceEffectTemplateBase<VFXConfig>,
                   effects_shared::WidthProvider<TiltNoise<VFXConfig>, VFXConfig::blockSize, true>
{
    static constexpr const char *displayName{"Tilt Noise"};
    static constexpr const char *streamingName{"osc-tilt-noise"};

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

    TiltNoise() : core::VoiceEffectTemplateBase<VFXConfig>(), FiltersL(*this), FiltersR(*this) {}

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
            return this->getWidthParam()
                .withName(!stereo ? std::string() : "Width")
                .withRange(0.f, 2.f);
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        }

        return pmd().asStereoSwitch().withDefault(true);
    }

    void initVoiceEffect()
    {
        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;

        float initNoiseL[11];
        float initNoiseR[11];

        for (int i = 0; i < 11; ++i)
        {
            initNoiseL[i] = rng.unifPM1();
            initNoiseR[i] = rng.unifPM1();
        }

        FiltersL.init(initNoiseL, slope);
        FiltersR.init(initNoiseR, slope);
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

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

        this->setWidthTarget(widthLerpS, widthLerpM, fpStereoWidth);

        float noiseL[VFXConfig::blockSize]{};
        float noiseR[VFXConfig::blockSize]{};
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            noiseL[i] = rng.unifPM1() * atten[i];
            noiseR[i] = stereo ? rng.unifPM1() * atten[i] : noiseL[i];
        }

        if (stereo)
        {
            this->applyWidth(noiseL, noiseR, widthLerpS, widthLerpM);
        }

        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;
        FiltersL.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersR.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersL.template processBlock<VFXConfig::blockSize>(noiseL, dataoutL);
        FiltersR.template processBlock<VFXConfig::blockSize>(noiseR, dataoutR);

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

        float noise[VFXConfig::blockSize]{};

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            noise[i] = rng.unifPM1() * atten[i];
        }

        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;
        FiltersL.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersL.template processBlock<VFXConfig::blockSize>(noise, dataout);

        levelLerp.multiply_block(dataout);
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
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

        this->setWidthTarget(widthLerpS, widthLerpM, fpStereoWidth);

        float noiseL[VFXConfig::blockSize]{};
        float noiseR[VFXConfig::blockSize]{};
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            noiseL[i] = rng.unifPM1() * atten[i];
            noiseR[i] = rng.unifPM1() * atten[i];
        }

        this->applyWidth(noiseL, noiseR, widthLerpS, widthLerpM);

        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;
        FiltersL.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersR.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersL.template processBlock<VFXConfig::blockSize>(noiseL, dataoutL);
        FiltersR.template processBlock<VFXConfig::blockSize>(noiseR, dataoutR);

        levelLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    sst::filters::FastTiltNoiseFilter<TiltNoise> FiltersL, FiltersR;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> levelLerp, widthLerpS, widthLerpM,
        attenLerp;

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
