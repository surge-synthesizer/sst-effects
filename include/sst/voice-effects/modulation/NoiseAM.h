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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_NOISEAM_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_NOISEAM_H

#include "../VoiceEffectCore.h"

#include <iostream>
#include <math.h>
#include <algorithm>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/RNG.h"
#include "sst/filters/FastTiltNoiseFilter.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct NoiseAM : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Noise AM"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{2};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpThreshold,
        fpDepth,
        fpTilt
    };

    enum IntParams
    {
        ipMode,
        ipStereo
    };

    NoiseAM() : core::VoiceEffectTemplateBase<VFXConfig>(), FiltersL(*this), FiltersR(*this) {}

    ~NoiseAM() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool mode = this->getIntParam(ipMode);

        switch (idx)
        {
        case fpThreshold:
            return pmd().asPercent().withDefault(.75f).withName("Threshold");
        case fpDepth:
            return pmd().asPercent().withDefault(.5f).withName("Depth");
        case fpTilt:
            return pmd()
                .asDecibelNarrow()
                .withRange(-6.f, 6.f)
                .withName("Tilt")
                .withCustomMinDisplay("Red")
                .withCustomDefaultDisplay("White")
                .withCustomMaxDisplay("Violet")
                .withDefault(0);
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipMode:
            return pmd()
                .asBool()
                .withDefault(true)
                .withUnorderedMapFormatting({{false, "One"}, {true, "Two"}})
                .withName("Mode");
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        }
        return pmd().asInt().withName("error");
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
        // yup!
        FiltersL.init(initNoiseL, slope);
        FiltersR.init(initNoiseR, slope);
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void makeNoise(float *L, float *R)
    {
        float tilt = this->getFloatParam(fpTilt);
        if (tilt > 0)
        {
            tilt *= -4.f;
        }
        tilt = this->dbToLinear(tilt);
        attenLerp.set_target(tilt);
        float atten alignas(16)[VFXConfig::blockSize];
        attenLerp.store_block(atten);

        float noiseL[VFXConfig::blockSize]{};
        float noiseR[VFXConfig::blockSize]{};
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            noiseL[i] = rng.unifPM1() * atten[i];
            noiseR[i] = rng.unifPM1() * atten[i];
        }

        float slope = std::clamp(this->getFloatParam(fpTilt), -6.f, 6.f) / 2.f;
        FiltersL.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersR.template setCoeffForBlock<VFXConfig::blockSize>(slope);
        FiltersL.template processBlock<VFXConfig::blockSize>(noiseL, L);
        FiltersR.template processBlock<VFXConfig::blockSize>(noiseR, R);
    }

    void makeNoise(float *C)
    {
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
        FiltersL.template processBlock<VFXConfig::blockSize>(noise, C);
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        bool stereo = this->getIntParam(ipStereo);
        float threshold = this->getFloatParam(fpThreshold);
        auto depth = this->getFloatParam(fpDepth);
        bool mode = this->getIntParam(ipMode);

        if (mode)
        {
            threshold *= 2.f;
            threshold -= 1.f;
        }

        float noiseL alignas(16)[VFXConfig::blockSize];
        float noiseR alignas(16)[VFXConfig::blockSize];
        basic_blocks::mechanics::clear_block<VFXConfig::blockSize>(noiseL);
        basic_blocks::mechanics::clear_block<VFXConfig::blockSize>(noiseR);

        if (stereo)
        {
            makeNoise(noiseL, noiseR);
        }
        else
        {
            makeNoise(noiseL);
            basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(noiseL, noiseR);
        }

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto envL = datainL[i];
            auto envR = datainR[i];

            float overL = 0.f;
            float overR = 0.f;

            if (!mode) // in bipolar mode
            {
                // the env is absolute
                envL = fabsf(envL);
                envR = fabsf(envR);

                // and we add some offset to the noise, more so at high depth
                noiseL[i] = noiseL[i] + (0.75f * (depth * depth * depth));
                noiseR[i] = noiseR[i] + (0.75f * (depth * depth * depth));

                // subtracting the noise if the input is positive
                if (datainL[i] > 0)
                {
                    noiseL[i] *= -1;
                }
                if (datainR[i] > 0)
                {
                    noiseR[i] *= -1;
                }
                // this keeps the combined peaks under control and saturates the signal a bit

                // this is the overshoot calculation
                overL = std::max(envL - threshold, 0.f);
                overR = std::max(envR - threshold, 0.f);
            }
            else
            {
                // which is reversed in unipolar mode, just so it plays nicer with asym waveshapes
                overL = std::min(envL + threshold, 0.f);
                overR = std::min(envR + threshold, 0.f);
                // runaway peaks are a bit less problematic here so don't mess with the noise
            }

            noiseL[i] *= depth * overL;
            noiseR[i] *= depth * overR;

            dataoutL[i] = datainL[i] + noiseL[i];
            dataoutR[i] = datainR[i] + noiseR[i];
        }
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        float threshold = this->getFloatParam(fpThreshold);
        auto depth = this->getFloatParam(fpDepth);
        bool mode = this->getIntParam(ipMode) > 0;

        if (mode)
        {
            threshold *= 2.f;
            threshold -= 1.f;
        }

        float noise alignas(16)[VFXConfig::blockSize];
        basic_blocks::mechanics::clear_block<VFXConfig::blockSize>(noise);

        makeNoise(noise);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto env = datain[i];

            float over = 0.f;

            if (!mode)
            {
                env = fabsf(env);

                noise[i] = noise[i] + (0.75f * (depth * depth * depth));

                if (datain[i] > 0)
                {
                    noise[i] *= -1;
                }

                over = std::max(env - threshold, 0.f);
            }
            else
            {
                over = std::min(env + threshold, 0.f);
            }

            noise[i] *= depth * over;

            dataout[i] = datain[i] + noise[i];
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    sst::filters::FastTiltNoiseFilter<NoiseAM> FiltersL, FiltersR;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> depthLerp, attenLerp;

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
#endif // SCXT_NOISEAM_H
