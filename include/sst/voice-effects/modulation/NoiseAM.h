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

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/RNG.h"

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

    NoiseAM() : core::VoiceEffectTemplateBase<VFXConfig>() {}

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
                .withDefault(false)
                .withUnorderedMapFormatting({{false, "Bipolar"}, {true, "Unipolar"}})
                .withName("Mode");
        case ipStereo:
            return pmd().asBool().withDefault(true).withName("Stereo");
        }
        return pmd().asInt().withName("error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void setCoeffs()
    {
        float slope = this->getFloatParam(fpTilt) / 2;
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

    void makeNoise(float *L, float *R)
    {
        float atten = this->getFloatParam(fpTilt);
        if (atten > 0)
        {
            atten *= -4.f;
        }
        atten = this->dbToLinear(atten);

        setCoeffs();

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            L[i] = rng.unifPM1();
            R[i] = rng.unifPM1();

            L[i] *= atten;
            R[i] *= atten;
        }
        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(L, R, L, R);
        }
    }

    void makeNoise(float *C)
    {
        float atten = this->getFloatParam(fpTilt);
        if (atten > 0)
        {
            atten *= -4.f;
        }
        atten = this->dbToLinear(atten);

        setCoeffs();

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            C[i] = rng.unifPM1();
            C[i] *= atten;
        }
        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(C, C);
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        bool stereo = this->getIntParam(ipStereo);
        float threshold = this->getFloatParam(fpThreshold);
        auto depth = this->getFloatParam(fpDepth);
        bool mode = this->getIntParam(ipMode);

        if (mode)
        {
            threshold *= 2.f;
            threshold -= 1.f;
            depth *= .5;
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
            noiseL[i] *= depth;
            noiseR[i] *= depth;

            auto envL = mode ? datainL[i] : fabsf(datainL[i]);
            auto envR = mode ? datainR[i] : fabsf(datainR[i]);

            auto overL = std::max(envL - threshold, 0.f);
            auto overR = std::max(envR - threshold, 0.f);

            noiseL[i] *= overL;
            noiseR[i] *= overR;

            dataoutL[i] = datainL[i] + noiseL[i];
            dataoutR[i] = datainR[i] + noiseR[i];
        }
    }

    void processMonoToMono(float *datain, float *dataout, float pitch)
    {
        float threshold = this->getFloatParam(fpThreshold);
        auto depth = this->getFloatParam(fpDepth);
        bool mode = this->getIntParam(ipMode) > 0;

        if (mode)
        {
            threshold *= 2.f;
            threshold -= 1.f;
            depth *= .5f;
        }

        float noise alignas(16)[VFXConfig::blockSize];
        basic_blocks::mechanics::clear_block<VFXConfig::blockSize>(noise);

        makeNoise(noise);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            noise[i] *= depth;

            auto env = mode ? datain[i] : fabsf(datain[i]);

            auto over = std::max(env - threshold, 0.f);

            noise[i] *= over;

            dataout[i] = datain[i] + noise[i];
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    float priorSlope = -1234.f;
    std::array<sst::filters::CytomicSVF, 11> filters;
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_NOISEAM_H
