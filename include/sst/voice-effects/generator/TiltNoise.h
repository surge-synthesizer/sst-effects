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

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct TiltNoise : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Tilt Noise"};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{1};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpLevel,
        fpTilt,
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

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        bool stereo = this->getIntParam(ipStereo);

        float atten = this->getFloatParam(fpTilt);
        if (atten > 0)
        {
            atten *= -4.f;
        }
        atten = this->dbToLinear(atten);
        float level = this->getFloatParam(fpLevel);
        level = level * level * level;

        setCoeffs();

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataoutL[i] = rng.unifPM1();
            dataoutR[i] = stereo ? rng.unifPM1() : dataoutL[i];

            dataoutL[i] *= level * atten;
            dataoutR[i] *= level * atten;
        }
        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL,
                                                                   dataoutR);
        }
    }

    void processMonoToMono(float *datain, float *dataout, float pitch)
    {
        float atten = this->getFloatParam(fpTilt);
        if (atten > 0)
        {
            atten *= -4.f;
        }
        atten = this->dbToLinear(atten);

        float level = this->getFloatParam(fpLevel);
        level = level * level * level;

        setCoeffs();

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataout[i] = rng.unifPM1();
            dataout[i] *= level * atten;
        }

        for (int i = 0; i < 11; ++i)
        {
            filters[i].template processBlock<VFXConfig::blockSize>(dataout, dataout);
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    float priorSlope = -1324.f;
    std::array<sst::filters::CytomicSVF, 11> filters;
};
} // namespace sst::voice_effects::generator
#endif // SCXT_TILTNOISE_H
