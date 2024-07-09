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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITIES_NOISEAM_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITIES_NOISEAM_H

#include "../VoiceEffectCore.h"

#include <iostream>
#include <random>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/CorrelatedNoise.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct NoiseAM : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Noise AM"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{2};

    enum FloatParams
    {
        fpThreshold,
        fpDepth,
        fpHighpass,
        fpLowpass
    };

    enum IntParams
    {
        ipMode,
        ipStereo
    };

    NoiseAM()
        : core::VoiceEffectTemplateBase<VFXConfig>(), mGenerator((size_t)this), mDistro(-1.f, 1.f)
    {
        for (int i = 0; i < 7; ++i)
        {
            sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0][0], mPrior[0][1], 0, mDistro(mGenerator));
            sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[1][0], mPrior[1][1], 0, mDistro(mGenerator));
        }
    }

    ~NoiseAM() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool mode = this->getIntParam(ipMode);

        switch (idx)
        {
        case fpThreshold:
            return pmd().asPercent().withName("Threshold");
        case fpDepth:
            return pmd().asPercent().withDefault(1.f).withName("Depth");
        case fpHighpass:
            return pmd().asAudibleFrequency().withName("HP Frequency");
        case fpLowpass:
            return pmd().asAudibleFrequency().withName("LP Frequency");
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

    void setCoeffsHighpass()
    {
        auto hpFreq = 440.f * this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpHighpass));

        if (hpFreq != hpFreqPrior)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::HP, hpFreq, 0.5f, VFXConfig::getSampleRateInv(this), 0.f);
            hpFreqPrior = hpFreq;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    void setCoeffsLowpass()
    {
        auto lpFreq = 440.f * this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpLowpass));

        if (lpFreq != lpFreqPrior)
        {
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::LP, lpFreq, 0.5f, VFXConfig::getSampleRateInv(this), 0.f);
            lpFreqPrior = lpFreq;
        }
        else
        {
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        float threshold = this->getFloatParam(fpThreshold);
        auto depth = this->getFloatParam(fpDepth);
        bool mode = this->getIntParam(ipMode) > 0;

        if (mode)
        {
            threshold *= 2.f;
            threshold -= 1.f;
        }

        setCoeffsHighpass();
        setCoeffsLowpass();

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto noiseL = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[0][0], mPrior[0][1], 0, mDistro(mGenerator));
            auto noiseR = sst::basic_blocks::dsp::correlated_noise_o2mk2_supplied_value(
                mPrior[1][0], mPrior[1][1], 0, mDistro(mGenerator));

            filters[0].processBlockStep(noiseL, noiseR);
            filters[1].processBlockStep(noiseL, noiseR);

            noiseL *= depth;
            noiseR *= depth;

            auto envL = mode ? datainL[i] : std::fabsf(datainL[i]);
            auto envR = mode ? datainR[i] : std::fabsf(datainR[i]);

            auto overL = std::min(threshold - envL, 0.f);
            auto overR = std::min(threshold - envR, 0.f);

            noiseL *= overL;
            noiseR *= overR;

            dataoutL[i] = datainL[i] + noiseL;
            dataoutR[i] = datainR[i] + noiseR;
        }
    }

    //    void processMonoToMono(float *datain, float *dataout, float pitch)
    //    {
    //        for (int i = 0; i < VFXConfig::blockSize; i++)
    //        {
    //            auto input = datain[i];
    //
    //            dataout[i] = input;
    //        }
    //    }
    //
    //    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    //    {
    //
    //        for (int i = 0; i < VFXConfig::blockSize; i++)
    //        {
    //            auto inL = datainL[i];
    //            auto inR = datainL[i];
    //
    //            dataoutL[i] = inL;
    //            dataoutR[i] = inR;
    //        }
    //    }
    //
    //    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    float mPrior[2][2]{{0.f, 0.f}, {0.f, 0.f}};

    std::minstd_rand mGenerator;
    std::uniform_real_distribution<float> mDistro;

    float hpFreqPrior = -1.f;
    float lpFreqPrior = -1.f;
    std::array<sst::filters::CytomicSVF, 2> filters;
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_NOISEAM_H
