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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_SLEWER_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_SLEWER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

namespace sst::voice_effects::distortion
{
template <typename VFXConfig> struct Slewer : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Slewer"};

    enum FloatParamIdx
    {
        fpPreGain,
        fpPreFreq,
        fpPreBW,
        fpSlewRate,
        fpPostGain,
        fpPostFreq,
        fpPostBW
    };

    static constexpr int numFloatParams{7};
    static constexpr int numIntParams{0};

    Slewer() : core::VoiceEffectTemplateBase<VFXConfig>(), bq{this, this} {}

    ~Slewer() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((FloatParamIdx)idx)
        {
        case fpPreGain:
            return pmd().asDecibelNarrow().withDefault(0).withName("Pre Gain");
        case fpPreFreq:
            return pmd().asAudibleFrequency().withDefault(0).withName("Pre Freq");
        case fpPreBW:
            return pmd()
                .asFloat()
                .withRange(0, 2)
                .withDefault(1)
                .withName("Pre BW")
                .withLinearScaleFormatting("octaves");

        case fpSlewRate:
            return pmd().asAudibleFrequency().withDefault(0).withName("Slew Rate");

        case fpPostGain:
            return pmd().asDecibelNarrow().withDefault(0).withName("Post Gain");
        case fpPostFreq:
            return pmd().asAudibleFrequency().withDefault(0).withName("Post Freq");
        case fpPostBW:
            return pmd()
                .asFloat()
                .withRange(0, 2)
                .withDefault(1)
                .withName("Post BW")
                .withLinearScaleFormatting("octaves");
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    void initVoiceEffect()
    {
        calc_coeffs();
        lipolRate.instantize();
        v[0] = 0.f;
        v[1] = 0.f;
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void calc_coeffs()
    {
        lipolRate.set_target(this->getSampleRateInv() * 440.0 *
                             this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpSlewRate)));
        std::array<float, numFloatParams> param;
        bool diff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }

        if (diff)
        {
            bq[0].coeff_orfanidisEQ(
                bq[0].calc_omega(param[fpPreFreq] / 12.f), std::max(param[fpPreBW], 0.001f),
                this->dbToLinear(param[fpPreGain]), this->dbToLinear(param[fpPreGain] * 0.5), 1);
            bq[1].coeff_orfanidisEQ(
                bq[1].calc_omega(param[fpPostFreq] / 12.f), std::max(param[fpPostBW], 0.001f),
                this->dbToLinear(param[fpPostGain]), this->dbToLinear(param[fpPostGain] * 0.5), 1);
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        calc_coeffs();
        float rate alignas(16)[VFXConfig::blockSize];
        lipolRate.store_block(rate);

        bq[0].process_block_to(datainL, datainR, dataoutL, dataoutR);

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            if (dataoutL[k] > v[0])
                v[0] = std::min(dataoutL[k], v[0] + rate[k]);
            else
                v[0] = std::max(dataoutL[k], v[0] - rate[k]);
            dataoutL[k] = v[0];

            if (dataoutR[k] > v[1])
                v[1] = std::min(dataoutR[k], v[1] + rate[k]);
            else
                v[1] = std::max(dataoutR[k], v[1] - rate[k]);
            dataoutR[k] = v[1];
        }

        bq[1].process_block_to(dataoutL, dataoutR, dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();
        float rate alignas(16)[VFXConfig::blockSize];
        lipolRate.store_block(rate);

        bq[0].process_block_to(datainL, dataoutL);

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            if (dataoutL[k] > v[0])
                v[0] = std::min(dataoutL[k], v[0] + rate[k]);
            else
                v[0] = std::max(dataoutL[k], v[0] - rate[k]);

            dataoutL[k] = v[0];
        }

        bq[1].process_block_to(dataoutL, dataoutL);
    }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<float, 2> v{0.f, 0.f};
    std::array<typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType, 2> bq;

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolRate;
};
} // namespace sst::voice_effects::distortion

#endif // SHORTCIRCUITXT_SLEWER_H
