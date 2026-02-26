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

#ifndef INCLUDE_SST_VOICE_EFFECTS_EQ_EQNBANDPARAMETRIC_H
#define INCLUDE_SST_VOICE_EFFECTS_EQ_EQNBANDPARAMETRIC_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::eq
{
template <typename VFXConfig, int NBands>
struct EqNBandParametric : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"EQNBand"};
    static constexpr const char *streamingNameByBand()
    {
        static_assert(NBands >= 1 && NBands <= 3);
        switch (NBands)
        {
        case 1:
            return "eq-parm-1band";
        case 2:
            return "eq-parm-2band";
        case 3:
            return "eq-parm-3band";
        }
        return "";
    }
    static constexpr const char *streamingName{streamingNameByBand()};

    static constexpr int numFloatParams{3 * NBands};
    static constexpr int numIntParams{0};

    EqNBandParametric() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
        std::fill(mParametric.begin(), mParametric.end(), this);
    }

    ~EqNBandParametric() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        int type = idx % 3;
        int which = idx / 3;

        switch (type)
        {
        case 0:
            return pmd()
                .asDecibelNarrow()
                .withName("Gain " + std::to_string(which + 1))
                .withDefault(0);
        case 1:
            return pmd()
                .asAudibleFrequency()
                .withDefault(1.f)
                .withName("Freq " + std::to_string(which + 1))
                .withDefault(-18 + which * 18);
        case 2:
            return pmd()
                .asFloat()
                .withName("BW " + std::to_string(which + 1))
                .withDefault(1)
                .withRange(.0833333, 5)
                .withLinearScaleFormatting("octaves");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        calc_coeffs();
        auto *inL = datainL;
        auto *inR = datainR;
        for (int i = 0; i < NBands; ++i)
        {
            mParametric[i].process_block_to(inL, inR, dataoutL, dataoutR);
            inL = dataoutL;
            inR = dataoutR;
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();
        auto *inL = datainL;
        for (int i = 0; i < NBands; ++i)
        {
            mParametric[i].process_block_to(inL, dataoutL);
            inL = dataoutL;
        }
    }

    float calc_GB(float x) { return this->dbToLinear(x * .5f); }

    void calc_coeffs()
    {
        std::array<float, NBands * 3> param;
        bool diff{false};
        for (int i = 0; i < NBands * 3; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }

        if (diff)
        {
            for (int i = 0; i < NBands; ++i)
            {
                auto bs = i * 3;
                mParametric[i].coeff_orfanidisEQ(mParametric[i].calc_omega(param[1 + bs] / 12.f),
                                                 param[2 + bs], this->dbToLinear(param[0 + bs]),
                                                 calc_GB(param[0 + bs]), 1);
            }
            mLastParam = param;
        }
    }

    float getFrequencyGraph(float f)
    {
        auto res = 1.f;
        for (int i = 0; i < NBands; ++i)
        {
            res *= mParametric[i].plot_magnitude(f);
        }
        return res;
    }

    float getBandFrequencyGraph(int band, float f) { return mParametric[band].plot_magnitude(f); }

  protected:
    std::array<float, NBands * 3> mLastParam{};
    std::array<typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType, NBands>
        mParametric;

  public:
    static constexpr int16_t streamingVersion{2};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom < streamingVersion);
        if (streamedFrom < 2)
        {
            for (int i = 0; i < NBands; ++i)
            {
                if (iparam[i] == 1)
                {
                    fparam[i * 3 + 2] /= 3.f;
                }
            }
        }
    }
};

} // namespace sst::voice_effects::eq

#endif // SHORTCIRCUITXT_EqNBandParametric_H
