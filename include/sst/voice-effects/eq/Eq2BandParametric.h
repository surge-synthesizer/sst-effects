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

#ifndef INCLUDE_SST_VOICE_EFFECTS_EQ_EQ2BANDPARAMETRIC_H
#define INCLUDE_SST_VOICE_EFFECTS_EQ_EQ2BANDPARAMETRIC_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::eq
{
template <typename VFXConfig> struct Eq2BandParametric : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Generate Sin"};

    enum struct Eq2BandParametricFloatParams : uint32_t
    {
        gain1,
        freq1,
        bw1,
        gain2,
        freq2,
        bw2,
        num_params
    };

    enum struct Eq2BandParametricIntParams : uint32_t
    {
        type1,
        type2,
        num_params
    };

    static constexpr int numFloatParams{(int)Eq2BandParametricFloatParams::num_params};
    static constexpr int numIntParams{(int)Eq2BandParametricIntParams::num_params};

    Eq2BandParametric() : core::VoiceEffectTemplateBase<VFXConfig>(), mParametdric{this, this} {}

    ~Eq2BandParametric() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)Eq2BandParametricFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((Eq2BandParametricFloatParams)idx)
        {
        case Eq2BandParametricFloatParams::gain1:
        case Eq2BandParametricFloatParams::gain2:
            return pmd().asDecibelNarrow().withName("Gain").withDefault(0);
        case Eq2BandParametricFloatParams::freq1:
            return pmd().asAudibleFrequency().withDefault(1.f).withName("Freq").withDefault(-12);
        case Eq2BandParametricFloatParams::freq2:
            return pmd().asAudibleFrequency().withDefault(1.f).withName("Freq").withDefault(24);
        case Eq2BandParametricFloatParams::bw1:
        case Eq2BandParametricFloatParams::bw2:
            return pmd()
                .asFloat()
                .withName("BW")
                .withDefault(1)
                .withRange(0, 5)
                .withLinearScaleFormatting("octaves");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd()
            .asInt()
            .withRange(0, 1)
            .withName("Type")
            .withUnorderedMapFormatting({{0, "A"}, {1, "B"}})
            .withDefault(0);
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        calc_coeffs();
        mParametdric[0].process_block_to(datainL, datainR, dataoutL, dataoutR);
        mParametdric[1].process_block_to(dataoutL, dataoutR, dataoutL, dataoutR);
    }
    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();
        mParametdric[0].process_block_to(datainL, dataoutL);
        mParametdric[1].process_block(dataoutL);
    }

    float calc_GB_type_B(bool b, float x)
    {
        if (b && (fabs(x) > 6))
        {
            if (x > 0)
                x = 3;
            else
                x = -3;
        }
        else
            x *= 0.5;

        return this->dbToLinear(x);
    }

    void calc_coeffs()
    {
        std::array<float, 6> param;
        std::array<int, 2> iparam;
        bool diff{false};
        for (int i = 0; i < 6; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }
        for (int i = 0; i < 2; ++i)
        {
            iparam[i] = this->getIntParam(i);
            diff = diff || (mLastIParam[i] != iparam[i]);
        }

        if (diff)
        {
            // FixMe - bandwidth units are wrong
            mParametdric[0].coeff_orfanidisEQ(mParametdric[0].calc_omega(param[1] / 12.f), param[2],
                                              this->dbToLinear(param[0]),
                                              calc_GB_type_B(iparam[0], param[0]), 1);
            mParametdric[1].coeff_orfanidisEQ(mParametdric[0].calc_omega(param[4] / 12.f), param[5],
                                              this->dbToLinear(param[3]),
                                              calc_GB_type_B(iparam[1], param[3]), 1);
            mLastParam = param;
            mLastIParam = iparam;
        }
    }

  protected:
    std::array<float, 6> mLastParam{};
    std::array<int, 2> mLastIParam{-100, -100};
    typename sst::filters::Biquad::BiquadFilter<Eq2BandParametric<VFXConfig>, VFXConfig::blockSize>
        mParametdric[2];
};
} // namespace sst::voice_effects::eq

#endif // SHORTCIRCUITXT_Eq2BandParametric_H
