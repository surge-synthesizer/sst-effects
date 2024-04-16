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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_STATICPHASER_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_STATICPHASER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/filters/CytomicSVF.h"

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct StaticPhaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Static Phaser"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{1};

    static constexpr int maxPhases{8};

    enum FloatParams
    {
        fpCenterFrequency,
        fpFrequencyWidth,
        fpResonance,
        fpFeedback
    };

    enum IntParams
    {
        ipStages
    };

    StaticPhaser() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~StaticPhaser() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < numFloatParams);
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpCenterFrequency:
            return pmd().asAudibleFrequency().withName("Center").withDefault(0);
        case fpFrequencyWidth:
            return pmd()
                .asFloat()
                .withRange(-48, 48)
                .withDefault(12)
                .withName("Spread")
                .withLinearScaleFormatting("semitones");
        case fpResonance:
            return pmd().asPercent().withDefault(0.707).withName("Stage Resonance");
        case fpFeedback:
            return pmd().asPercent().withDefault(0.f).withName("Feedback");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        assert(idx == ipStages);

        return pmd()
            .asInt()
            .withRange(1, maxPhases)
            .withName("Stages")
            .withLinearScaleFormatting("stages")
            .withDefault(4);
    }

    void initVoiceEffect()
    {
        lipolFb.set_target_instant(
            std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        calc_coeffs(pitch);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        lipolFb.set_target(std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
        float fb alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(fb);

        // lipol the feedback please
        for (int k = 0; k < VFXConfig::blockSize; ++k)
        {
            auto dL = dataoutL[k] + fbAmt[0];
            auto dR = dataoutR[k] + fbAmt[1];
            for (int i = 0; i < this->getIntParam(ipStages); ++i)
            {
                apfs[i].processBlockStep(dL, dR);
            }
            fbAmt[0] = dL * fb[k];
            fbAmt[1] = dR * fb[k];
            dataoutL[k] = dL;
            dataoutR[k] = dR;
        }
    }

    void calc_coeffs(float pitch = 0.f)
    {
        std::array<float, numFloatParams> param;
        std::array<int, numIntParams> iparam;
        bool diff{false}, idiff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }
        for (int i = 0; i < numIntParams; ++i)
        {
            iparam[i] = this->getIntParam(i);
            idiff = idiff || (mLastIParam[i] != iparam[i]);
        }

        if (diff || idiff)
        {
            if (idiff)
            {
                for (int i = 0; i < iparam[ipStages]; ++i)
                {
                    apfs[i].init();
                }
            }
            auto mode = sst::filters::CytomicSVF::Mode::ALL;
            auto spread{0.f};
            auto base{param[fpCenterFrequency]};
            if (iparam[ipStages] > 1)
            {
                spread = (param[fpFrequencyWidth] * 2) / (iparam[ipStages] - 1);
                base = param[fpCenterFrequency] - param[fpFrequencyWidth];
            }
            for (int i = 0; i < iparam[ipStages]; ++i)
            {
                auto freq = 440.0 * this->note_to_pitch_ignoring_tuning(base + spread * i);

                auto res = std::clamp(param[fpResonance], 0.f, 1.f);
                apfs[i].template setCoeffForBlock<VFXConfig::blockSize>(mode, freq, res,
                                                                        this->getSampleRateInv());
            }
            mLastParam = param;
            mLastIParam = iparam;
        }
        else
        {
            for (int i = 0; i < iparam[ipStages]; ++i)
                apfs[i].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

  protected:
    float fbAmt[2]{0.f, 0.f};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<sst::filters::CytomicSVF, maxPhases> apfs;

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb;
};

} // namespace sst::voice_effects::filter
#endif // SHORTCIRCUITXT_CYTOMICSVF_H
