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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_SSTFILTERS_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_SSTFILTERS_H

#include "sst/basic-blocks/params/ParamMetadata.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/filters.h"
#include "sst/filters/FilterConfiguration.h"
#include "sst/filters/FilterConfigurationLabels.h"

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct SSTFilters : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Surge Synth Team Filters"};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{2};

    static constexpr size_t combBlockSize{
        sizeof(float) *
        (sst::filters::utilities::MAX_FB_COMB + sst::filters::utilities::SincTable::FIRipol_N)};

    SSTFilters() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
        this->preReservePool(combBlockSize);
        memset(&qfus, 0, sizeof(sst::filters::QuadFilterUnitState));
    }

    ~SSTFilters()
    {
        for (int i = 0; i < 4; ++i)
        {
            if (qfus.DB[i])
            {
                VFXConfig::returnBlock(this, (uint8_t *)(&qfus.DB[i][0]), combBlockSize);
            }
        }
    }

    static constexpr int fpCutoff{0};
    static constexpr int fpResonance{1};

    static constexpr int ipType{0};
    static constexpr int ipSubType{1};

    sst::filters::QuadFilterUnitState qfus;
    sst::filters::FilterCoefficientMaker<> coefMaker;
    sst::filters::FilterUnitQFPtr filterUnitPtr{nullptr};

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpCutoff:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName("Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Cutoff").withDefault(0);

        case fpResonance:
            return pmd()
                .asPercent()
                .withDefault(0.7f)
                .withName("Resonance")
                .withLinearScaleFormatting("");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        if (idx == ipType)
        {
            auto res = pmd()
                           .asInt()
                           .withRange(0, sst::filters::FilterType::num_filter_types - 1)
                           .withName("Type");
            std::unordered_map<int, std::string> nameMap;
            for (int i = 0; i < sst::filters::FilterType::num_filter_types; ++i)
            {
                nameMap.emplace(i, sst::filters::filter_type_names[i]);
            }
            return res.withUnorderedMapFormatting(nameMap);
        }

        if (idx == ipSubType)
        {
            auto ft = this->getIntParam(ipType);
            auto nft = sst::filters::fut_subcount[ft];
            if (nft != 0)
            {
                auto res = pmd().asInt().withRange(0, nft - 1).withName("SubType");

                std::unordered_map<int, std::string> nameMap;
                for (int i = 0; i < nft; ++i)
                {
                    nameMap.emplace(i, sst::filters::subtypeLabel(ft, i));
                }
                return res.withUnorderedMapFormatting(nameMap);
            }
            else
            {
                return pmd()
                    .asInt()
                    .withRange(0, 0)
                    .withName("SubType")
                    .withDefault(0)
                    .withDimensionlessFormatting();
            }
        }

        return pmd();
    }

    void initVoiceEffect() { resetFilter(false); }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void resetFilter(bool stereo)
    {
        std::fill(qfus.R, &qfus.R[sst::filters::n_filter_registers], SIMD_MM(setzero_ps)());
        std::fill(qfus.C, &qfus.C[sst::filters::n_cm_coeffs], SIMD_MM(setzero_ps)());

        auto type = (sst::filters::FilterType)this->getIntParam(ipType);

        for (int i = 0; i < 4; ++i)
        {
            qfus.active[i] = i < (stereo ? 2 : 1) ? 0xffffffff : 0;
            qfus.WP[i] = 0;
            if (type == filters::fut_comb_neg || type == filters::fut_comb_pos)
            {
                if (i < (stereo ? 2 : 1))
                {
                    if (!qfus.DB[i])
                    {
                        qfus.DB[i] = (float *)VFXConfig::checkoutBlock(this, combBlockSize);
                    }
                    memset(&(qfus.DB[i][0]), 0, combBlockSize);
                }
                else
                {
                    qfus.DB[i] = nullptr;
                }
            }
            else
            {
                if (qfus.DB[i])
                {
                    VFXConfig::returnBlock(this, (uint8_t *)(&qfus.DB[i][0]), combBlockSize);
                }
                qfus.DB[i] = nullptr;
            }
        }

        filterUnitPtr = sst::filters::GetQFPtrFilterUnit(
            type, (sst::filters::FilterSubType)this->getIntParam(ipSubType));

        coefMaker.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
    }

    void updateCoefficients(float pitch, bool stereo)
    {
        std::array<float, numFloatParams> param;
        std::array<int, numIntParams> iparam;
        bool diff{false}, idiff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            if (i == 0 && keytrackOn)
            {
                param[i] += pitch;
            }
            diff = diff || (mLastParam[i] != param[i]);
        }
        for (int i = 0; i < numIntParams; ++i)
        {
            iparam[i] = this->getIntParam(i);
            idiff = idiff || (mLastIParam[i] != iparam[i]);
        }
        idiff = idiff || (stereo != lastBuildWasStereo);
        lastBuildWasStereo = stereo;

        if (idiff)
        {
            resetFilter(stereo);
        }

        for (int f = 0; f < sst::filters::n_cm_coeffs; ++f)
        {
            float res alignas(16)[4];
            SIMD_MM(store_ps)(res, qfus.C[f]);
            coefMaker.C[f] = res[0];
        }

        coefMaker.MakeCoeffs(
            param[0], param[1], (sst::filters::FilterType)this->getIntParam(ipType),
            (sst::filters::FilterSubType)this->getIntParam(ipSubType), nullptr, false);

        coefMaker.updateState(qfus);

        mLastIParam = iparam;
        mLastParam = param;
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        updateCoefficients(pitch, true);
        if (filterUnitPtr)
        {
            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                float r alignas(16)[4];
                r[0] = datainL[i];
                r[1] = datainR[i];
                r[2] = 0.f;
                r[3] = 0.f;

                auto resS = filterUnitPtr(&qfus, SIMD_MM(load_ps)(r));
                float res alignas(16)[4];
                SIMD_MM(store_ps)(res, resS);
                dataoutL[i] = res[0];
                dataoutR[i] = res[1];
            }
        }
        else
        {
            sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        updateCoefficients(pitch, false);

        if (filterUnitPtr)
        {
            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                float r alignas(16)[4];
                r[0] = datainL[i];
                r[1] = 0.f;
                r[2] = 0.f;
                r[3] = 0.f;

                auto resS = filterUnitPtr(&qfus, SIMD_MM(load_ps)(r));
                float res alignas(16)[4];
                SIMD_MM(store_ps)(res, resS);
                dataoutL[i] = res[0];
            }
        }
        else
        {
            sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        }
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

    bool checkParameterConsistency()
    {
        auto ft = this->getIntParam(ipType);
        auto st = this->getIntParam(ipSubType);

        if (st >= sst::filters::fut_subcount[ft])
        {
            this->setIntParam(ipSubType, 0);
        }
        return true;
    }

  protected:
    bool keytrackOn{false}, wasKeytrackOn{false}, lastBuildWasStereo{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};

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

} // namespace sst::voice_effects::filter
#endif // SHORTCIRCUITXT_CYTOMICSVF_H
