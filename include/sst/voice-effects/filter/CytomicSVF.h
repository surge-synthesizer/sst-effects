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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_CYTOMICSVF_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_CYTOMICSVF_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/filters/CytomicSVF.h"

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct CytomicSVF : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Fast SVF"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{1};

    CytomicSVF() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~CytomicSVF() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case 0:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Cutoff").withDefault(0);

        case 1:
            return pmd()
                .asPercent()
                .withDefault(0.7f)
                .withName("Resonance")
                .withLinearScaleFormatting("");
        case 2:
            return pmd().asDecibelNarrow().withRange(-12, 12).withName("Shelf").withDefault(-6);

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        using md = sst::filters::CytomicSVF::Mode;

        return pmd()
            .asInt()
            .withRange(0, 8)
            .withName("Mode")
            .withUnorderedMapFormatting({
                {md::LP, "Low Pass"},
                {md::HP, "High Pass"},
                {md::BP, "Band Pass"},
                {md::NOTCH, "Notch"},
                {md::PEAK, "Peak"},
                {md::ALL, "All Pass"},
                {md::BELL, "Bell"},
                {md::LOW_SHELF, "Low Shelf"},
                {md::HIGH_SHELF, "High Shelf"},
            })
            .withDefault(md::LP);
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        calc_coeffs(pitch);
        cySvf.processBlock<VFXConfig::blockSize>(datainL, datainR, dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        calc_coeffs(pitch);
        cySvf.processBlock<VFXConfig::blockSize>(datainL, dataoutL);
    }

    void calc_coeffs(float pitch = 0.f)
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
        idiff |= (wasKeytrackOn != keytrackOn);
        wasKeytrackOn = keytrackOn;

        if (diff || idiff)
        {
            if (idiff)
            {
                cySvf.init();
            }
            auto mode = (sst::filters::CytomicSVF::Mode)(iparam[0]);
            auto freq = 440.0 * this->note_to_pitch_ignoring_tuning(param[0]);

            auto res = std::clamp(param[1], 0.f, 1.f);
            auto shelf = this->dbToLinear(param[2]);
            cySvf.setCoeffForBlock<VFXConfig::blockSize>(mode, freq, res, this->getSampleRateInv(),
                                                         shelf);

            mLastParam = param;
            mLastIParam = iparam;
        }
        else
        {
            cySvf.retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    bool keytrackOn{false}, wasKeytrackOn{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    sst::filters::CytomicSVF cySvf;
};

} // namespace sst::voice_effects::filter
#endif // SHORTCIRCUITXT_CYTOMICSVF_H
