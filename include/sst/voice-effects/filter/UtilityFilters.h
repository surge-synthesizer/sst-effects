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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITYFILTERS_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITYFILTERS_H

#include "../VoiceEffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct UtilityFilters : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Utility Filters"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpHPFreq,
        fpLPFreq,
        fpLSFreq,
        fpLSGain,
        fpHSFreq,
        fpHSGain
    };

    UtilityFilters() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(priorFP.begin(), priorFP.end(), -1000.f);
    }

    ~UtilityFilters() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpHPFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("LoCut Offset")
                    .withDefault(-48)
                    .deactivatable()
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withDefault(-60)
                .withName("LoCut Freq")
                .deactivatable();
        case fpLPFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName("HiCut Offset")
                    .withDefault(-48)
                    .deactivatable()
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withDefault(70)
                .withName("HiCut Freq")
                .deactivatable();
        case fpLSFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(0, 48)
                    .withName("LoSHelf Offset")
                    .withDefault(-48)
                    .deactivatable()
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withDefault(-60)
                .withName("LoShelf Freq")
                .deactivatable();
        case fpHSFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(0, 96)
                    .withName("HiShelf Offset")
                    .withDefault(-48)
                    .deactivatable()
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withDefault(70)
                .withName("HiShelf Freq")
                .deactivatable();
        case fpLSGain:
            return pmd().asDecibelWithRange(-12.f, 12.f).withName("LoShelf Gain").withDefault(0.f);
        case fpHSGain:
            return pmd().asDecibelWithRange(-12.f, 12.f).withName("LoShelf Gain").withDefault(0.f);
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        std::fill(priorFP.begin(), priorFP.end(), -1000.f);
        for (auto &f : CySVFs)
        {
            f.init();
        }
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void setCoeffs(float pitch)
    {
        bool ktChanged = keytrackOn != wasKeytrackOn;

        float hpFreq = this->getFloatParam(fpHPFreq);
        if (hpFreq != priorFP[0] || ktChanged)
        {
            priorFP[0] = hpFreq;
            hpFreq =
                440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + hpFreq : hpFreq);

            CySVFs[0].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Highpass, hpFreq, 0.5f,
                VFXConfig::getSampleRateInv(this), 0.f);
        }
        else
        {
            CySVFs[0].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        float lpFreq = this->getFloatParam(fpLPFreq);
        if (lpFreq != priorFP[1] || ktChanged)
        {
            priorFP[1] = lpFreq;
            lpFreq =
                440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + lpFreq : lpFreq);

            CySVFs[1].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::Lowpass, lpFreq, 0.5f, VFXConfig::getSampleRateInv(this),
                0.f);
        }
        else
        {
            CySVFs[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        float lsFreq = this->getFloatParam(fpLSFreq);
        float lsGain = this->getFloatParam(fpLSGain);
        if (lsFreq != priorFP[2] || lsGain != priorFP[4] || ktChanged)
        {
            priorFP[2] = lsFreq;
            priorFP[4] = lsGain;
            lsFreq =
                440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + lsFreq : lsFreq);
            lsGain = this->dbToLinear(lsGain);

            CySVFs[2].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::LowShelf, lsFreq, 0.5f,
                VFXConfig::getSampleRateInv(this), lsGain);
        }
        else
        {
            CySVFs[2].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        float hsFreq = this->getFloatParam(fpHSFreq);
        float hsGain = this->getFloatParam(fpHSGain);
        if (hsFreq != priorFP[3] || hsGain != priorFP[5] || ktChanged)
        {
            priorFP[3] = hsFreq;
            priorFP[5] = hsGain;

            hsFreq =
                440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + hsFreq : hsFreq);
            hsGain = this->dbToLinear(hsGain);

            CySVFs[3].template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::Mode::HighShelf, hsFreq, 0.5f,
                VFXConfig::getSampleRateInv(this), hsGain);
        }
        else
        {
            CySVFs[3].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        setCoeffs(pitch);

        float dataL alignas(16)[VFXConfig::blockSize];
        float dataR alignas(16)[VFXConfig::blockSize];
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataL);
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataR);

        int it{0};
        for (auto &f : CySVFs)
        {
            if (!this->getIsDeactivated(it))
            {
                f.template processBlock<VFXConfig::blockSize>(dataL, dataR, dataL, dataR);
            }
            ++it;
        }
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataL, dataoutL);
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataR, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        setCoeffs(pitch);

        float data alignas(16)[VFXConfig::blockSize];
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datain, data);

        int it{0};
        for (auto &f : CySVFs)
        {
            if (!this->getIsDeactivated(it))
            {
                f.template processBlock<VFXConfig::blockSize>(datain, dataout);
            }
            ++it;
        }
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(data, dataout);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{false}, wasKeytrackOn{false};
    std::array<float, numFloatParams> priorFP;

    std::array<sst::filters::CytomicSVF, 4> CySVFs;

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

#endif // UTILITYFILTERS_H
