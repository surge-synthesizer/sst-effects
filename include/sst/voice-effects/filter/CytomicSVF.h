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

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct CytomicSVF : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Fast SVF"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{3};

    CytomicSVF() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~CytomicSVF() {}

    enum floatParams
    {
        fpFreqL,
        fpFreqR,
        fpResonance,
        fpShelf,
    };

    enum intParams
    {
        ipMode,
        ipStereo,
        ipSlope
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        bool gain{false};
        using md = sst::filters::CytomicSVF::Mode;
        auto mode = (md)this->getIntParam(ipMode);
        if (mode == md::BELL || mode == md::HIGH_SHELF || mode == md::LOW_SHELF)
        {
            gain = true;
        }

        switch (idx)
        {
        case 0:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(std::string("Offset") + (stereo ? " L" : ""))
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(std::string("Cutoff") + (stereo ? " L" : ""))
                .withDefault(0);

        case 1:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(!stereo ? std::string() : "Offset R")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(!stereo ? std::string() : std::string("Cutoff R"))
                .withDefault(0);

        case 2:
            return pmd()
                .asPercent()
                .withName("Resonance")
                .withLinearScaleFormatting("")
                .withDefault(0.707);
        case 3:
            return pmd()
                .asDecibelNarrow()
                .withRange(-12, 12)
                .withName(!gain ? std::string() : "Gain")
                .withDefault(0);
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        using md = sst::filters::CytomicSVF::Mode;
        switch (idx)
        {
        case ipMode:
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
        case ipStereo:
            return pmd().asBool().withDefault(false).withName("Stereo");
        case ipSlope:
            return pmd()
                .asBool()
                .withDefault(false)
                .withUnorderedMapFormatting({
                    {false, "12 dB/oct"},
                    {true, "24 dB/oct"},
                })
                .withName("Slope");
        }
        return pmd().withName("error");
    }

    void initVoiceEffect()
    {
        cySvf[0].init();
        cySvf[1].init();
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void calc_coeffs(float pitch = 0.f)
    {
        std::array<float, numFloatParams> param;
        std::array<int, numIntParams> iparam;
        bool diff{false}, idiff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            if (i < 2 && keytrackOn)
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
            auto mode = (sst::filters::CytomicSVF::Mode)(iparam[0]);

            auto res = std::clamp(param[2], 0.f, 1.f);
            if (iparam[2] == true)
            {
                res *= .885f; // I just checked peak heights on a spectrum analyzer.
            }

            auto shelf = this->dbToLinear(param[3]);

            if (this->getIntParam(ipStereo))
            {
                auto freqL = 440.0 * this->note_to_pitch_ignoring_tuning(param[0]);
                auto freqR = 440.0 * this->note_to_pitch_ignoring_tuning(param[1]);
                cySvf[0].template setCoeffForBlock<VFXConfig::blockSize>(
                    mode, freqL, freqR, res, res, this->getSampleRateInv(), shelf, shelf);
                if (iparam[2])
                {
                    cySvf[1].fetchCoeffs(cySvf[0]);
                }
            }
            else
            {
                auto freq = 440.0 * this->note_to_pitch_ignoring_tuning(param[0]);
                cySvf[0].template setCoeffForBlock<VFXConfig::blockSize>(
                    mode, freq, res, this->getSampleRateInv(), shelf);
                if (iparam[2])
                {
                    cySvf[1].fetchCoeffs(cySvf[0]);
                }
            }

            mLastParam = param;
            mLastIParam = iparam;
        }
        else
        {
            cySvf[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            if (iparam[2])
            {
                cySvf[1].template retainCoeffForBlock<VFXConfig::blockSize>();
            }
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        calc_coeffs(pitch);
        cySvf[0].template processBlock<VFXConfig::blockSize>(datainL, datainR, dataoutL, dataoutR);
        if (this->getIntParam(ipSlope) == true)
        {
            cySvf[1].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL,
                                                                 dataoutR);
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        calc_coeffs(pitch);
        cySvf[0].template processBlock<VFXConfig::blockSize>(datainL, dataoutL);
        if (this->getIntParam(ipSlope))
        {
            cySvf[1].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutL);
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        calc_coeffs(pitch);
        cySvf[0].template processBlock<VFXConfig::blockSize>(datainL, datainL, dataoutL, dataoutR);
        if (this->getIntParam(ipSlope))
        {
            cySvf[1].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutL, dataoutL,
                                                                 dataoutR);
        }
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool checkParameterConsistency() const { return true; }

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
    std::array<sst::filters::CytomicSVF, 2> cySvf;

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
