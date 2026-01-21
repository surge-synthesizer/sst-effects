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

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct StaticPhaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"Static Phaser"};
    static constexpr const char *streamingName{"filt-statph"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    static constexpr int maxPhases{6};

    enum FloatParams
    {
        fpCenterFrequencyL,
        fpCenterFrequencyR,
        fpSpacing,
        fpResonance,
        fpFeedback
    };

    enum IntParams
    {
        ipStages,
        ipStereo
    };

    StaticPhaser() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~StaticPhaser() {}

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipStages:
            return pmd()
                .asInt()
                .withRange(1, maxPhases)
                .withName("Stages")
                .withLinearScaleFormatting("")
                .withDefault(4);
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case fpCenterFrequencyL:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(std::string("Offset L") + (stereo ? " L" : ""))
                    .withDefault(0)
                    .withSemitoneFormatting();
            }
            return pmd()
                .asAudibleFrequency()
                .withName(std::string("Frequency") + (stereo ? " L" : ""))
                .withDefault(0);
        case fpCenterFrequencyR:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(!stereo ? std::string() : "Offset R")
                    .withDefault(0)
                    .withSemitoneFormatting();
            }
            return pmd()
                .asAudibleFrequency()
                .withName(!stereo ? std::string() : "Frequency R")
                .withDefault(0);
        case fpSpacing:
            return pmd()
                .asFloat()
                .withRange(-96, 96)
                .withDefault(12)
                .withName("Spacing")
                .withSemitoneFormatting();
        case fpResonance:
            return pmd().asPercent().withDefault(0.5f).withName("Resonance");
        case fpFeedback:
            return pmd().asPercent().withDefault(0.f).withName("Feedback");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect()
    {
        lipolFb.set_target_instant(
            std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
        shelf.init();
        hpf.init();
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        calc_coeffs(pitch);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        lipolFb.set_target(std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
        float fb alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(fb);

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
            shelf.processBlockStep(fbAmt[0], fbAmt[1]);
            hpf.processBlockStep(fbAmt[0], fbAmt[1]);
            dataoutL[k] = dL;
            dataoutR[k] = dR;
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    void processMonoToMono(const float *const dataIn, float *dataOut, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        this->calc_coeffs(pitch);

        mech::copy_from_to<VFXConfig::blockSize>(dataIn, dataOut);

        lipolFb.set_target(std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
        float fb alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(fb);

        for (int k = 0; k < VFXConfig::blockSize; ++k)
        {
            auto dL = dataOut[k] + fbAmt[0];
            for (int i = 0; i < this->getIntParam(ipStages); ++i)
            {
                float tmp{0};
                apfs[i].processBlockStep(dL, tmp);
            }
            fbAmt[0] = dL * fb[k];
            shelf.processBlockStep(fbAmt[0]);
            hpf.processBlockStep(fbAmt[0]);
            dataOut[k] = dL;
        }
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

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
            if (idiff)
            {
                for (int i = 0; i < iparam[ipStages]; ++i)
                {
                    apfs[i].init();
                }
            }

            auto freqL = 440.0 * this->note_to_pitch_ignoring_tuning(param[fpCenterFrequencyL]);
            auto freqR =
                iparam[ipStereo]
                    ? 440.0 * this->note_to_pitch_ignoring_tuning(param[fpCenterFrequencyR])
                    : freqL;

            auto freqInc = 0.0;
            if (iparam[ipStages] > 1)
            {
                freqInc = 440.0 * this->note_to_pitch_ignoring_tuning(param[fpSpacing] /
                                                                      (iparam[ipStages] - 1));
            }

            auto res = std::clamp(param[fpResonance], 0.f, 1.f);
            auto sg = 1 - param[fpFeedback] * 0.05;

            hpf.template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::Highpass, 35.f, .25f, this->getSampleRateInv());

            if (iparam[ipStereo])
            {
                for (int i = 0; i < iparam[ipStages]; ++i)
                {
                    apfs[i].template setCoeffForBlock<VFXConfig::blockSize>(
                        allpass, freqL, freqR, res, res, this->getSampleRateInv(), 1.f, 1.f);
                    freqL += freqInc;
                    freqR += freqInc;
                }
                shelf.template setCoeffForBlock<VFXConfig::blockSize>(
                    sst::filters::CytomicSVF::Mode::HighShelf, freqL, freqR, 0.f, 0.f,
                    this->getSampleRateInv(), sg, sg);
            }
            else
            {
                for (int i = 0; i < iparam[ipStages]; ++i)
                {
                    apfs[i].template setCoeffForBlock<VFXConfig::blockSize>(
                        allpass, freqL, res, this->getSampleRateInv());
                    freqL += freqInc;
                }
                shelf.template setCoeffForBlock<VFXConfig::blockSize>(
                    sst::filters::CytomicSVF::Mode::HighShelf, freqL, 0.f, this->getSampleRateInv(),
                    sg);
            }
            mLastParam = param;
            mLastIParam = iparam;
        }
        else
        {
            for (int i = 0; i < iparam[ipStages]; ++i)
                apfs[i].template retainCoeffForBlock<VFXConfig::blockSize>();
            shelf.retainCoeffForBlock<VFXConfig::blockSize>();
            hpf.retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }
    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool checkParameterConsistency() const { return true; }
    size_t silentSamplesLength() const { return 10; }

  protected:
    bool keytrackOn{false}, wasKeytrackOn{false};
    float fbAmt[2]{0.f, 0.f};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<sst::filters::CytomicSVF, maxPhases> apfs;
    sst::filters::CytomicSVF shelf;
    sst::filters::CytomicSVF hpf;
    const sst::filters::CytomicSVF::Mode allpass = sst::filters::CytomicSVF::Mode::Allpass;

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb;

  public:
    static constexpr int16_t streamingVersion{2};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        assert(streamedFrom <= streamingVersion);

        if (streamedFrom == 1)
        {
            auto stages = iparam[ipStages];
            if (stages > 1)
            {
                auto spread = fparam[fpSpacing];
                auto halfStage = stages * 0.5;
                fparam[0] -= halfStage * spread;
                fparam[1] -= halfStage * spread;

                fparam[fpSpacing] = std::clamp((spread * (stages - 1)), -96.f, 96.f);
            }
        }
    }
};
} // namespace sst::voice_effects::filter
#endif // INCLUDE_SST_VOICE_EFFECTS_FILTER_STATICPHASER_H
