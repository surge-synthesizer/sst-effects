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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DYNAMICS_COMPRESSOR_H
#define INCLUDE_SST_VOICE_EFFECTS_DYNAMICS_COMPRESSOR_H

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FollowSlewAndSmooth.h"

// This compressor is based on the VCV module "pressor" by Bog audio.
// Many thanks to Matt Demanett for making it.

namespace sst::voice_effects::dynamics
{
template <typename VFXConfig> struct Compressor : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Compressor"};

    static constexpr size_t rmsBufferSize{1024}; // TODO: SR invariance...
    float *rmsBlock{nullptr};

    static constexpr int numFloatParams{7};
    static constexpr int numIntParams{1};

    enum FloatParams
    {
        fpThreshold,
        fpRatio,
        fpAttack,
        fpRelease,
        fpMakeUp,
        fpSideChainHP,
        fpSideChainLP
    };

    enum IntParams
    {
        ipDetector
        //        ipKnee,
    };

    Compressor() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        this->preReservePool(rmsBufferSize * sizeof(float));
    }

    ~Compressor()
    {
        if (rmsBlock)
        {
            VFXConfig::returnBlock(this, (uint8_t *)rmsBlock, rmsBufferSize * sizeof(float));
            rmsBlock = nullptr;
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpThreshold:
            return pmd()
                .asFloat()
                .withRange(-48.f, 0.f)
                .withDefault(0.f)
                .withDecimalPlaces(2)
                .withLinearScaleFormatting("dB")
                .withName("Threshold");
        case fpRatio:
            return pmd()
                .asFloat()
                .withRange(1.f, 12.f)
                .withDefault(2.f)
                .withLinearScaleFormatting("one over")
                .withDecimalPlaces(2)
                .withName("Ratio");
        case fpAttack:
            return pmd()
                .asFloat()
                .withRange(0.000001f, .1f)
                .withDefault(0.01f)
                .withLinearScaleFormatting("ms", 1000.f)
                .withName("Attack");
        case fpRelease:
            return pmd()
                .asFloat()
                .withRange(.000001f, .5f)
                .withDefault(0.2f)
                .withLinearScaleFormatting("ms", 1000.f)
                .withName("Release");
        case fpMakeUp:
            return pmd()
                .asFloat()
                .withRange(1.f, 4.f)
                .withDefault(1.f)
                .withLinearScaleFormatting("x")
                .withName("Makeup Gain");
        case fpSideChainHP:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48.f, 48.f)
                    .withName("SC HP Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asFloat()
                .withRange(-96.f, 0.f)
                .withDefault(-96.f)
                .withSemitoneZeroAt400Formatting()
                .withName("Sidechain HP");
        case fpSideChainLP:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(0.f, 96.f)
                    .withName("SC LP Offset")
                    .withDefault(72)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asFloat()
                .withRange(0.f, 72.f)
                .withDefault(72.f)
                .withSemitoneZeroAt400Formatting()
                .withName("Sidechain LP");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        //        switch (idx)
        //        {
        //        case ipKnee:
        //            return pmd()
        //                .asBool()
        //                .withUnorderedMapFormatting({
        //                    {false, "hard"},
        //                    {true, "soft"},
        //                })
        //                .withDefault(false)
        //                .withName("Knee");
        //        case ipDetector:
        //            return pmd()
        //                .asBool()
        //                .withUnorderedMapFormatting({
        //                    {false, "Peak"},
        //                    {true, "RMS"},
        //                })
        //                .withDefault(false)
        //                .withName("Detector");
        //        }

        return pmd()
            .asBool()
            .withUnorderedMapFormatting({
                {false, "Peak"},
                {true, "RMS"},
            })
            .withDefault(false)
            .withName("Detector");
    }

    void initVoiceEffect()
    {
        if (!rmsBlock)
        {
            auto block = VFXConfig::checkoutBlock(this, rmsBufferSize * sizeof(float));
            memset(block, 0, rmsBufferSize * sizeof(float));
            rmsBlock = (float *)block;
            RA.setStorage(rmsBlock, rmsBufferSize);
        }
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    static float decibelsToAmplitude(float db) { return powf(10.0f, db * 0.05f); }

    static float amplitudeToDecibels(float amplitude)
    {
        if (amplitude < 0.000001f)
        {
            return -120.0f;
        }
        return 20.0f * log10f(amplitude);
    }

    struct BallisticCoeffs
    {
        float a1{};
        float b0{};
    };

    static BallisticCoeffs computeBallisticCoeffs(float time_seconds, float T)
    {
        BallisticCoeffs coeffs{};
        coeffs.a1 = std::exp(-T / time_seconds);
        coeffs.b0 = 1.0f - coeffs.a1;
        return coeffs;
    }

    static float setBallistics(float abs_x, float &z, BallisticCoeffs attack_coeffs,
                               BallisticCoeffs release_coeffs)
    {
        const auto b0 = abs_x > z ? attack_coeffs.b0 : release_coeffs.b0;
        z += b0 * (abs_x - z);
        return z;
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto gain = this->getFloatParam(fpMakeUp);
        //        bool knee = this->getIntParam(ipKnee);
        bool RMS = this->getIntParam(ipDetector);
        auto threshold_db = this->getFloatParam(fpThreshold);
        auto ratio_recip = 1 / this->getFloatParam(fpRatio);

        const auto T = this->getSampleRateInv();
        auto attack_coeffs = computeBallisticCoeffs(this->getFloatParam(fpAttack), T);
        auto release_coeffs = computeBallisticCoeffs(this->getFloatParam(fpRelease), T);

        if (first)
        {
            lastEnv = 0.f;
            RA.reset();
            first = false;
        }

        setCoeffsHighpass(pitch);
        setCoeffsLowpass(pitch);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto outputL = datainL[i];
            auto outputR = datainR[i];

            float sidechain = (outputL + outputR) / 2;
            filters[0].processBlockStep(sidechain);
            filters[1].processBlockStep(sidechain);
            float env = fabsf(sidechain);

            if (RMS)
            {
                env = RA.step(env);
            }
            env = setBallistics(env, lastEnv, attack_coeffs, release_coeffs);
            env = amplitudeToDecibels(env);

            auto over = env - threshold_db;
            float reductionFactorDB = 0.0f;
            if (env > threshold_db)
            {
                reductionFactorDB = threshold_db + over * ratio_recip - env;
            }
            float reductionFactor = decibelsToAmplitude(reductionFactorDB);
            outputL *= reductionFactor;
            outputR *= reductionFactor;

            dataoutL[i] = outputL * gain;
            dataoutR[i] = outputR * gain;
        }
    }

    void processMonoToMono(float *datain, float *dataout, float pitch)
    {
        auto gain = this->getFloatParam(fpMakeUp);
        //        bool knee = this->getIntParam(ipKnee);
        bool RMS = this->getIntParam(ipDetector);
        auto threshold_db = this->getFloatParam(fpThreshold);
        auto ratio_recip = 1 / this->getFloatParam(fpRatio);

        const auto T = this->getSampleRateInv();
        auto attack_coeffs = computeBallisticCoeffs(this->getFloatParam(fpAttack), T);
        auto release_coeffs = computeBallisticCoeffs(this->getFloatParam(fpRelease), T);

        if (first)
        {
            lastEnv = 0.f;
            first = false;
        }

        setCoeffsHighpass(pitch);
        setCoeffsLowpass(pitch);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            float output = datain[i];

            float sidechain = output;
            filters[0].processBlockStep(sidechain);
            filters[1].processBlockStep(sidechain);
            float env = fabsf(sidechain);

            if (RMS)
            {
                env = RA.step(env);
            }
            env = setBallistics(env, lastEnv, attack_coeffs, release_coeffs);
            env = amplitudeToDecibels(env);

            auto over = env - threshold_db;
            float reductionFactorDB = 0.0f;
            if (env > threshold_db)
            {
                reductionFactorDB = threshold_db + over * ratio_recip - env;
            }
            float reductionFactor = decibelsToAmplitude(reductionFactorDB);

            output *= reductionFactor;

            dataout[i] = output * gain;
        }
    }

    void setCoeffsHighpass(float pitch)
    {
        auto hpParam = this->getFloatParam(fpSideChainHP);
        auto hpFreq =
            440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + hpParam : hpParam);

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

    void setCoeffsLowpass(float pitch)
    {
        auto lpParam = this->getFloatParam(fpSideChainLP);
        auto lpFreq =
            440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + lpParam : lpParam);

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

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    bool first = true;
    bool keytrackOn = false;
    float lastEnv = -1.f;
    sst::basic_blocks::dsp::RunningAverage RA;

    float hpFreqPrior = -1.f;
    float lpFreqPrior = -1.f;
    std::array<sst::filters::CytomicSVF, 2> filters;
};
} // namespace sst::voice_effects::dynamics
#endif // SCXT_COMPRESSOR_H
