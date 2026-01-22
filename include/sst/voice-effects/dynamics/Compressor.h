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
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FollowSlewAndSmooth.h"
#include "sst/filters/CytomicTilt.h"

// This compressor is based on the VCV module "pressor" by Bog audio (thx Matt!),
// and a ballistics calculation by Jatin Chowdhury (thx Jatin)

namespace sst::voice_effects::dynamics
{
template <typename VFXConfig> struct Compressor : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"Compressor"};
    static constexpr const char *streamingName{"compressor"};

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
        fpSCTiltAmt,
        fpSCTiltFreq,
    };

    enum IntParams
    {
        ipDetector
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
                .withName("Threshold")
                .withMultiplicativeModulationOffByDefault();
        case fpRatio:
            return pmd()
                .asFloat()
                .withRange(1.f, 12.f)
                .withDefault(2.f)
                .withLinearScaleFormatting(": 1")
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
            return pmd().asDecibelWithRange(0, 24).withName("Makeup Gain");
        case fpSCTiltAmt:
            return pmd()
                .asFloat()
                .withRange(-18, 18)
                .withDefault(0)
                .withLinearScaleFormatting("db")
                .withName("SC Tilt");
        case fpSCTiltFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48.f, 48.f)
                    .withName("Tilt Freq")
                    .withDefault(0)
                    .withSemitoneFormatting();
            }
            return pmd().asAudibleFrequency().withName("Tilt Freq");
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
    void initVoiceEffectPitch(float pitch)
    {
        float freq = 440 * this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpSCTiltFreq) +
                                                               pitch * keytrackOn);
        tilter.setCoeff(freq, .07f, this->getSampleRateInv());
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

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        auto makeup = decibelsToAmplitude(this->getFloatParam(fpMakeUp));
        gainLerp.set_target(makeup);

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

        setTiltCoeffs(pitch);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto outputL = datainL[i];
            auto outputR = datainR[i];

            float sidechain = (outputL + outputR) / 2;
            tilter.processBlockStep(sidechain);
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

            dataoutL[i] = outputL;
            dataoutR[i] = outputR;
        }
        gainLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        auto makeup = decibelsToAmplitude(this->getFloatParam(fpMakeUp));
        gainLerp.set_target(makeup);

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

        setTiltCoeffs(pitch);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            float output = datain[i];

            float sidechain = output;
            tilter.processBlockStep(sidechain);
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

            dataout[i] = output;
        }
        gainLerp.multiply_block(dataout);
    }

    void setTiltCoeffs(float pitch)
    {
        auto freqParam = this->getFloatParam(fpSCTiltFreq) + pitch * keytrackOn;
        float freq = 440 * this->note_to_pitch_ignoring_tuning(freqParam);
        float slope = this->dbToLinear(this->getFloatParam(fpSCTiltAmt) / 2);

        tilter.template setCoeffForBlock<VFXConfig::blockSize>(freq, .07f, this->getSampleRateInv(),
                                                               slope);
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

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> gainLerp;
    sst::filters::CytomicTilt tilter;

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
} // namespace sst::voice_effects::dynamics
#endif // SCXT_COMPRESSOR_H
