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

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    enum FloatParams
    {
        fpThreshold,
        fpRatio,
        fpAttack,
        fpRelease,
        fpMakeUp,
//        fpSideChainHP,
//        fpSideChainLP
    };

    enum IntParams
    {
        ipKnee,
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
                .withRange(0.000001f, .3f)
                .withDefault(0.01f)
                .withLinearScaleFormatting("ms", 1000.f)
                .withName("Attack");
        case fpRelease:
                return pmd()
                .asFloat()
                .withRange(.01f, 1.f)
                .withDefault(0.2f)
                .withLinearScaleFormatting("ms", 1000.f)
                .withName("Release");
            case fpMakeUp:
                return pmd()
                .asFloat()
                .withRange(1.f,4.f)
                .withDefault(1.f)
                .withLinearScaleFormatting("x")
                .withName("Makeup Gain");
//        case fpSideChainHP:
//                return pmd()
//                .asFloat()
//                .withRange(-96.f, 0.f)
//                .withDefault(-96.f)
//                .withSemitoneZeroAt400Formatting()
//                .withName("Sidechain HP");
//        case fpSideChainLP:
//                return pmd()
//                .asFloat()
//                .withRange(0.f, 96.f)
//                .withDefault(96.f)
//                .withSemitoneZeroAt400Formatting()
//                .withName("Sidechain LP");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        
        switch(idx)
        {
            case ipKnee:
                return pmd()
                .asBool()
                .withUnorderedMapFormatting({
                    {false, "hard"},
                    {true, "soft"},
                })
                .withDefault(false)
                .withName("Knee");
            case ipDetector:
                return pmd()
                .asBool()
                .withUnorderedMapFormatting({
                    {false, "Peak"},
                    {true, "RMS"},
                })
                .withDefault(false)
                .withName("Detector");
                
        }
        
        return pmd()
        .asBool()
        .withUnorderedMapFormatting({
            {false, "hard"},
            {true, "soft"},
        })
        .withDefault(true)
        .withName("Knee");
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
    
    
    float decibelsToAmplitude(float db) {
        return powf(10.0f, db * 0.05f);
    }
    
    float amplitudeToDecibels(float amplitude)
    {
        if (amplitude < 0.000001f)
        {
            return -120.0f;
        }
        return 20.0f * log10f(amplitude);
    }
    
    
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto gain = this->getFloatParam(fpMakeUp);
        bool knee = this->getIntParam(ipKnee);
        bool RMS = this->getIntParam(ipDetector);
        auto threshold = this->getFloatParam(fpThreshold);
        auto ratio = 1 / this->getFloatParam(fpRatio);
        
        float sampleRate = this->getSampleRate();
        
        float attack = this->getFloatParam(fpAttack) * 1000.f;
        float release = this->getFloatParam(fpRelease) * 1000.f;
        attackSL.setParams(attack, 1.f, sampleRate);
        releaseSL.setParams(release, 1.f, sampleRate);
        
        if (first)
        {
            attackSL.reset();
            releaseSL.reset();
            RA.reset();
            lastEnv = 0.f;
            first = false;
        }
        
        auto outputL = 0.f;
        auto outputR = 0.f;
        auto outLevel = 0.f;
        
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            float env = datainL[i] + datainR[i];
            env = fabsf(env);
            
            if (RMS)
            {
                env = RA.step(env);
            }
        
            if (lastEnv > env)
            {
                env = releaseSL.step(env);
                attackSL.setLast(env);
            }
            else
            {
                env = attackSL.step(env);
                releaseSL.setLast(env);
            }
            lastEnv = env;
            
            env = amplitudeToDecibels(env);
            
            auto outputL = datainL[i];
            auto outputR = datainR[i];

            auto over = env - threshold;
            auto reductionFactor = (env < threshold) ? 1.f : 1.f - (env - (ratio * over + threshold));
            reductionFactor = decibelsToAmplitude(reductionFactor);
            outputL *= reductionFactor;
            outputR *= reductionFactor;
            
            dataoutL[i] = outputL * gain;
            dataoutR[i] = outputR * gain;
        }
    }
    
    void processMonoToMono(float *datain, float *dataout, float pitch)
    {
        auto gain = this->getFloatParam(fpMakeUp);
        bool knee = this->getIntParam(ipKnee);
        bool RMS = this->getIntParam(ipDetector);
        auto threshold = amplitudeToDecibels(this->getFloatParam(fpThreshold));
        auto ratio = 1 / this->getFloatParam(fpRatio);
        
        float sampleRate = this->getSampleRate();
        
        float attack = this->getFloatParam(fpAttack) * 1000.f;
        float release = this->getFloatParam(fpRelease) * 1000.f;
        attackSL.setParams(attack, 1.f, sampleRate);
        releaseSL.setParams(release, 1.f, sampleRate);
        
        if (first)
        {
            attackSL.reset();
            releaseSL.reset();
            RA.reset();
            lastEnv = 0.f;
            first = false;
        }
        
        auto output = 0.f;
        auto outLevel = 0.f;
        
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            float env = datain[i];
            env = fabsf(env);
            if (RMS)
            {
                env = RA.step(env);
            }
            
            if (env > lastEnv)
            {
                env = attackSL.step(env);
                releaseSL.setLast(env);
            }
            else
            {
                env = releaseSL.step(env);
                attackSL.setLast(env);
            }
            lastEnv = env;
            
            env = amplitudeToDecibels(env);
            
            output = datain[i];
            
            auto over = env - threshold;
            auto reductionFactor = (env < threshold) ? 1.f : 1.f - (env - (ratio * over + threshold));
            reductionFactor = decibelsToAmplitude(reductionFactor);
            output *= reductionFactor;
            
            dataout[i] = output * gain;
        }
    }
    
    /*
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
    */
    
    /*
    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    */
protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    bool first = true;
//    bool keytrackOn = false;
    float lastEnv = -1.f;
    sst::basic_blocks::dsp::SlewLimiter attackSL;
    sst::basic_blocks::dsp::SlewLimiter releaseSL;
    sst::basic_blocks::dsp::RunningAverage RA;
    
    //    float hpFreqPrior = -1.f;
    //    float lpFreqPrior = -1.f;
    //    std::sst::filters::CytomicSVF, 2> filters;
};
} // namespace sst::voice_effects::dynamics
#endif // SCXT_COMPRESSOR_H
