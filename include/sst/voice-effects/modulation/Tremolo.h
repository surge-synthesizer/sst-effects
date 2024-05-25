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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_TREMOLO_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_TREMOLO_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <random>
#include <chrono>

#include <iostream>
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/filters/CytomicSVF.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct Tremolo : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Tremolo"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{3};
    
    enum  FloatParams
    {
        fpVolume,
        fpRate,
        fpDepth,
        fpCenterFreq,
    };
    
    enum IntParams
    {
        ipHarmonic,
        ipShape,
        ipStereo
    };
    
    Tremolo() : core::VoiceEffectTemplateBase<VFXConfig>() {}
    
    ~Tremolo() {}
    
    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        
        switch (idx)
        {
            case fpVolume:
                return pmd()
                .asLinearDecibel(-60.f, 12.f)
                .withName("Volume");
            case fpRate:
                return pmd()
                .asLfoRate()
                .withName("Rate");
            case fpDepth:
                return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(1.f)
                .withName("Depth");
            case fpCenterFreq:
                return pmd()
                .asFloat()
                .withRange(150.f, 1500.f)
                .withDefault(700.f)
                .withLinearScaleFormatting("Hz")
                .withName("Crossover");
        }
        return pmd().asFloat().withName("Error");
    }
    
    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
            case ipStereo:
                return pmd().asBool().withDefault(false).withName("Stereo");
            case ipShape:
                return pmd().asInt().withRange(0, 6).withName("LFO shape");
            case ipHarmonic:
                return pmd().asBool().withDefault(false).withName("Harmonic");
        }
        return pmd().asInt().withName("Error");
    }
    
    void initVoiceEffect()
    {
        filters[0].init();
        filters[1].init();
    }
    
    void initVoiceEffectParams()
    {
        this->initToParamMetadataDefault(this);
        // TODO: this doesn't compile. Why on earth not?
    }
    
    // This ain't in VFXConfig so put it here (the simpleLFO needs it):
    float envelope_rate_linear_nowrap(float f)
    {
        return VFXConfig::blockSize * VFXConfig::getSampleRateInv(this) * std::pow(2, -f);
    }
    
    // Ok, so let's introduce the simpleLFO and create one out here.
    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<Tremolo, VFXConfig::blockSize>;
    lfo_t actualLFO{this, 1};
    typename lfo_t::Shape lfoShape = lfo_t::Shape::SINE;
    
    // ...which we can then use in here to get these variables for later.
    void shapeCheck()
    {
        switch (this->getIntParam(ipShape))
        {
            case 0:
                lfoShape = lfo_t::SINE;
            break;
            case 1:
                lfoShape = lfo_t::TRI;
            break;
            case 2:
                lfoShape = lfo_t::RAMP;
            break;
            case 3:
                lfoShape = lfo_t::DOWN_RAMP;
            break;
            case 4:
                lfoShape = lfo_t::PULSE;
            break;
            case 5:
                lfoShape = lfo_t::SMOOTH_NOISE;
            break;
            case 6:
                lfoShape = lfo_t::SH_NOISE;
            break;
        }
    }
    
    // Here's a little RNG engine we'll use to randomize start phase
    float randUniZeroToOne()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        return static_cast<float>(dis(gen));
    }
    
    // We need these to initialize some stuff in the DSP functions.
    bool isFirst = true;
    bool phaseSet = false;
    
    /*
     Now we're ready for the actual DSP.
     Stereo + Mono * Standard vs. Harmonic = 4 cases, each gets its own function here.
     The process functions further down will choose which one to call.
    */
    
    void harmonicStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto CenterFreq = this->getFloatParam(fpCenterFreq);
        
        if (!phaseSet)
        {
            shapeCheck();
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }
        
        actualLFO.process_block(lfoRate, 0.f, lfoShape);
        
        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInv = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;
        
        if (isFirst)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(filters::CytomicSVF::LP, CenterFreq, CenterFreq, .707f, .707f, VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(filters::CytomicSVF::HP, CenterFreq, CenterFreq, .707f, .707f, VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            isFirst = false;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
        
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inputLeftLP = datainL[i];
            auto inputLeftHP = datainL[i];
            auto inputRightLP = datainR[i];
            auto inputRightHP = datainR[i];
            
            filters[0].processBlockStep(inputLeftLP, inputRightLP);
            filters[1].processBlockStep(inputLeftHP, inputRightHP);
            
            inputLeftLP *= 1 - lfoValue;
            inputLeftHP *= 1 - lfoValueInv;
            // These ternary statements let us do both stereo or mono modulation on stereo audio.
            // If the user asked for mono, right and left get same LFO, else they get opposite.
            inputRightLP *= 1 - (this->getIntParam(ipStereo) == true ? lfoValueInv : lfoValue);
            inputRightHP *= 1 - (this->getIntParam(ipStereo) == true ? lfoValue : lfoValueInv);
            
            inputLeftLP += inputLeftHP;
            inputRightLP += inputRightHP;
            
            dataoutL[i] = inputLeftLP * outputVolume;
            dataoutR[i] = inputRightLP * outputVolume;
        }
    }
    
    void standardStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        
        if (!phaseSet)
        {
            shapeCheck();
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }
        
        actualLFO.process_block(lfoRate, 0.f, lfoShape);
        
        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInv = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;
        
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inputL = datainL[i];
            auto inputR = datainR[i];
            
            inputL *= 1 - lfoValue;
            inputR *= 1 - (this->getIntParam(ipStereo) == true ? lfoValueInv : lfoValue);
            
            dataoutL[i] = inputL * outputVolume;
            dataoutR[i] = inputR * outputVolume;
        }
    }
    
    void harmonicMono( float *datainL, float *dataoutL, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        auto CenterFreq = this->getFloatParam(fpCenterFreq);
        
        if (!phaseSet)
        {
            shapeCheck();
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }
        
        actualLFO.process_block(lfoRate, 0.f, lfoShape);
        
        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        float lfoValueInv = (actualLFO.lastTarget * lfoDepth * -1 + 1) / 2;
        
        if (isFirst)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(filters::CytomicSVF::LP, CenterFreq, .707f, VFXConfig::getSampleRateInv(this), 0.f);
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(filters::CytomicSVF::HP, CenterFreq, .707f, VFXConfig::getSampleRateInv(this), 0.f);
            isFirst = false;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inputLP = datainL[i];
            auto inputHP = datainL[i];
            
            filters[0].processBlockStep(inputLP);
            filters[1].processBlockStep(inputHP);
            
            inputLP *= 1 - lfoValue;
            inputHP *= 1 - lfoValueInv;
            
            inputLP += inputHP;
        
            dataoutL[i] = inputLP * outputVolume;
        }
    }
    
    void standardMono(float *datainL, float *dataoutL, float pitch)
    {
        auto outputVolume = this->dbToLinear(this->getFloatParam(fpVolume));
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        
        if (!phaseSet)
        {
            shapeCheck();
            auto phase = randUniZeroToOne();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }
        
        actualLFO.process_block(lfoRate, 0.f, lfoShape);
        
        float lfoValue = (actualLFO.lastTarget * lfoDepth + 1) / 2;
        
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataoutL[i] = (datainL[i] * (1 - lfoValue)) * outputVolume;
        }
    }
    
    /*
     When starting a voice, the host will call one of the following functions.
     This first one gets called if incoming audio is Stereo...
    */
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipHarmonic) == true)
        {
            harmonicStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
        else
        {
            standardStereo(datainL, datainR, dataoutL, dataoutR, pitch);
        }
    }
    
    // ...this second one if incoming audio is Mono...
    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        if (this->getIntParam(ipHarmonic) == true)
        {
            harmonicMono(datainL, dataoutL, pitch);
        }
        else
        {
            standardMono(datainL, dataoutL, pitch);
        }
    }
    
    // ...and this last one if the input is Mono, but the user asked for stereo modulation.
    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        // which in turn simply copies the mono audio and calls the stereo one with the copies.
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }
    // How does it know which MonoTo... function to choose? By first calling this.
    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    
    protected:
        std::array<float, numFloatParams> mLastParam{};
        std::array<int, numIntParams> mLastIParam{};
        std::array<sst::filters::CytomicSVF, 2> filters;
};
} // namespace scxt
#endif //SCXT_TREMOLO_H
