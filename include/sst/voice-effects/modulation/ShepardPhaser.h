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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARD_PHASER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARD_PHASER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct ShepardPhaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Shepard Phaser"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{1};

    static constexpr int maxPhases{6};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpFeedback,
        fpResonance,
        fpRate,
        fpStartFreq,
        fpEndFreq,
    };

    enum IntParams
    {
        ipMode,
        ipStereo
    };

    ShepardPhaser() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~ShepardPhaser() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpFeedback:
                return pmd().asPercent().withDefault(0.5f).withName("Feedback");
        case fpResonance:
            return pmd().asPercent().withDefault(0.707).withName("Resonance");
        case fpRate:
            return pmd().asLfoRate(-7, 2).withName("Rate");
        case fpStartFreq:
            return pmd().asAudibleFrequency().withName("Start Freq").withDefault(-20);
        case fpEndFreq:
            return pmd().asAudibleFrequency().withName("End Freq").withDefault(20);
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        using md = sst::filters::CytomicSVF::Mode;
        
        switch (idx)
        {
        case ipMode:
            return pmd()
                .asBool()
                .withName("Mode")
                .withUnorderedMapFormatting({
                    {md::ALL, "AP"},
                    {md::BP, "BP"},
                })
                .withDefault(md::ALL);
        case ipStereo:
            return pmd().asBool().withDefault(false).withName("Stereo");
        }
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect()
    {
//        lipolFb.set_target_instant(
//            std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }
     
    void calc_coeffs(const float freq, const int filterIndex)
    {

    }
    

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<ShepardPhaser, VFXConfig::blockSize>;
    lfo_t sawLFO[8]{this, this, this, this, this, this, this, this};
    lfo_t triLFO[8]{this, this, this, this, this, this, this, this};
    
    
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto range = this->getFloatParam(fpEndFreq) - this->getFloatParam(fpStartFreq);
        auto res = std::clamp(this->getFloatParam(fpResonance), 0.f, 1.f);
        auto mode = (sst::filters::CytomicSVF::Mode)this->getIntParam(ipMode);
        
        namespace mech = sst::basic_blocks::mechanics;
        
        if (isFirst)
        {
            for (int i = 0; i < 8; ++i)
            {
                float sawOffset = i / 8.f;
                float triOffset = sawOffset + .75f;
                sawLFO[i].applyPhaseOffset(sawOffset);
                triLFO[i].applyPhaseOffset(triOffset);
            }
            isFirst = false;
        }
        
        mech::clear_block<VFXConfig::blockSize>(dataoutL);
        mech::clear_block<VFXConfig::blockSize>(dataoutR);
                
        for (int i = 0; i < 8; ++i)
        {
            sawLFO[i].process_block(lfoRate, 0.f, lfo_t::RAMP);
            auto freqMod = this->getFloatParam(fpStartFreq) + (range * (sawLFO[i].lastTarget * .5f + .5f));
            // set filter to it
            auto freq = 440.f * this->note_to_pitch_ignoring_tuning(freqMod);
            filters[i].template setCoeffForBlock<VFXConfig::blockSize>(mode, freq, freq, res, res, this->getSampleRateInv(), 1.f, 1.f);
            
            float tmpL alignas(16)[VFXConfig::blockSize];
            float tmpR alignas(16)[VFXConfig::blockSize];
            
            filters[i].template processBlock<VFXConfig::blockSize>(datainL, datainR, tmpL, tmpR);
            
            triLFO[i].process_block(lfoRate, .707f, lfo_t::TRI);
            auto levelMod = triLFO[i].lastTarget * .5f + .5f;
            levelMod = levelMod * levelMod * levelMod;
            lipolLevel[i].set_target(levelMod);
            lipolLevel[i].multiply_2_blocks(tmpL, tmpR);
            
            mech::scale_accumulate_from_to<VFXConfig::blockSize>(tmpL, tmpR, 0.3333f, dataoutL, dataoutR);
        }
    }
    
    /*
     Feedback stuff
    {
    
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        this->lipolFb.set_target(
            std::sqrt(std::clamp(this->getFloatParam(this->fpFeedback), 0.f, 0.97f)));
        float fb alignas(16)[VFXConfig::blockSize];
        this->lipolFb.store_block(fb);

        for (int k = 0; k < VFXConfig::blockSize; ++k)
        {
            float sumL = 0.f;
            float sumR = 0.f;
            
            float dL[4];
            float dR[4];
            
            for (int i = 0; i < 4; ++i)
            {

                
                this->filters[i].processBlockStep(dL[i], dR[i]);
                

                
                dL[i] *= triLFO[i].lastTarget * .5f + .5f;
                dR[i] *= triLFO[i].lastTarget * .5f + .5f;
                
                sumL += dL[i];
                sumR += dR[i];
            }
            dataoutL[k] = dL[0];
            dataoutR[k] = dR[0];
            
            dataoutL[k] = sumL;
            dataoutR[k] = sumR;
        }
    }
     */

protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
     std::array<sst::filters::CytomicSVF, 8> filters;
    // float fbAmt[2][4];
//    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolLevel[8];
    bool isFirst{true};
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_SHEPARD_PHASER_H
