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

#ifndef INCLUDE_SST_EFFECTS_FLOATY_DELAY_H
#define INCLUDE_SST_EFFECTS_FLOATY_DELAY_H

#include <cstring>
#include <cmath>
#include <utility>
#include <iostream>

#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"

#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"

#include "sst/basic-blocks/mechanics/simd-ops.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

#include "sst/basic-blocks/tables/SincTableProvider.h"
#include "sst/basic-blocks/dsp/SSESincDelayLine.h"

#include "sst/filters/CytomicSVF.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::effects::floatydelay
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct FloatyDelay : core::EffectTemplateBase<FXConfig>
{
    enum floaty_params
    {
        fld_mix = 0,
        fld_time,
        fld_offset,
        fld_feedback,
        fld_warp_rate,
        fld_warp_depth_1,
        fld_warp_depth_2,
        fld_cutoff,
        fld_resonance,
        fld_playrate,
        fld_test,

        fld_num_params,
    };

    static constexpr int numParams{fld_num_params};
    static constexpr const char *effectName{"Floaty Delay"};

    basic_blocks::dsp::RNG rng;

    FloatyDelay(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return -1; }
    void onSampleRateChanged() { initialize(); }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fld_mix:
            return pmd().withName("Mix").asPercent().withDefault(0.3f);

        case fld_time:
            return pmd()
                .asEnvelopeTime()
                .withRange(-5.64386f, 3.f) // 20ms to 8s
                .withDefault(-1.73697f) // 300ms
                .withName("Time");

        case fld_offset:
            return pmd()
                .asFloat()
                .withRange(-10.f, 10.f)
                .withDefault(0.f)
                .withLinearScaleFormatting("ms", 2.f)
                .withName("L/R Offset");

        case fld_feedback:
            return pmd().asPercent().withDefault(.5f).withName("Feedback");

        case fld_warp_rate:
            return pmd().asLfoRate(-3, 4).withName("Warp Rate");

        case fld_warp_depth_1:
            return pmd()
                .asPercent()
                .withName("Pitch Warp");
                
        case fld_warp_depth_2:
            return pmd()
                .asPercent()
                .withName("Filter Warp");

        case fld_cutoff:
            return pmd().asAudibleFrequency().withDefault(20.f).withName("Cutoff");

        case fld_resonance:
            return pmd().asPercent().withName("Resonance").withDefault(.5f);

        case fld_playrate:
            return pmd()
                .asFloat()
                .withRange(-4, 4)
                .withName("Playrate")
                .withDefault(1);
        case fld_test:
            return pmd()
                .asFloat()
                .withRange(-1, 1)
                .withName("test")
                .withDefault(0);
        }
        return {};
    }
    
    int samplerate = this->sampleRate();
    const float sampleRateInv = 1 / this->sampleRate();
    inline float envelope_rate_linear_nowrap(float f)
    {
        return this->envelopeRateLinear(f);
    }
    
    
  protected:
    static constexpr int max_delay_length{1 << 19};
    
    const sst::basic_blocks::tables::SurgeSincTableProvider sincTable;
    using line_t = sst::basic_blocks::dsp::SSESincDelayLine<max_delay_length>;
    line_t delayLineL{sincTable};
    line_t delayLineR{sincTable};
    
    float min_delay_length = static_cast<float>(sincTable.FIRipol_N);
    
    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<FloatyDelay, FXConfig::blockSize>;
    lfo_t sineLFO{this, rng};
    lfo_t noiseLFO{this, rng};
    typename lfo_t::Shape sine = lfo_t::Shape::SINE;
    typename lfo_t::Shape noise = lfo_t::Shape::SMOOTH_NOISE;
    
    sst::filters::CytomicSVF filter;
    sst::filters::CytomicSVF DCfilter;
    
    sst::basic_blocks::dsp::lipol_sse<FXConfig::blockSize, false> timeLerp, offsLerp, rateLerp, feedbackLerp, mixLerp;
    
    // int ringout_time;

    inline float rateToSeconds(float f)
    {
        return std::pow(2, f);
    }
                                                                 
    inline void softClip(float &L, float &R)
    {
        L = std::clamp(L, -1.5f, 1.5f);
        L = L - 4.0 / 27.0 * L * L * L;
        
        R = std::clamp(R, -1.5f, 1.5f);
        R = R - 4.0 / 27.0 * R * R * R;
    }
    
    float readHeadMove{0};
    // overlap smoothing window of about 2ms in double speed
    int osw = 64;
    
    int counter{0};
    
    bool test{false};
    float prior{-1};
};

template <typename FXConfig> inline void FloatyDelay<FXConfig>::initialize()
{
    // ringout_time = 100000;
    filter.init();
    DCfilter.init();
    DCfilter.template setCoeffForBlock<FXConfig::blockSize>(sst::filters::CytomicSVF::HP, 30.f, .5f, sampleRateInv, 0.f);
    timeLerp.instantize();
    offsLerp.instantize();
    rateLerp.instantize();
    feedbackLerp.instantize();
    mixLerp.instantize();
    delayLineL.clear();
    delayLineR.clear();
    sineLFO.attack(sine);
    noiseLFO.attack(noise);
}

template <typename FXConfig>
inline void FloatyDelay<FXConfig>::processBlock(float *dataL, float *dataR)
{
//    if (prior != this->floatValue(fld_test))
//    {
//        test = true;
//        prior = this->floatValue(fld_test);
//    }
    float wr = this->floatValue(fld_warp_rate);
    float wd1 = this->floatValue(fld_warp_depth_1);
    float wd2 = this->floatValue(fld_warp_depth_2);
    sineLFO.process_block(wr, 0.f, sine);
    noiseLFO.process_block(wr + 1, 0.f, noise);
    float mod = (sineLFO.lastTarget * 0.66f + noiseLFO.lastTarget * 0.33f);
    
    auto freqL = 440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_cutoff) + mod * wd2 * 12.f);
    auto freqR = 440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_cutoff) - mod * wd2 * 12.f);
    auto res =  this->floatValue(fld_resonance);
    filter.template setCoeffForBlock<FXConfig::blockSize>(sst::filters::CytomicSVF::LP, freqL, freqR, res, res, sampleRateInv, 0.f, 0.f);
    DCfilter.template retainCoeffForBlock<FXConfig::blockSize>();
  
    float baseTime = std::clamp(rateToSeconds(this->floatValue(fld_time)),.002f, 8.f) * this->sampleRate();
    
    mod *= wd1;
    mod *= .01225f * (baseTime - .002f) + .002f;
    
    timeLerp.set_target(baseTime + mod);
    float time alignas(16)[FXConfig::blockSize];
    timeLerp.store_block(time);

    offsLerp.set_target(std::clamp(this->floatValue(fld_offset), -10.f, 10.f) * .001 * this->sampleRate());
    float offset alignas(16)[FXConfig::blockSize];
    offsLerp.store_block(offset);
    
    rateLerp.set_target(this->floatValue(fld_playrate));
    float playrate alignas(16)[FXConfig::blockSize];
    rateLerp.store_block(playrate);
    
    float fb = this->floatValue(fld_feedback);
    feedbackLerp.set_target(fb);
    float feedback alignas(16)[FXConfig::blockSize];
    feedbackLerp.store_block(feedback);
    

    
    float dBufferL alignas(16)[FXConfig::blockSize];
    float dBufferR alignas(16)[FXConfig::blockSize];

    for (int i = 0; i < FXConfig::blockSize; i++)
    {
        if (readHeadMove >= baseTime * playrate[i])
        {
            readHeadMove = 0;
        }
        
        float increment = std::fabsf(playrate[i]) - 1;
        
        auto readPos = time[i] * playrate[i];
        readPos -= (playrate[i] >= 0) ? readHeadMove : readPos - readHeadMove;
        
        readHeadMove += increment;
        
        auto readPosL = readPos + offset[i];
        auto readPosR = readPos - offset[i];

        auto fromLineL = delayLineL.read(std::max(readPosL, min_delay_length));
        auto fromLineR = delayLineR.read(std::max(readPosR, min_delay_length));
        
        DCfilter.processBlockStep(fromLineL, fromLineR);

        
        dBufferL[i] = fromLineL;
        dBufferR[i] = fromLineR;
        
        auto toLineL = feedback[i] * dBufferL[i] + dataL[i];
        auto toLineR = feedback[i] * dBufferR[i] + dataR[i];
        
        filter.processBlockStep(toLineL, toLineR);
        softClip(toLineL, toLineR);
        
        // overlap smoothing
        // In repeat and/or non-1 playrate, read and write will approach each other,
        // turn the delay output down gradually in a little window around the approach.
        // float ol = std::abs(RP - wp);
        // float ols = (ol > osw) ? 1 : ol / osw;

        delayLineL.write(toLineL);
        delayLineR.write(toLineR);
        
//        if (test)
//        {
//            std::cout << "dataL = " << dataL[i] << std::endl;
//            std::cout << "dataR = " << dataR[i] << std::endl;
//            test = false;
//        }
    }

    
    mixLerp.set_target(this->floatValue(fld_mix));
    mixLerp.fade_2_blocks_inplace(dataL, dBufferL, dataR, dBufferR, this->blockSize_quad);
}
} // namespace sst::effects::floatydelay
#endif // FLOATYDELAY_H
