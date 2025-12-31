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

#ifndef INCLUDE_SST_EFFECTS_FLOATYDELAY_H
#define INCLUDE_SST_EFFECTS_FLOATYDELAY_H

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

/*
 Big thanks to Danial Arena at remaincalm.org for writing the reaper JS plug which
inspired this effect! :)
 */

namespace sst::effects::floatydelay
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct FloatyDelay : core::EffectTemplateBase<FXConfig>
{
    enum floaty_params
    {
        fld_time,
        fld_playrate,
        fld_feedback,
        fld_cutoff,
        fld_resonance,
        fld_warp_rate,
        fld_warp_width,
        fld_pitch_warp_depth,
        fld_filt_warp_depth,
        fld_HP_freq,
        fld_mix,
        fld_num_params,
    };

    static constexpr int numParams{fld_num_params};
    static constexpr const char *streamingName{"floatydelay"};
    static constexpr const char *displayName{"Floaty Delay"};

    basic_blocks::dsp::RNG rng;

    FloatyDelay(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
        static_assert(core::ValidEffect<FloatyDelay>);
    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return -1; }
    float silentSamplesLastCheck;
    size_t silentSamplesVal{0};
    size_t silentSamplesLength()
    {
        auto t1 = this->floatValue(fld_time);
        if (t1 != silentSamplesLastCheck)
        {
            auto t = pow(2.f, t1);
            t *= this->temposyncRatio(fld_time);
            this->silentSamplesVal = (size_t)(1.1 * t * this->sampleRate());
            this->silentSamplesLastCheck = t1;
        }
        return silentSamplesVal;
    }
    void onSampleRateChanged() { initialize(); }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fld_time:
            return pmd()
                .asEnvelopeTime()
                .withRange(-5.64386f, 3.f) // 20 ms to 8 s
                .withDefault(-1.73697f)    // 300 ms
                .withName("Time");

        case fld_playrate:
            return pmd()
                .asFloat()
                .withRange(-5, 5)
                .withName("Playrate")
                .withDefault(1)
                .withFeature(pmd::Features::ALLOW_FRACTIONAL_TYPEINS)
                .withLinearScaleFormatting("x");

        case fld_feedback:
            return pmd().asPercent().withDefault(.5f).withName("Feedback");

        case fld_cutoff:
            return pmd().asAudibleFrequency().withDefault(20.f).withName("Cutoff");

        case fld_resonance:
            return pmd().asPercent().withName("Resonance").withDefault(.5f);

        case fld_warp_rate:
            return pmd().asLfoRate(-3, 4).withName("Rate").temposyncable(false);

        case fld_warp_width:
            return pmd().asPercent().withDefault(0.f).withName("Width");

        case fld_pitch_warp_depth:
            return pmd().asPercent().withName("Pitch Depth");

        case fld_filt_warp_depth:
            return pmd().asPercent().withName("Filter Depth");

        case fld_HP_freq:
            return pmd().asAudibleFrequency().withDefault(-60).withName("Low Cut");

        case fld_mix:
            return pmd().withName("Mix").asPercent().withDefault(0.3f);
        }
        return {};
    }

    int samplerate = this->sampleRate();
    const float sampleRateInv = 1 / this->sampleRate();
    inline float envelope_rate_linear_nowrap(float f) { return this->envelopeRateLinear(f); }

  protected:
    static constexpr int max_delay_length{1 << 19};

    const sst::basic_blocks::tables::SurgeSincTableProvider sincTable;
    using line_t = sst::basic_blocks::dsp::SSESincDelayLine<max_delay_length>;
    line_t delayLineL{sincTable};
    line_t delayLineR{sincTable};

    float min_delay_length = static_cast<float>(sincTable.FIRipol_N);

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<FloatyDelay, FXConfig::blockSize>;
    lfo_t sineLFO{this, rng};
    lfo_t noiseLFO1{this, rng};
    lfo_t noiseLFO2{this, rng};
    typename lfo_t::Shape sine = lfo_t::Shape::SINE;
    typename lfo_t::Shape noise = lfo_t::Shape::SMOOTH_NOISE;

    sst::filters::CytomicSVF inputFilter;
    sst::filters::CytomicSVF feedbackFilter;
    sst::filters::CytomicSVF DCfilter;
    sst::filters::CytomicSVF HPfilter;

    sst::basic_blocks::dsp::lipol_sse<FXConfig::blockSize, false> timeLerp, modLerpL, modLerpR,
        rateLerp, feedbackLerp, mixLerp;

    // int ringout_time;

    inline float rateToSeconds(float f) { return std::pow(2, f); }

    inline void softClip(float &L, float &R)
    {
        L = std::clamp(L, -1.5f, 1.5f);
        L = L - 4.0 / 27.0 * L * L * L;

        R = std::clamp(R, -1.5f, 1.5f);
        R = R - 4.0 / 27.0 * R * R * R;
    }

    float readHeadMove{0};
    bool wasOne{true};
};

template <typename FXConfig> inline void FloatyDelay<FXConfig>::initialize()
{
    // ringout_time = 100000;
    inputFilter.init();
    feedbackFilter.init();
    DCfilter.init();
    DCfilter.template setCoeffForBlock<FXConfig::blockSize>(
        sst::filters::CytomicSVF::Mode::Highpass, 30.f, .5f, sampleRateInv, 0.f);
    HPfilter.init();
    timeLerp.instantize();
    modLerpL.instantize();
    modLerpR.instantize();
    rateLerp.instantize();
    feedbackLerp.instantize();
    mixLerp.instantize();
    delayLineL.clear();
    delayLineR.clear();
    sineLFO.attack(sine);
    noiseLFO1.attack(noise);
    noiseLFO2.attack(noise);
}

template <typename FXConfig>
inline void FloatyDelay<FXConfig>::processBlock(float *dataL, float *dataR)
{
    float wr = this->floatValue(fld_warp_rate);
    float ww = this->floatValue(fld_warp_width);
    float pd = this->floatValue(fld_pitch_warp_depth);
    float fd = this->floatValue(fld_filt_warp_depth);

    sineLFO.process_block(wr, 0.f, sine);
    noiseLFO1.process_block(wr + 1, 0.f, noise);
    noiseLFO2.process_block(wr + 1, 0.f, noise);
    float sine = sineLFO.lastTarget;
    float noise1 = noiseLFO1.lastTarget;
    float noise2 = noiseLFO2.lastTarget;

    float mL = sine + noise1;
    float mR = sine + (noise1 * (1 - ww)) + (noise2 * ww);

    // input filter is modulated, feedback filter is static 2 1/2 octaves above the input one

    auto freqL =
        440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_cutoff) + mL * fd * 24.f);
    auto freqR =
        440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_cutoff) + mR * fd * 24.f);
    freqL = std::clamp(freqL, 20.f, 20000.f);
    freqR = std::clamp(freqR, 20.f, 20000.f);
    auto freqFB = 440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_cutoff) + 31.02f);
    auto res = this->floatValue(fld_resonance);
    inputFilter.template setCoeffForBlock<FXConfig::blockSize>(
        sst::filters::CytomicSVF::Mode::Lowpass, freqL, freqR, res, res, sampleRateInv, 0.f, 0.f);
    feedbackFilter.template setCoeffForBlock<FXConfig::blockSize>(
        sst::filters::CytomicSVF::Mode::Lowpass, freqFB, freqFB, .55f, .55f, sampleRateInv, 0.f,
        0.f);
    DCfilter.template retainCoeffForBlock<FXConfig::blockSize>();

    float baseTime =
        std::clamp(rateToSeconds(this->floatValue(fld_time)), .002f, 8.f) * this->sampleRate();
    timeLerp.set_target(baseTime);
    float time alignas(16)[FXConfig::blockSize];
    timeLerp.store_block(time);

    mL *= pd;
    mR *= pd;
    // FIXME: This tries and fails to make pitch warp depth consistent between long and short times
    mL *= .01225f * (baseTime - .002f) + .002f;
    mR *= .01225f * (baseTime - .002f) + .002f;

    modLerpL.set_target(mL);
    modLerpR.set_target(mR);
    float modL alignas(16)[FXConfig::blockSize];
    float modR alignas(16)[FXConfig::blockSize];
    modLerpL.store_block(modL);
    modLerpR.store_block(modR);

    rateLerp.set_target(this->floatValue(fld_playrate));
    float playrate alignas(16)[FXConfig::blockSize];
    rateLerp.store_block(playrate);

    float fb = this->floatValue(fld_feedback);
    feedbackLerp.set_target(fb);
    float feedback alignas(16)[FXConfig::blockSize];
    feedbackLerp.store_block(feedback);

    float dBufferL alignas(16)[FXConfig::blockSize];
    float dBufferR alignas(16)[FXConfig::blockSize];

    float smooth{1.f};
    float smoothWindow = 256.f;

    float HPfreq = 440 * this->noteToPitchIgnoringTuning(this->floatValue(fld_HP_freq));
    HPfilter.template setCoeffForBlock<FXConfig::blockSize>(
        sst::filters::CytomicSVF::Mode::Highpass, HPfreq, .55f, sampleRateInv, 0.f);

    for (int i = 0; i < FXConfig::blockSize; i++)
    {
        auto absrate = std::fabs(playrate[i]);
        auto absRHM = std::fabs(readHeadMove);
        auto adjustedTime = baseTime * absrate;

        if (absRHM >= adjustedTime)
        {
            readHeadMove = 0;
        }

        auto readPos = adjustedTime;
        readPos -= (playrate[i] >= 0) ? readHeadMove : readPos - readHeadMove;
        // The increment determines the playback speed.
        // Write head advances 1 each sample, and we're setting read positions relative to it,
        // hence the -1 in fwd and +1 in rev.
        float increment = (playrate[i] >= 0) ? absrate - 1 : absrate + 1;
        readHeadMove += increment;

        auto readPosL = readPos + modL[i];
        auto readPosR = readPos + modR[i];

        // Clamp at min_delay lest bad things happen
        readPosL = std::max(readPosL, min_delay_length);
        readPosR = std::max(readPosR, min_delay_length);

        /*
         Smoothing stragegy
         In any playrate except 1, turn the read head signal down around each clicky jump
         // TODO: Improve...
         Either try a 2-head strategy or make the window size relative to the speed
         */
        if (playrate[i] == 1)
        {
            smooth = 1.f; // no smoothing needed

            if (!wasOne)
            {
                if (readHeadMove > 1) // but delay time will be wrong
                {
                    readHeadMove -= 1; // so wind the head back towards zero
                }
                else if (readHeadMove < 1)
                {
                    readHeadMove += 1;
                }
                // (yeah yeah, not quite exactly zero maybe, but close enough)
                wasOne = true;
            }
        }
        else
        {
            wasOne = false;
            if (playrate[i] < 0)
            {
                smoothWindow = 512.f; // lengthen the window in reverse
            }
            // this slightly cursed nest of mins answers "how far are we from a jump"
            auto s = std::min(std::min(smoothWindow, absRHM),
                              std::min(smoothWindow, adjustedTime - absRHM));
            // the answer has no business outside these bounds
            s = std::clamp(s, 0.f, smoothWindow);
            // divide it by the window total to get the smoothing amount
            smooth = s / smoothWindow;
        }

        auto fromLineL = delayLineL.read(readPosL) * smooth;
        auto fromLineR = delayLineR.read(readPosR) * smooth;

        // lest very slow speeds get a little unwieldy
        DCfilter.processBlockStep(fromLineL, fromLineR);

        dBufferL[i] = fromLineL;
        dBufferR[i] = fromLineR;

        auto inL = dataL[i];
        auto inR = dataR[i];
        inputFilter.processBlockStep(inL, inR);

        auto toLineL = inL + feedback[i] * dBufferL[i];
        auto toLineR = inR + feedback[i] * dBufferR[i];

        feedbackFilter.processBlockStep(toLineL, toLineR);
        softClip(toLineL, toLineR);

        delayLineL.write(toLineL);
        delayLineR.write(toLineR);
    }

    HPfilter.template processBlock<FXConfig::blockSize>(dBufferL, dBufferR, dBufferL, dBufferR);

    mixLerp.set_target(this->floatValue(fld_mix));
    mixLerp.fade_2_blocks_inplace(dataL, dBufferL, dataR, dBufferR, this->blockSize_quad);
}
} // namespace sst::effects::floatydelay
#endif // FLOATYDELAY_H
