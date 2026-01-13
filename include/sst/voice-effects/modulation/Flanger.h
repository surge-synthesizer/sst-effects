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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_FLANGER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_FLANGER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <cmath>
#include <array>

#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/RNG.h"
#include "sst/basic-blocks/simd/setup.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"
#include "sst/basic-blocks/modulators/FXModControl.h"
#include "sst/basic-blocks/tables/SimpleSineProvider.h"
#include "sst/basic-blocks/dsp/Interpolators.h"
#include "../delay/DelaySupport.h"
#include "sst/filters++/api.h"

namespace sst::voice_effects::modulation
{
#define ADD(a, b) SIMD_MM(add_ps)(a, b)
#define SUB(a, b) SIMD_MM(sub_ps)(a, b)
#define DIV(a, b) SIMD_MM(div_ps)(a, b)
#define MUL(a, b) SIMD_MM(mul_ps)(a, b)
#define SETALL(a) SIMD_MM(set1_ps)(a)

template <typename VFXConfig> struct VoiceFlanger : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"VoiceFlanger"};
    static constexpr const char *streamingName{"voice-flanger"};

    // really we just need enough for 4 * 110Hz
    static constexpr float maxTotalMilliseconds{50};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{3};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpRoot,
        fpFeedback,
        fpRate,
        fpDepth,
    };

    enum IntParams
    {
        ipStereo,
        ipPolarity,
        ipBehavior,
    };

    using SineTable = basic_blocks::tables::SimpleSineProvider;
    SineTable &sT;

    VoiceFlanger(SineTable &sineTable)
        : core::VoiceEffectTemplateBase<VFXConfig>(), sT(sineTable), quadHelper(&sT)
    {
    }
    ~VoiceFlanger() { modLines.returnAll(this); }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case fpRoot:
            if (keytrackOn)
            {
                return pmd().asFloat().withName("Tune").asSemitoneRange(-36, 36).withDefault(0);
            }
            return pmd()
                .asFloat()
                .withName("Base Freq")
                .withSemitoneZeroAt440Formatting()
                .withDefault(-0)
                .withRange(-24, 24.f);
        case fpFeedback:
            return pmd().asPercent().withDefault(0.75f).withName("Feedback");
        case fpRate:
            return pmd().asLfoRate(-7, 3.3219280f).withDefault(-2).withName("Rate");
        case fpDepth:
            return pmd().asPercent().withDefault(.5f).withName("Depth");
        }
        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipPolarity:
            return pmd()
                .asInt()
                .withRange(0, 1)
                .withDefault(false)
                .withName("Polarity")
                .withUnorderedMapFormatting({
                    {0, "Norm"},
                    {1, "Inv"},
                });
        case ipBehavior:
            return pmd()
                .asBool()
                .withDefault(false)
                .withUnorderedMapFormatting({
                    {false, "Steady"},
                    {true, "Random"},
                })
                .withName("Rotation");
        }
        return pmd().asInt().withName("Error");
    }

    size_t lineSize() const
    {
        int sz{1};

        while (this->getSampleRate() * maxTotalMilliseconds * .001 > 1 << sz)
        {
            sz++;
        }

        return static_cast<size_t>(std::clamp(sz, 12, 20));
    }

    void initVoiceEffect()
    {
        modLines.returnAllExcept(lineSize(), this);
        modLines.reservePrepareAndClear(lineSize(), this);

        SampleRateSSE = SETALL(this->getSampleRate());
        float p = this->getFloatParam(fpRoot); // TODO: pitch awareness
        while (p < -24)
        {
            p += 12.f;
        }
        auto freqSSE =
            MUL(DIV(ONE, MUL(SETALL(440 * this->note_to_pitch_ignoring_tuning(p)), COMBSPACE)),
                SampleRateSSE);

        fixTimeLerp.set_initial_targets(freqSSE);

        // same time calculations as in the impl, using the LFO values at phase == 0
        modTimeLerp.set_initial_targets(
            ADD(freqSSE, MUL(MUL(SIMD_MM(set_ps)(0.f, -1.f, 0.f, 1.f), freqSSE),
                             SETALL(.95f * std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f)))));
        leftLerp.set_initial_targets(SIMD_MM(set_ps)(0.707f, 1.f, 0.707f, 0.f));
        rightLerp.set_initial_targets(SIMD_MM(set_ps)(0.707f, 0.f, 0.707f, 1.f));
        feedbackLerp.set_target_instant(this->getFloatParam(fpFeedback));

        LPfilter.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
        LPfilter.setFilterModel(filtersplusplus::FilterModel::CytomicSVF);
        LPfilter.setPassband(filtersplusplus::Passband::LP);
        if (!LPfilter.prepareInstance())
        {
            std::cout << "LP filter prep failed" << std::endl;
        }
        LPfilter.makeConstantCoefficients(0, 65.f, 0.f);
        for (int i = 1; i < 3; ++i)
        {
            LPfilter.copyCoefficientsFromVoiceToVoice(0, i);
        }

        DCblocker.setCoeff(filters::CytomicSVF::Mode::Highpass, -60, -0.f,
                           this->getSampleRateInv());
        DCblocker.template retainCoeffForBlock<VFXConfig::blockSize>();
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <typename T>
    void stereoImpl(const T &lines, const float *const datainL, const float *const datainR,
                    float *dataoutL, float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        float p = this->getFloatParam(fpRoot) + (pitch * keytrackOn);
        while (p < -24)
        {
            p += 12.f;
        }
        // 1.f / (Hz * spacing) * this->getSampleRate();
        auto freqSSE =
            MUL(DIV(ONE, MUL(SETALL(440 * this->note_to_pitch_ignoring_tuning(p)), COMBSPACE)),
                SampleRateSSE);

        fixTimeLerp.set_targets(freqSSE);
        SIMD_M128 fixedTime[VFXConfig::blockSize];
        fixTimeLerp.store_block(fixedTime);

        double rate = this->envelope_rate_linear_nowrap(-this->getFloatParam(fpRate)) *
                      this->getTempoSyncRatio();

        lfoPhase = std::fmod(lfoPhase + rate, 1.f);
        float currentPhase{lfoPhase};

        if (this->getIntParam(ipBehavior))
        {
            if (lfoPhase - rate <= 0.f)
            {
                rnd_hist[3] = rnd_hist[2];
                rnd_hist[2] = rnd_hist[1];
                rnd_hist[1] = rnd_hist[0];
                rnd_hist[0] = rng.unif01();
            }
            currentPhase = sst::basic_blocks::dsp::cubic_ipol(rnd_hist[3], rnd_hist[2], rnd_hist[1],
                                                              rnd_hist[0], lfoPhase);
        }

        SIMD_M128 lv, pl, pr; // lfo values, left pan factors, right pan factors
        quadHelper.newValues(currentPhase, lv, pl, pr);

        // the 5% downscale prevents delay time hitting 0. Sounds bad-ish if it does.
        auto depth = .95f * std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f);
        // delay time in samples = freq + freq * lfo values * depth
        modTimeLerp.set_targets(ADD(freqSSE, MUL(MUL(lv, freqSSE), SETALL(depth))));
        SIMD_M128 time[VFXConfig::blockSize];
        modTimeLerp.store_block(time);

        leftLerp.set_targets(pl);
        rightLerp.set_targets(pr);
        SIMD_M128 leftPans[VFXConfig::blockSize];
        SIMD_M128 rightPans[VFXConfig::blockSize];
        leftLerp.store_block(leftPans);
        rightLerp.store_block(rightPans);

        auto fbp = std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f);
        feedbackLerp.set_target(fbp * fbp);
        float fbAmt alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fbAmt);

        // -1 or 1
        SIMD_M128 POL = SETALL(1.f - this->getIntParam(ipPolarity) * 2.f);

        LPfilter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromModLine = lines->read(time[i]);
            QuadHelper::panLinesToOutputs(leftPans[i], rightPans[i], fromModLine, dataoutL[i],
                                          dataoutR[i]);

            SIMD_M128 inputs =
                QuadHelper::balancedMonoSum(leftPans[i], rightPans[i], datainL[i], datainR[i]);

            auto backToLine = MUL(SETALL(fbAmt[i]), fromModLine);

            // feedback gets the (sqrt2*x)/(1+x^2) treatment
            backToLine = DIV(MUL(backToLine, SQRT2), ADD(ONE, MUL(backToLine, backToLine)));
            // then filter off a tiny bit of highs
            backToLine = LPfilter.processSample(backToLine);
            backToLine = MUL(backToLine, POL);

            lines->write(ADD(inputs, backToLine));
        }
        LPfilter.concludeBlock();
        DCblocker.processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL, dataoutR);

        mech::scale_by<VFXConfig::blockSize>(.5f, dataoutL, dataoutR);
    }

    template <typename T>
    void monoImpl(T *lines, const float *const datain, float *dataout, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        float p = this->getFloatParam(fpRoot) + (pitch * keytrackOn);
        while (p < -24)
        {
            p += 12.f;
        }
        auto freqSSE =
            MUL(DIV(ONE, MUL(SETALL(440 * this->note_to_pitch_ignoring_tuning(p)), COMBSPACE)),
                SampleRateSSE);

        fixTimeLerp.set_targets(freqSSE);
        SIMD_M128 fixedTime[VFXConfig::blockSize];
        fixTimeLerp.store_block(fixedTime);

        double rate = this->envelope_rate_linear_nowrap(-this->getFloatParam(fpRate)) *
                      this->getTempoSyncRatio();

        lfoPhase = std::fmod(lfoPhase + rate, 1.f);
        monoLfoPhase = std::fmod(lfoPhase + rate * .6666667f, 1.f);
        float curLFOPhase{lfoPhase};
        float curLevPhase{lfoPhase};

        if (this->getIntParam(ipBehavior))
        {
            if (lfoPhase - rate <= 0.f)
            {
                rnd_hist[3] = rnd_hist[2];
                rnd_hist[2] = rnd_hist[1];
                rnd_hist[1] = rnd_hist[0];
                rnd_hist[0] = rng.unif01();
            }
            curLFOPhase = sst::basic_blocks::dsp::cubic_ipol(rnd_hist[3], rnd_hist[2], rnd_hist[1],
                                                             rnd_hist[0], lfoPhase);
        }

        SIMD_M128 lfov, levv; // lfo values, level values
        quadHelper.monoLFOVals(curLFOPhase, lfov);
        quadHelper.monoLevels(curLevPhase, levv);

        auto depth = .95f * std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f);
        modTimeLerp.set_targets(ADD(freqSSE, MUL(MUL(lfov, freqSSE), SETALL(depth))));
        SIMD_M128 time[VFXConfig::blockSize];
        modTimeLerp.store_block(time);

        leftLerp.set_targets(levv);
        SIMD_M128 levels[VFXConfig::blockSize];
        leftLerp.store_block(levels);

        auto fbp = std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f);
        feedbackLerp.set_target(fbp * fbp);
        float fbAmt alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fbAmt);

        // -1 or 1
        SIMD_M128 POL = SETALL(1.f - this->getIntParam(ipPolarity) * 2.f);

        LPfilter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromModLine = lines->read(time[i]);
            dataout[i] = .5f * mech::hsum_ps(MUL(levels[i], fromModLine));

            auto backToLine = MUL(SETALL(fbAmt[i]), fromModLine);
            backToLine = DIV(MUL(backToLine, SQRT2), ADD(ONE, MUL(backToLine, backToLine)));
            backToLine = LPfilter.processSample(backToLine);
            backToLine = MUL(backToLine, POL);

            SIMD_M128 inputs = MUL(levels[i], SETALL(datain[i]));
            lines->write(ADD(inputs, backToLine));
        }
        LPfilter.concludeBlock();
        DCblocker.processBlock<VFXConfig::blockSize>(dataout, dataout);
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        modLines.dispatch(lineSize(), [&](auto N) {
            auto *lines = modLines.template getLinePointer<N>();
            stereoImpl(lines, datainL, datainR, dataoutL, dataoutR, pitch);
        });
    }

    void processMonoToStereo(const float *const datain, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        float dataInv alignas(16)[VFXConfig::blockSize];
        mech::copy_from_to<VFXConfig::blockSize>(datain, dataInv);
        mech::scale_by<VFXConfig::blockSize>(-0.f, dataInv);

        processStereo(dataInv, datain, dataoutL, dataoutR, pitch);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        modLines.dispatch(lineSize(), [&](auto N) {
            auto *lines = modLines.template getLinePointer<N>();
            monoImpl(lines, datain, dataout, pitch);
        });
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

    size_t silentSamplesLength() const
    {
        return this->getSampleRate() * maxTotalMilliseconds * .001;
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
    bool keytrackOn{false};
    bool first{true};
    delay::details::DelayLineSupport<sst::basic_blocks::dsp::quadDelayLine> modLines;
    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> feedbackLerp;
    filtersplusplus::Filter LPfilter;
    filters::CytomicSVF DCblocker;

    float lfoPhase;
    float monoLfoPhase;
    float rnd_hist[4] = {0, 0, 0, 0};

    const SIMD_M128 ONE = SETALL(1.f);
    const SIMD_M128 NEGONE = SETALL(-1.f);
    const SIMD_M128 TWO = SETALL(2.f);
    const SIMD_M128 HALF = SETALL(.5f);
    const SIMD_M128 SQRT2 = SETALL(M_SQRT2);
    // comb freqs are spaced 4:5:7:9
    const SIMD_M128 COMBSPACE = SIMD_MM(set_ps)(2.25f, 1.75f, 1.5, 1.f);
    SIMD_M128 SampleRateSSE{};

    struct QuadHelper
    {
        QuadHelper(SineTable *sineTable) : sine(sineTable->table)
        {
            auto tS = static_cast<float>(sineTable->tableSize);
            auto tM = sineTable->tableSize - 1;
            LFO_TABLE_SIZE_SSE = SIMD_MM(set1_ps)(tS);
            LFO_TABLE_MASK_SSE = SIMD_MM(set1_epi32)(tM);
        }
        ~QuadHelper() {}

        /* This gives you 4 sine outputs at 0, 90°, 180° and 270°,
         * plus 4 sines for panning the 4 signals
         * around in sync with the modulation.
         * If you feed in a ramp or saw you get the regular tri/sine,
         * But you can also feed in other waveforms and retain the quadrature relations
         * See basic_blocks::Modulators::FXModControl for more detail.
         */
        inline void newValues(float phase, SIMD_M128 &lfoVals, SIMD_M128 &panL, SIMD_M128 &panR)
        {
            namespace mech = sst::basic_blocks::mechanics;
            assert(sine != nullptr);
            assert(0 <= phase && phase <= 1);

            auto quadPhase = ADD(SIMD_MM(set1_ps)(phase), OFFSETS);
            quadPhase = SUB(quadPhase, SIMD_MM(floor_ps)(quadPhase));

            auto lips = MUL(quadPhase, LFO_TABLE_SIZE_SSE);
            auto lipsi = SIMD_MM(cvttps_epi32(lips));
            auto lipsn = SIMD_MM(and_si128)(SIMD_MM(add_epi32)(lipsi, SIMD_MM(set1_epi32)(1)),
                                            LFO_TABLE_MASK_SSE);
            auto lipsf = SUB(lips, SIMD_MM(cvtepi32_ps)(lipsi));
            SIMD_M128 liv = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(lipsi, 3)], sine[SIMD_MM(extract_epi32)(lipsi, 2)],
                sine[SIMD_MM(extract_epi32)(lipsi, 1)], sine[SIMD_MM(extract_epi32)(lipsi, 0)]);
            SIMD_M128 livn = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(lipsn, 3)], sine[SIMD_MM(extract_epi32)(lipsn, 2)],
                sine[SIMD_MM(extract_epi32)(lipsn, 1)], sine[SIMD_MM(extract_epi32)(lipsn, 0)]);

            lfoVals = mech::shuffle_all_ps<1>(ADD(MUL(liv, SUB(oneSSE, lipsf)), MUL(lipsf, livn)));

            quadPhase = MUL(quadPhase, halfSSE);

            auto sips = MUL(quadPhase, LFO_TABLE_SIZE_SSE);
            auto sipsi = SIMD_MM(cvttps_epi32(sips));
            auto sipsn = SIMD_MM(and_si128)(SIMD_MM(add_epi32)(sipsi, SIMD_MM(set1_epi32)(1)),
                                            LFO_TABLE_MASK_SSE);
            auto sipsf = SUB(sips, SIMD_MM(cvtepi32_ps)(sipsi));
            SIMD_M128 siv = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(sipsi, 3)], sine[SIMD_MM(extract_epi32)(sipsi, 2)],
                sine[SIMD_MM(extract_epi32)(sipsi, 1)], sine[SIMD_MM(extract_epi32)(sipsi, 0)]);
            SIMD_M128 sivn = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(sipsn, 3)], sine[SIMD_MM(extract_epi32)(sipsn, 2)],
                sine[SIMD_MM(extract_epi32)(sipsn, 1)], sine[SIMD_MM(extract_epi32)(sipsn, 0)]);

            panL = ADD(MUL(siv, SUB(oneSSE, sipsf)), MUL(sipsf, sivn));
            panR = mech::shuffle_all_ps<2>(panL);
        }

        // these are similar, except do LFOval and level computations separately
        // which is useful in MonoToMono mode
        inline void monoLFOVals(float phase, SIMD_M128 &lfoVals)
        {
            namespace mech = sst::basic_blocks::mechanics;
            assert(sine != nullptr);
            assert(0 <= phase && phase <= 1);

            auto quadPhase = ADD(SIMD_MM(set1_ps)(phase), OFFSETS);
            quadPhase = SUB(quadPhase, SIMD_MM(floor_ps)(quadPhase));

            auto lips = MUL(quadPhase, LFO_TABLE_SIZE_SSE);
            auto lipsi = SIMD_MM(cvttps_epi32(lips));
            auto lipsn = SIMD_MM(and_si128)(SIMD_MM(add_epi32)(lipsi, SIMD_MM(set1_epi32)(1)),
                                            LFO_TABLE_MASK_SSE);
            auto lipsf = SUB(lips, SIMD_MM(cvtepi32_ps)(lipsi));
            SIMD_M128 liv = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(lipsi, 3)], sine[SIMD_MM(extract_epi32)(lipsi, 2)],
                sine[SIMD_MM(extract_epi32)(lipsi, 1)], sine[SIMD_MM(extract_epi32)(lipsi, 0)]);
            SIMD_M128 livn = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(lipsn, 3)], sine[SIMD_MM(extract_epi32)(lipsn, 2)],
                sine[SIMD_MM(extract_epi32)(lipsn, 1)], sine[SIMD_MM(extract_epi32)(lipsn, 0)]);

            lfoVals = mech::shuffle_all_ps<1>(ADD(MUL(liv, SUB(oneSSE, lipsf)), MUL(lipsf, livn)));
        }

        inline void monoLevels(float phase, SIMD_M128 &levelVals)
        {
            assert(sine != nullptr);
            assert(0 <= phase && phase <= 1);

            auto quadPhase = ADD(SIMD_MM(set1_ps)(phase), OFFSETS);
            quadPhase = SUB(quadPhase, SIMD_MM(floor_ps)(quadPhase));
            quadPhase = MUL(quadPhase, halfSSE);

            auto sips = MUL(quadPhase, LFO_TABLE_SIZE_SSE);
            auto sipsi = SIMD_MM(cvttps_epi32(sips));
            auto sipsn = SIMD_MM(and_si128)(SIMD_MM(add_epi32)(sipsi, SIMD_MM(set1_epi32)(1)),
                                            LFO_TABLE_MASK_SSE);
            auto sipsf = SUB(sips, SIMD_MM(cvtepi32_ps)(sipsi));
            SIMD_M128 siv = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(sipsi, 3)], sine[SIMD_MM(extract_epi32)(sipsi, 2)],
                sine[SIMD_MM(extract_epi32)(sipsi, 1)], sine[SIMD_MM(extract_epi32)(sipsi, 0)]);
            SIMD_M128 sivn = SIMD_MM(set_ps)(
                sine[SIMD_MM(extract_epi32)(sipsn, 3)], sine[SIMD_MM(extract_epi32)(sipsn, 2)],
                sine[SIMD_MM(extract_epi32)(sipsn, 1)], sine[SIMD_MM(extract_epi32)(sipsn, 0)]);

            levelVals = ADD(MUL(siv, SUB(oneSSE, sipsf)), MUL(sipsf, sivn));
        }

        static SIMD_M128 balancedMonoSum(const SIMD_M128 fromL, const SIMD_M128 fromR, float inL,
                                         float inR)
        {
            return ADD(MUL(fromL, SETALL(inL)), MUL(fromR, SETALL(inR)));
        }

        static void panLinesToOutputs(const SIMD_M128 toL, const SIMD_M128 toR,
                                      const SIMD_M128 input, float &outL, float &outR)
        {
            namespace mech = sst::basic_blocks::mechanics;
            outL = mech::hsum_ps(MUL(toL, input));
            outR = mech::hsum_ps(MUL(toR, input));
        }

      private:
        const SIMD_M128 oneSSE{SETALL(1.f)};
        const SIMD_M128 negoneSSE{SETALL(-1.f)};
        const SIMD_M128 twoSSE{SETALL(2.f)};
        const SIMD_M128 negtwoSSE{SETALL(-2.f)};
        const SIMD_M128 halfSSE{SETALL(0.5f)};
        const SIMD_M128 fourSSE{SETALL(4.f)};
        // pointer to a float[8192] sine table
        float *sine{nullptr};
        SIMD_M128 LFO_TABLE_SIZE_SSE;
        SIMD_M128I LFO_TABLE_MASK_SSE;
        const SIMD_M128 OFFSETS = SIMD_MM(set_ps)(.75f, .5f, .25f, 0.f);
    };
    QuadHelper quadHelper;

    /* A linear interpolator that processes
     * 4 values in parallel and gives you an array of
     * __m128's to use across the block
     */
    template <int blocksize> struct quadLerp
    {
        void set_initial_targets(SIMD_M128 vals) { targets = vals; }

        void set_targets(SIMD_M128 vals)
        {
            currents = targets;
            targets = vals;
            deltas = SIMD_MM(mul_ps)(SIMD_MM(sub_ps)(targets, currents), bsInvSSE);
        }

        void store_block(SIMD_M128 *storage)
        {
            for (int i = 0; i < blocksize; ++i)
            {
                storage[i] = nextVal();
            }
        }

      private:
        SIMD_M128 targets = SIMD_MM(setzero_ps)();
        SIMD_M128 deltas = SIMD_MM(setzero_ps)();
        SIMD_M128 currents = SIMD_MM(setzero_ps)();
        SIMD_M128 bsInvSSE =
            SIMD_MM(div_ps)(SIMD_MM(set1_ps)(1.f), SIMD_MM(set1_ps)((float)blocksize));

        SIMD_M128 nextVal()
        {
            auto res = currents;
            currents = SIMD_MM(add_ps)(currents, deltas);
            return res;
        }
    };
    quadLerp<VFXConfig::blockSize> leftLerp, rightLerp, modTimeLerp, fixTimeLerp;

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
#undef ADD
#undef SUB
#undef DIV
#undef MUL
#undef SETALL
};
} // namespace sst::voice_effects::modulation

#endif // INCLUDE_SST_VOICE_EFFECTS_MODULATION_FLANGER_H
