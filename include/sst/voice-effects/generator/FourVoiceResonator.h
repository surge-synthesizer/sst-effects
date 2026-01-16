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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_FOURVOICE_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_FOURVOICE_H

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

namespace sst::voice_effects::generator
{
#define ADD(a, b) SIMD_MM(add_ps)(a, b)
#define SUB(a, b) SIMD_MM(sub_ps)(a, b)
#define DIV(a, b) SIMD_MM(div_ps)(a, b)
#define MUL(a, b) SIMD_MM(mul_ps)(a, b)
#define SETALL(a) SIMD_MM(set1_ps)(a)
// bruh
template <typename VFXConfig> struct FourVoiceResonator : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"FourVoiceResonator"};
    static constexpr const char *streamingName{"four-voice-resonator"};

    // really we just need enough for 4 * 110Hz
    static constexpr float maxTotalMilliseconds{50};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{4};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpRoot,
        fpFeedback,
    };

    enum IntParams
    {
        ipStereo,
        ipPolarity,
        ipChord,
        ipInversion
    };

    using SineTable = basic_blocks::tables::SimpleSineProvider;
    SineTable &sT;

    FourVoiceResonator(SineTable &sineTable)
        : core::VoiceEffectTemplateBase<VFXConfig>(), sT(sineTable), panHelper(&sT)
    {
    }
    ~FourVoiceResonator() { voices.returnAll(this); }

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
        case ipChord:
            return pmd()
                .asInt()
                .withDefault(0)
                .withRange(0, numChords - 1)
                .withUnorderedMapFormatting({
                    {0, CHORDS[0].name},
                    {1, CHORDS[1].name},
                    {2, CHORDS[2].name},
                    {3, CHORDS[3].name},
                    {4, CHORDS[4].name},
                    {5, CHORDS[5].name},
                })
                .withName("Chord");
        case ipInversion:
            return pmd()
                .asInt()
                .withDefault(0)
                .withRange(0, 3)
                .withUnorderedMapFormatting({
                    {0, "Root"},
                    {1, "First"},
                    {2, "Second"},
                    {3, "Third"},
                })
                .withName("Inversion");
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
        voices.returnAllExcept(lineSize(), this);
        voices.reservePrepareAndClear(lineSize(), this);

        if (this->getIntParam(ipStereo) > 0)
        {
            panHelper.newValues(rng.unif01(), leftPans, rightPans);
        }
        else
        {
            leftPans = SETALL(1.f);
            rightPans = SETALL(1.f);
        }

        switch (this->getIntParam(ipInversion))
        {
        default:
        case 0:
            INVERSION = SIMD_MM(set_ps)(1.f, 1.f, 1.f, 1.f);
            break;
        case 1:
            INVERSION = SIMD_MM(set_ps)(1.f, 1.f, 1.f, 2.f);
            break;
        case 2:
            INVERSION = SIMD_MM(set_ps)(1.f, 1.f, 2.f, 2.f);
            break;
        case 3:
            INVERSION = SIMD_MM(set_ps)(1.f, 2.f, 2.f, 2.f);
            break;
        }

        SampleRateSSE = SETALL(this->getSampleRate());

        feedbackLerp.set_target_instant(this->getFloatParam(fpFeedback));

        LPfilter.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
        HPfilter.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
        LPfilter.setFilterModel(filtersplusplus::FilterModel::CytomicSVF);
        HPfilter.setFilterModel(filtersplusplus::FilterModel::CytomicSVF);
        LPfilter.setPassband(filtersplusplus::Passband::LP);
        HPfilter.setPassband(filtersplusplus::Passband::HP);
        if (!LPfilter.prepareInstance())
        {
            std::cout << "LP filter prep failed" << std::endl;
        }
        if (!HPfilter.prepareInstance())
        {
            std::cout << "DC blocker prep failed" << std::endl;
        }
        LPfilter.makeConstantCoefficients(0, 65.f, 0.f);
        for (int i = 1; i < 4; ++i)
        {
            LPfilter.copyCoefficientsFromVoiceToVoice(0, i);
        }
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
        p += 12.f * this->getIntParam(ipPolarity);

        auto freqSSE = MUL(DIV(ONE, MUL(SETALL(440 * this->note_to_pitch_ignoring_tuning(p)),
                                        MUL(CHORDS[this->getIntParam(ipChord)].ratios, INVERSION))),
                           SampleRateSSE);

        timeLerp.set_targets(freqSSE);
        SIMD_M128 time[VFXConfig::blockSize];
        timeLerp.store_block(time);

        auto fbp = std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f);

        feedbackLerp.set_target(fbp * .35f + .65f); // that's the useful range pretty much
        float fbAmt alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fbAmt);

        // -1 or 1
        SIMD_M128 POL = SETALL(1.f - this->getIntParam(ipPolarity) * 2.f);

        HPfilter.makeCoefficients(0, p - 36.f, 0.f);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 1);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 2);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 3);
        LPfilter.prepareBlock();
        HPfilter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromModLine = lines->read(time[i]);

            PanHelper::panLinesToOutputs(leftPans, rightPans, fromModLine, dataoutL[i],
                                         dataoutR[i]);

            SIMD_M128 inputs =
                PanHelper::balancedMonoSum(leftPans, rightPans, datainL[i], datainR[i]);

            auto backToLine = MUL(SETALL(fbAmt[i]), fromModLine);

            // feedback gets the (sqrt2*x)/(1+x^2) treatment
            backToLine = DIV(MUL(backToLine, SQRT2), ADD(ONE, MUL(backToLine, backToLine)));
            // then filter off a tiny bit of highs
            backToLine = LPfilter.processSample(backToLine);
            backToLine = HPfilter.processSample(backToLine);
            backToLine = MUL(backToLine, POL);

            lines->write(ADD(inputs, backToLine));
        }
        LPfilter.concludeBlock();
        HPfilter.concludeBlock();

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
        p += 12.f * this->getIntParam(ipPolarity);

        auto freqSSE = MUL(DIV(ONE, MUL(SETALL(440 * this->note_to_pitch_ignoring_tuning(p)),
                                        MUL(CHORDS[this->getIntParam(ipChord)].ratios, INVERSION))),
                           SampleRateSSE);

        timeLerp.set_targets(freqSSE);
        SIMD_M128 time[VFXConfig::blockSize];
        timeLerp.store_block(time);

        auto fbp = std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f);

        feedbackLerp.set_target(fbp * .35f + .65f); // that's the useful range pretty much
        float fbAmt alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fbAmt);

        // -1 or 1
        SIMD_M128 POL = SETALL(1.f - this->getIntParam(ipPolarity) * 2.f);

        HPfilter.makeCoefficients(0, p - 36.f, 0.f);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 1);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 2);
        HPfilter.copyCoefficientsFromVoiceToVoice(0, 3);
        LPfilter.prepareBlock();
        HPfilter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromModLine = lines->read(time[i]);
            dataout[i] = .5f * mech::hsum_ps(fromModLine);

            auto backToLine = MUL(SETALL(fbAmt[i]), fromModLine);
            backToLine = DIV(MUL(backToLine, SQRT2), ADD(ONE, MUL(backToLine, backToLine)));
            backToLine = LPfilter.processSample(backToLine);
            backToLine = HPfilter.processSample(backToLine);
            backToLine = MUL(backToLine, POL);

            SIMD_M128 inputs = SETALL(datain[i]);
            lines->write(ADD(inputs, backToLine));
        }
        LPfilter.concludeBlock();
        HPfilter.concludeBlock();
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        voices.dispatch(lineSize(), [&](auto N) {
            auto *lines = voices.template getLinePointer<N>();
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
        voices.dispatch(lineSize(), [&](auto N) {
            auto *lines = voices.template getLinePointer<N>();
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
    delay::details::DelayLineSupport<sst::basic_blocks::dsp::quadDelayLine> voices;
    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> feedbackLerp;
    filtersplusplus::Filter LPfilter, HPfilter;

    static constexpr int numChords{6};
    struct alignas(16) Chord
    {
        Chord(float a, float b, float c, float d, const std::string &n)
        {
            ratios = SIMD_MM(set_ps)(d, c, b, a);
            name = n;
        }
        SIMD_M128 ratios;
        std::string name;
    };

    const std::array<Chord, numChords> CHORDS = {
        Chord(1, 5.f / 4, 3.f / 2, 15.f / 8, "∆"),       // 8:10:12:15 (∆)
        Chord(1, 5.f / 4, 3.f / 2, 7.f / 4, "7"),        // 4:5:6:7 (7)
        Chord(1, 6.f / 5, 3.f / 2, 9.f / 5, "m7"),       // 18:15:12:10 (minor7)
        Chord(1, 9.f / 8, 4.f / 3, 3.f / 2, "11"),       // 16:18:24:27 (11)
        Chord(1, 7.f / 6, 7.f / 5, 7.f / 4, "half-dim"), // 7:6:5:4 (half-dim)
        Chord(1, 75.f / 64, 45.f / 32, 5.f / 3, "dim"),  // 225:192:160:135 (dim)
    };
    // Many choices for a JI diminished chord...
    // I like that 5-limit one

    SIMD_M128 leftPans;
    SIMD_M128 rightPans;
    SIMD_M128 INVERSION;

    const SIMD_M128 ONE = SETALL(1.f);
    const SIMD_M128 NEGONE = SETALL(-1.f);
    const SIMD_M128 TWO = SETALL(2.f);
    const SIMD_M128 HALF = SETALL(.5f);
    const SIMD_M128 SQRT2 = SETALL(M_SQRT2);

    SIMD_M128 SampleRateSSE{};

    struct PanHelper
    {
        PanHelper(SineTable *sineTable) : sine(sineTable->table)
        {
            auto tS = static_cast<float>(sineTable->tableSize);
            auto tM = sineTable->tableSize - 1;
            LFO_TABLE_SIZE_SSE = SIMD_MM(set1_ps)(tS);
            LFO_TABLE_MASK_SSE = SIMD_MM(set1_epi32)(tM);
        }
        ~PanHelper() {}

        inline void newValues(float pan01, SIMD_M128 &panL, SIMD_M128 &panR)
        {
            namespace mech = sst::basic_blocks::mechanics;
            assert(sine != nullptr);
            assert(0 <= pan01 && pan01 <= 1);

            auto quadPan = ADD(SIMD_MM(set1_ps)(pan01), OFFSETS);
            quadPan = SUB(quadPan, SIMD_MM(floor_ps)(quadPan));
            quadPan = MUL(quadPan, halfSSE);

            auto sips = MUL(quadPan, LFO_TABLE_SIZE_SSE);
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
    PanHelper panHelper;

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
    quadLerp<VFXConfig::blockSize> timeLerp;

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
} // namespace sst::voice_effects::generator

#endif // INCLUDE_SST_VOICE_EFFECTS_GENERATOR_FOURVOICE_H
