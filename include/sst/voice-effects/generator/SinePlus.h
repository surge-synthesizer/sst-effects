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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_SINEPLUS_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_SINEPLUS_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/basic-blocks/dsp/PanLaws.h"

#include "../VoiceEffectCore.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct SinePlus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Sine Plus"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    enum FloatParams
    {
        fpBaseFrequency,
        fpOffsetA,
        fpOffsetB,
        fpMainBalance,
        fpBalanceAB,
    };

    enum IntParams
    {
        ipQuantA,
        ipQuantB,
    };

    SinePlus() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~SinePlus() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        bool quantA = this->getIntParam(ipQuantA);
        bool quantB = this->getIntParam(ipQuantB);

        switch (idx)
        {
        case fpBaseFrequency:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones")
                    .withName("Tune");
            }
            return pmd().asAudibleFrequency().withName("Frequency");
        case fpOffsetA:
            if (quantA)
            {
                return pmd()
                    .asFloat()
                    .withName("Offset A")
                    .withRange(2, 24)
                    .withDefault(2)
                    .withQuantizedStepCount(22)
                    .withLinearScaleFormatting("Harmonic");
            }
            return pmd()
                .asFloat()
                .withName("Offset A")
                .withRange(12, 44)
                .withDefault(12)
                .withLinearScaleFormatting("semitones");
        case fpOffsetB:
            if (quantB)
            {
                return pmd()
                    .asFloat()
                    .withName("Offset B")
                    .withRange(2, 24)
                    .withDefault(3)
                    .withQuantizedStepCount(22)
                    .withLinearScaleFormatting("Harmonic");
            }
            return pmd()
                .asFloat()
                .withName("Offset B")
                .withRange(12, 44)
                .withDefault(19)
                .withLinearScaleFormatting("semitones");
        case fpMainBalance:
            return pmd().asPercentBipolar().withName("Main Balance");
        case fpBalanceAB:
            return pmd().asPercentBipolar().withName("Balance AB");
        }

        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipQuantA:
            return pmd()
                .asBool()
                .withUnorderedMapFormatting({{false, "Off"}, {true, "On"}})
                .withDefault(true)
                .withName("Harmonic Quantize A");
        case ipQuantB:
            return pmd()
                .asBool()
                .withUnorderedMapFormatting({{false, "Off"}, {true, "On"}})
                .withDefault(true)
                .withName("Harmonic Quantize B");
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        processMonoToMono(datainL, dataoutL, pitch);
        basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        auto freq = this->getFloatParam(fpBaseFrequency);
        if (keytrackOn)
            freq += pitch;

        float refA{440};
        float refB{440};
        float freqA{freq};
        float freqB{freq};

        if (this->getIntParam(ipQuantA))
        {
            refA *= std::round(this->getFloatParam(fpOffsetA));
        }
        else
        {
            freqA += this->getFloatParam(fpOffsetA);
        }

        if (this->getIntParam(ipQuantB))
        {
            refB *= std::round(this->getFloatParam(fpOffsetB));
        }
        else
        {
            freqB += this->getFloatParam(fpOffsetB);
        }

        sineOsc1.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freq) *
                         this->getSampleRateInv());

        sineOsc2.setRate(refA * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqA) *
                         this->getSampleRateInv());

        sineOsc3.setRate(refB * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqB) *
                         this->getSampleRateInv());

        namespace pan = basic_blocks::dsp::pan_laws;

        pan::stereoEqualPower((this->getFloatParam(fpMainBalance) + 1) * .5f, matrix);
        mainLerp.set_target(matrix[0]);
        overtoneLerp.set_target(matrix[1]);
        float mainLevel alignas(16)[VFXConfig::blockSize];
        float overtoneLevel alignas(16)[VFXConfig::blockSize];
        mainLerp.store_block(mainLevel);
        overtoneLerp.store_block(overtoneLevel);

        pan::stereoEqualPower((this->getFloatParam(fpBalanceAB) + 1) * .5f, matrix);
        aLerp.set_target(matrix[0]);
        bLerp.set_target(matrix[1]);
        float aLevel alignas(16)[VFXConfig::blockSize];
        float bLevel alignas(16)[VFXConfig::blockSize];
        aLerp.store_block(aLevel);
        bLerp.store_block(bLevel);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto window = (sineOsc1.u + 1) * -0.5f;
            window = window * window * window;

            auto A = sineOsc2.v * aLevel[i] * window;
            auto B = sineOsc3.v * bLevel[i] * window;

            auto main = sineOsc1.v * mainLevel[i];

            auto overtones = (A + B) * overtoneLevel[i];

            dataout[i] = main + overtones;

            sineOsc1.step();
            sineOsc2.step();
            sineOsc3.step();
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

  protected:
    bool keytrackOn{true};
    basic_blocks::dsp::QuadratureOscillator<> sineOsc1, sineOsc2, sineOsc3;

    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> mainLerp, overtoneLerp, aLerp, bLerp;
    basic_blocks::dsp::pan_laws::panmatrix_t matrix{1, 1, 0, 0};

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

} // namespace sst::voice_effects::generator
#endif
