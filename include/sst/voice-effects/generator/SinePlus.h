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

#include <cmath>

#include "../VoiceEffectCore.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig, bool forDisplay = false>
struct SinePlus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"Sine Plus"};
    static constexpr const char *streamingName{"osc-sineplus"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{3};

    enum FloatParams
    {
        fpBaseFrequency,
        fpOffsetA,
        fpOffsetB,
        fpMainBalance,
        fpOvertoneBalance,
        fpLevel
    };

    enum IntParams
    {
        ipQuantA,
        ipQuantB,
        ipStereo,
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
                    .withSemitoneFormatting()
                    .withName("Tune");
            }
            return pmd().asAudibleFrequency().withName("Frequency");
        case fpOffsetA:
            if (quantA)
            {
                return pmd()
                    .asFloat()
                    .withName("Tune A")
                    .withRange(2, 24)
                    .withDefault(2)
                    .withQuantizedStepCount(22)
                    .withDecimalPlaces(0)
                    .withUnitSeparator("")
                    .withLinearScaleFormatting("th Harmonic");
            }
            return pmd()
                .asFloat()
                .withName("Tune A")
                .withRange(12, 55.02)
                .withDefault(2)
                .withSemitoneFormatting();
        case fpOffsetB:
            if (quantB)
            {
                return pmd()
                    .asFloat()
                    .withName("Tune B")
                    .withRange(2, 24)
                    .withDefault(12)
                    .withQuantizedStepCount(22)
                    .withDecimalPlaces(0)
                    .withUnitSeparator("")
                    .withLinearScaleFormatting("th Harmonic");
            }
            return pmd()
                .asFloat()
                .withName("Tune B")
                .withRange(12, 55.02)
                .withDefault(12)
                .withSemitoneFormatting();
        case fpMainBalance:
            return pmd().asPercentBipolar().withDefault(-.5f).withName("Main Balance");
        case fpOvertoneBalance:
            return pmd().asPercentBipolar().withDefault(-.5f).withName("Overtone Balance");
        case fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(1.f).withName("Level");
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
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        // TODO: This is not actually a thousand milliseconds. Why?
        pitchLagA.setRateInMilliseconds(1000.f, this->getSampleRate(), 1.0 / VFXConfig::blockSize);
        pitchLagB.setRateInMilliseconds(1000.f, this->getSampleRate(), 1.0 / VFXConfig::blockSize);
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }
    void initVoiceEffectPitch(float pitch)
    {
        auto freq = this->getFloatParam(fpBaseFrequency) + pitch * keytrackOn;
        if (this->getIntParam(ipQuantA))
        {
            pitchLagA.snapTo(std::round(this->getFloatParam(fpOffsetA)));
        }
        else
        {
            pitchLagA.snapTo(this->getFloatParam(fpOffsetA));
        }

        if (this->getIntParam(ipQuantB))
        {
            pitchLagB.snapTo(std::round(this->getFloatParam(fpOffsetB)));
        }
        else
        {
            pitchLagB.snapTo(this->getFloatParam(fpOffsetB));
        }
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        if constexpr (forDisplay)
        {
            processForDisplay(dataout);
            return;
        }

        setFrequencies(pitch);

        namespace pan = basic_blocks::dsp::pan_laws;

        float mainLevel alignas(16)[VFXConfig::blockSize];
        float overtoneLevel alignas(16)[VFXConfig::blockSize];
        float aLevel alignas(16)[VFXConfig::blockSize];
        float bLevel alignas(16)[VFXConfig::blockSize];

        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;

        pan::stereoEqualPower((this->getFloatParam(fpMainBalance) + 1) * .5f, matrix);
        if constexpr (forDisplay)
        {
            // for display we draw at full amplitude and don't smooth
            mainLevel[0] = matrix[0];
            overtoneLevel[0] = matrix[1];
        }
        else
        {
            mainLerp.set_target(matrix[0] * levT);
            overtoneLerp.set_target(matrix[1] * levT);
            mainLerp.store_block(mainLevel);
            overtoneLerp.store_block(overtoneLevel);
        }

        pan::stereoEqualPower((this->getFloatParam(fpOvertoneBalance) + 1) * .5f, matrix);
        if constexpr (forDisplay)
        {
            aLevel[0] = matrix[0];
            bLevel[0] = matrix[1];
        }
        else
        {
            aLerp.set_target(matrix[0]);
            bLerp.set_target(matrix[1]);
            aLerp.store_block(aLevel);
            bLerp.store_block(bLevel);
        }

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto window = (-sineOscMain.u + 1) * 0.5f;
            window = window * window * window;

            float A, B, main, overtones;

            if constexpr (forDisplay)
            {
                A = sineOscA.v * aLevel[0] * window;
                B = sineOscB.v * bLevel[0] * window;
                main = sineOscMain.v * mainLevel[0];
                overtones = (A + B) * overtoneLevel[0];
            }
            else
            {
                A = sineOscA.v * aLevel[i] * window;
                B = sineOscB.v * bLevel[i] * window;
                main = sineOscMain.v * mainLevel[i];
                overtones = (A + B) * overtoneLevel[i];
            }

            dataout[i] = main + overtones;

            sineOscMain.blockStep();
            sineOscA.blockStep();
            sineOscB.blockStep();
            pitchLagA.process();
            pitchLagB.process();
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (!this->getIntParam(ipStereo))
        {
            processMonoToMono(datainL, dataoutL, pitch);
            basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
        }
        else
        {
            setFrequencies(pitch);

            namespace pan = basic_blocks::dsp::pan_laws;

            float mainLevel alignas(16)[VFXConfig::blockSize];
            float overtoneLevel alignas(16)[VFXConfig::blockSize];
            float aLevel alignas(16)[VFXConfig::blockSize];
            float bLevel alignas(16)[VFXConfig::blockSize];

            auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
            levT = levT * levT * levT;

            pan::stereoEqualPower((this->getFloatParam(fpMainBalance) + 1) * .5f, matrix);

            mainLerp.set_target(matrix[0] * levT);
            overtoneLerp.set_target(matrix[1] * levT);
            mainLerp.store_block(mainLevel);
            overtoneLerp.store_block(overtoneLevel);

            pan::stereoEqualPower((this->getFloatParam(fpOvertoneBalance) + 1) * .5f, matrix);

            aLerp.set_target(matrix[0]);
            bLerp.set_target(matrix[1]);
            aLerp.store_block(aLevel);
            bLerp.store_block(bLevel);

            for (int i = 0; i < VFXConfig::blockSize; i++)
            {
                auto window = (-sineOscMain.u + 1) * 0.5f;
                window = window * window * window;

                float A, B, main;

                A = sineOscA.v * aLevel[i] * window * overtoneLevel[i];
                B = sineOscB.v * bLevel[i] * window * overtoneLevel[i];
                main = sineOscMain.v * mainLevel[i];

                dataoutL[i] = main + A;
                dataoutR[i] = main + B;

                sineOscMain.blockStep();
                sineOscA.blockStep();
                sineOscB.blockStep();
                pitchLagA.process();
                pitchLagB.process();
            }
        }
    }

    void processMonoToStereo(const float *const datain, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datain, datain, dataoutL, dataoutR, pitch);
    }

    void setFrequencies(float pitch)
    {
        auto freq = this->getFloatParam(fpBaseFrequency) + pitch * keytrackOn;

        if (freq != priorMain)
        {
            sineOscMain.setRateForBlock(440.0 * 2 * M_PI *
                                        this->note_to_pitch_ignoring_tuning(freq) *
                                        this->getSampleRateInv());
            priorMain = freq;
        }
        else
        {
            sineOscMain.maintainRateForBlock();
        }

        float refA{440};
        float refB{440};
        float freqA{freq};
        float freqB{freq};

        if (this->getIntParam(ipQuantA))
        {
            auto newA = std::round(this->getFloatParam(fpOffsetA));
            if (newA != priorA)
            {
                pitchLagA.setTarget(newA);
                priorA = newA;
            }
            refA *= pitchLagA.getValue();
        }
        else
        {
            auto newA = this->getFloatParam(fpOffsetA);
            if (newA != priorA)
            {
                pitchLagA.setTarget(newA);
                priorA = newA;
            }
            freqA += pitchLagA.getValue();
        }

        if (this->getIntParam(ipQuantB))
        {
            auto newB = std::round(this->getFloatParam(fpOffsetB));
            if (newB != priorB)
            {
                pitchLagB.setTarget(newB);
                priorB = newB;
            }
            refB *= pitchLagB.getValue();
        }
        else
        {
            auto newB = this->getFloatParam(fpOffsetB);
            if (newB != priorB)
            {
                pitchLagB.setTarget(newB);
                priorB = newB;
            }
            freqB += pitchLagB.getValue();
        }

        sineOscA.setRateForBlock(refA * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqA) *
                                 this->getSampleRateInv());
        sineOscB.setRateForBlock(refB * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqB) *
                                 this->getSampleRateInv());
    }

    void resetPhase()
    {
        sineOscMain.resetPhase();
        sineOscA.resetPhase();
        sineOscB.resetPhase();
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }
    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{true};
    basic_blocks::dsp::QuadratureOscillator<float, VFXConfig::blockSize> sineOscMain, sineOscA,
        sineOscB;
    basic_blocks::dsp::LinearLag<float, false> pitchLagA, pitchLagB;
    float priorMain{-12354.6789}, priorA{-12354.6789}, priorB{-12354.6789};

    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> mainLerp, overtoneLerp, aLerp, bLerp;
    basic_blocks::dsp::pan_laws::panmatrix_t matrix{1, 1, 0, 0};

    void processForDisplay(float *out)
    {
        auto freq = this->getFloatParam(fpBaseFrequency);

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
        sineOscMain.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freq) *
                            this->getSampleRateInv());
        sineOscA.setRate(refA * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqA) *
                         this->getSampleRateInv());
        sineOscB.setRate(refB * 2 * M_PI * this->note_to_pitch_ignoring_tuning(freqB) *
                         this->getSampleRateInv());

        namespace pan = basic_blocks::dsp::pan_laws;

        // for display we draw at full amplitude and don't smooth
        pan::stereoEqualPower((this->getFloatParam(fpMainBalance) + 1) * .5f, matrix);
        float mainLevel = matrix[0];
        float overtoneLevel = matrix[1];

        pan::stereoEqualPower((this->getFloatParam(fpOvertoneBalance) + 1) * .5f, matrix);

        float aLevel = matrix[0];
        float bLevel = matrix[1];

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto window = (-sineOscMain.u + 1) * 0.5f;
            window = window * window * window;

            float A = sineOscA.v * aLevel * window;
            float B = sineOscB.v * bLevel * window;
            float main = sineOscMain.v * mainLevel;
            float overtones = (A + B) * overtoneLevel;

            out[i] = main + overtones;

            sineOscMain.step();
            sineOscA.step();
            sineOscB.step();
        }
    }

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
