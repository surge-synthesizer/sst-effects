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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_STRINGRESONATOR_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_STRINGRESONATOR_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/SSESincDelayLine.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"
#include "DelaySupport.h"

namespace sst::voice_effects::delay
{
template <typename VFXConfig> struct StringResonator : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"String Exciter"};

    static constexpr int numFloatParams{8};
    static constexpr int numIntParams{2};

    static constexpr float maxMiliseconds{100.f}; // 10 hz floor

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    static constexpr int shortLineSize{14};
    static constexpr int longLineSize{16};

    enum FloatParams
    {
        fpLevelOne,
        fpLevelTwo,
        fpOffsetOne,
        fpOffsetTwo,
        fpPanOne,
        fpPanTwo,
        fpDecay,
        fpStiffness,
    };

    enum IntParams
    {
        ipStereo,
        ipDualString,
    };

    StringResonator(const SincTable &st)
        : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>(), lp(this), hp(this)
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~StringResonator()
    {
        if (isShort)
        {
            lineSupport[0].template returnLines<shortLineSize>(this);
            lineSupport[1].template returnLines<shortLineSize>(this);
        }
        else
        {
            lineSupport[0].template returnLines<longLineSize>(this);
            lineSupport[1].template returnLines<longLineSize>(this);
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool dual = this->getIntParam(ipDualString) > 0;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case fpLevelOne:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(dual ? .85f : 1.f)
                .withLinearScaleFormatting("%", 100.f)
                .withDecimalPlaces(2)
                .withName(std::string("Level") + (dual ? " One" : ""));
        case fpLevelTwo:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(.85f)
                .withLinearScaleFormatting("%", 100.f)
                .withDecimalPlaces(2)
                .withName(!dual ? std::string() : "Level Two");
        case fpOffsetOne:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones")
                    .withName(std::string("Offset") + (dual ? " One" : ""));
            }
            return pmd().asAudibleFrequency().withName(std::string("Frequency") +
                                                       (dual ? " One" : ""));
        case fpOffsetTwo:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones")
                    .withName(!dual ? std::string() : "Offset Two");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(!dual ? std::string() : "Frequency Two")
                .withDefault(0);
        case fpPanOne:
            return pmd()
                .asPercentBipolar()
                .withCustomMinDisplay("L")
                .withCustomMaxDisplay("R")
                .withCustomDefaultDisplay("C")
                .withDefault(dual ? -1.f : 0.f)
                .withName(!stereo ? std::string() : (std::string("Pan") + (dual ? " One" : "")));
        case fpPanTwo:
            return pmd()
                .asPercentBipolar()
                .withCustomMinDisplay("L")
                .withCustomMaxDisplay("R")
                .withCustomDefaultDisplay("C")
                .withDefault(1.f)
                .withName((dual && stereo) ? "Pan Two" : std::string());
        case fpDecay:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(0.8f)
                .withLinearScaleFormatting("")
                .withName("Decay");
        case fpStiffness:
            return pmd().asPercentBipolar().withDefault(0.f).withName("Stiffness");
        }
        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(true);
        case ipDualString:
            return pmd().asOnOffBool().withDefault(true).withName("Dual String");
        }
        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        if (this->getSampleRate() * 0.1 > (1 << 14))
        {
            isShort = false;
            for (int i = 0; i < 2; ++i)
            {
                lineSupport[i].template preReserveLines<longLineSize>(this);
                lineSupport[i].template prepareLine<longLineSize>(this, sSincTable);
            }
        }
        else
        {
            isShort = true;
            for (int i = 0; i < 2; ++i)
            {
                lineSupport[i].template preReserveLines<shortLineSize>(this);
                lineSupport[i].template prepareLine<shortLineSize>(this, sSincTable);
            }
        }
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float equalPowerFormula(float theta)
    {
        return float(theta +
                     (theta * theta * theta) * (-0.166666667f + theta * theta * 0.00833333333f)) *
               1.414213562f;
    }

    void balancedMonoSum(float pan, float leftIn, float rightIn, float &sum)
    {
        if (pan == 0.5f)
        {
            sum = leftIn + rightIn;
        }
        else if (pan == 0)
        {
            sum = leftIn;
        }
        else if (pan == 1)
        {
            sum = rightIn;
        }
        else
        {
            float rTheta = pan * M_PI_2;
            float lTheta = M_PI_2 - rTheta;
            float leftVolume = leftIn * equalPowerFormula(lTheta);
            float rightVolume = rightIn * equalPowerFormula(rTheta);
            sum = leftVolume + rightVolume;
        }
    }

    void panLineToOutput(float pan, float monoIn, float &outL, float &outR)
    {
        float rTheta = pan * M_PI_2;
        float lTheta = M_PI_2 - rTheta;
        outL = monoIn * equalPowerFormula(lTheta);
        outR = monoIn * equalPowerFormula(rTheta);
    }

    template <typename T>
    void stereoDualString(const std::array<T *, 2> &lines, const float *const datainL,
                          const float *const datainR, float *dataoutL, float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        auto levelParamOne = std::clamp(this->getFloatParam(fpLevelOne), 0.f, 1.f);
        auto levelParamTwo = std::clamp(this->getFloatParam(fpLevelTwo), 0.f, 1.f);
        levelLerpOne.set_target(levelParamOne);
        levelLerpTwo.set_target(levelParamTwo);

        auto panParamOne = std::clamp((this->getFloatParam(fpPanOne) * .5f + .5f), 0.f, 1.f);
        auto panParamTwo = std::clamp((this->getFloatParam(fpPanTwo) * .5f + .5f), 0.f, 1.f);
        if (!this->getIntParam(ipStereo))
        {
            panParamOne = 0.5f;
            panParamTwo = 0.5f;
        }
        panLerpOne.set_target(panParamOne);
        panLerpTwo.set_target(panParamTwo);

        auto pitchParamOne = this->getFloatParam(fpOffsetOne);
        auto pitchParamTwo = this->getFloatParam(fpOffsetTwo);
        if (keytrackOn)
        {
            pitchParamOne += pitch;
            pitchParamTwo += pitch;
        }
        pitchParamOne += pitchAdjustmentForStiffness();
        pitchParamTwo += pitchAdjustmentForStiffness();
        setupFilters(pitchParamOne);
        setupFilters(pitchParamTwo);
        pitchLerpOne.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParamOne)));
        pitchLerpTwo.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParamTwo)));

        auto decayParam = std::clamp(this->getFloatParam(fpDecay), 0.f, 1.f) * 0.12 + 0.88;
        decayParam = std::min(sqrt(decayParam), 0.99999);
        decayLerp.set_target(decayParam);

        if (firstPitch)
        {
            pitchLerpOne.instantize();
            pitchLerpTwo.instantize();
            decayLerp.instantize();
            panLerpOne.instantize();
            panLerpTwo.instantize();
            firstPitch = false;
        }

        float levelOne alignas(16)[VFXConfig::blockSize];
        float levelTwo alignas(16)[VFXConfig::blockSize];
        float panOne alignas(16)[VFXConfig::blockSize];
        float panTwo alignas(16)[VFXConfig::blockSize];
        float frequencyOne alignas(16)[VFXConfig::blockSize];
        float frequencyTwo alignas(16)[VFXConfig::blockSize];
        float feedback alignas(16)[VFXConfig::blockSize];

        levelLerpOne.store_block(levelOne);
        levelLerpTwo.store_block(levelTwo);
        panLerpOne.store_block(panOne);
        panLerpTwo.store_block(panTwo);
        pitchLerpOne.store_block(frequencyOne);
        pitchLerpTwo.store_block(frequencyTwo);
        decayLerp.store_block(feedback);

        float tone = this->getFloatParam(fpStiffness);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromLineOne = lines[0]->read(frequencyOne[i]);
            auto fromLineTwo = lines[1]->read(frequencyTwo[i]);

            float toLineOne = 0.f;
            float toLineTwo = 0.f;

            balancedMonoSum(panOne[i], datainL[i], datainR[i], toLineOne);
            balancedMonoSum(panTwo[i], datainL[i], datainR[i], toLineTwo);

            toLineOne = toLineOne + feedback[i] * fromLineOne;
            toLineTwo = toLineTwo + feedback[i] * fromLineTwo;

            if (tone < 0)
            {
                lp.process_sample(toLineOne, toLineTwo, toLineOne, toLineTwo);
            }
            else if (tone > 0)
            {
                hp.process_sample(toLineOne, toLineTwo, toLineOne, toLineTwo);
            }

            lines[0]->write(toLineOne);
            lines[1]->write(toLineTwo);

            float leftOutOne = 0.f, rightOutOne = 0.f;
            float leftOutTwo = 0.f, rightOutTwo = 0.f;

            panLineToOutput(panOne[i], toLineOne, leftOutOne, rightOutOne);
            panLineToOutput(panTwo[i], toLineTwo, leftOutTwo, rightOutTwo);

            leftOutOne *= levelOne[i];
            rightOutOne *= levelOne[i];
            leftOutTwo *= levelTwo[i];
            rightOutTwo *= levelTwo[i];

            dataoutL[i] = leftOutOne + leftOutTwo;
            dataoutR[i] = rightOutOne + rightOutTwo;
        }
    }

    template <typename T>
    void stereoSingleString(T *line, const float *const datainL, const float *const datainR,
                            float *dataoutL, float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        auto levelParam = std::clamp(this->getFloatParam(fpLevelOne), 0.f, 1.f);
        levelLerpOne.set_target(levelParam);

        auto panParam = std::clamp((this->getFloatParam(fpPanOne) * .5f + .5f), 0.f, 1.f);
        if (!this->getIntParam(ipStereo))
        {
            panParam = 0.5f;
        }
        panLerpOne.set_target(panParam);

        auto pitchParam = this->getFloatParam(fpOffsetOne);
        if (keytrackOn)
        {
            pitchParam += pitch;
        }
        pitchParam += pitchAdjustmentForStiffness();
        setupFilters(pitchParam);
        pitchLerpOne.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParam)));

        auto decayParam = std::clamp(this->getFloatParam(fpDecay), 0.f, 1.f) * 0.12 + 0.88;
        decayParam = std::min(sqrt(decayParam), 0.99999);
        decayLerp.set_target(decayParam);

        if (firstPitch)
        {
            pitchLerpOne.instantize();
            decayLerp.instantize();
            panLerpOne.instantize();
            firstPitch = false;
        }

        float level alignas(16)[VFXConfig::blockSize];
        float pan alignas(16)[VFXConfig::blockSize];
        float frequency alignas(16)[VFXConfig::blockSize];
        float feedback alignas(16)[VFXConfig::blockSize];

        levelLerpOne.store_block(level);
        panLerpOne.store_block(pan);
        pitchLerpOne.store_block(frequency);
        decayLerp.store_block(feedback);

        float tone = this->getFloatParam(fpStiffness);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromLine = line->read(frequency[i]);

            float toLine = datainL[i];

            balancedMonoSum(pan[i], datainL[i], datainR[i], toLine);

            toLine = toLine + feedback[i] * fromLine;

            float dummyR = 0.f;

            if (tone < 0)
            {
                lp.process_sample(toLine, dummyR, toLine, dummyR);
            }
            else if (tone > 0)
            {
                hp.process_sample(toLine, dummyR, toLine, dummyR);
            }

            line->write(toLine);

            float leftOut = 0.f, rightOut = 0.f;

            panLineToOutput(pan[i], toLine, leftOut, rightOut);

            dataoutL[i] = leftOut * level[i];
            dataoutR[i] = rightOut * level[i];
        }
    }

    template <typename T>
    void monoDualString(const std::array<T *, 2> &lines, const float *const datainL,
                        float *dataoutL, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        auto levelParamOne = std::clamp(this->getFloatParam(fpLevelOne), 0.f, 1.f);
        auto levelParamTwo = std::clamp(this->getFloatParam(fpLevelTwo), 0.f, 1.f);
        levelLerpOne.set_target(levelParamOne);
        levelLerpTwo.set_target(levelParamTwo);

        auto pitchParamOne = this->getFloatParam(fpOffsetOne);
        auto pitchParamTwo = this->getFloatParam(fpOffsetTwo);
        if (keytrackOn)
        {
            pitchParamOne += pitch;
            pitchParamTwo += pitch;
        }
        pitchParamOne += pitchAdjustmentForStiffness();
        pitchParamTwo += pitchAdjustmentForStiffness();
        setupFilters(pitchParamOne);
        setupFilters(pitchParamTwo);

        pitchLerpOne.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParamOne)));
        pitchLerpTwo.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParamTwo)));

        auto decayParam = std::clamp(this->getFloatParam(fpDecay), 0.f, 1.f) * 0.12 + 0.88;
        decayParam = std::min(sqrt(decayParam), 0.99999);
        decayLerp.set_target(decayParam);
        if (firstPitch)
        {
            pitchLerpOne.instantize();
            pitchLerpTwo.instantize();
            decayLerp.instantize();
            firstPitch = false;
        }

        float levelOne alignas(16)[VFXConfig::blockSize];
        float levelTwo alignas(16)[VFXConfig::blockSize];
        float frequencyOne alignas(16)[VFXConfig::blockSize];
        float frequencyTwo alignas(16)[VFXConfig::blockSize];
        float feedback alignas(16)[VFXConfig::blockSize];

        levelLerpOne.store_block(levelOne);
        levelLerpTwo.store_block(levelTwo);
        pitchLerpOne.store_block(frequencyOne);
        pitchLerpTwo.store_block(frequencyTwo);
        decayLerp.store_block(feedback);

        float tone = this->getFloatParam(fpStiffness);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromLineOne = lines[0]->read(frequencyOne[i]);
            auto fromLineTwo = lines[1]->read(frequencyTwo[i]);

            float toLineOne = 0.f;
            float toLineTwo = 0.f;

            toLineOne = datainL[i] + feedback[i] * fromLineOne;
            toLineTwo = datainL[i] + feedback[i] * fromLineTwo;

            if (tone < 0)
            {
                lp.process_sample(toLineOne, toLineTwo, toLineOne, toLineTwo);
            }
            else if (tone > 0)
            {
                hp.process_sample(toLineOne, toLineTwo, toLineOne, toLineTwo);
            }

            lines[0]->write(toLineOne);
            lines[1]->write(toLineTwo);

            dataoutL[i] = toLineOne * levelOne[i] + toLineTwo * levelTwo[i];
        }
    }

    template <typename T>
    void monoSingleString(T *line, const float *const datainL, float *dataoutL, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        auto levelParam = std::clamp(this->getFloatParam(fpLevelOne), 0.f, 1.f);
        levelLerpOne.set_target(levelParam);

        auto pitchParam = this->getFloatParam(fpOffsetOne);
        if (keytrackOn)
        {
            pitchParam += pitch;
        }
        pitchParam += pitchAdjustmentForStiffness();
        setupFilters(pitchParam);

        pitchLerpOne.set_target(this->getSampleRate() /
                                (440 * this->note_to_pitch_ignoring_tuning(pitchParam)));

        auto decayParam = std::clamp(this->getFloatParam(fpDecay), 0.f, 1.f) * 0.12 + 0.88;
        decayParam = std::min(sqrt(decayParam), 0.99999);
        decayLerp.set_target(decayParam);
        if (firstPitch)
        {
            pitchLerpOne.instantize();
            decayLerp.instantize();
            firstPitch = false;
        }

        float level alignas(16)[VFXConfig::blockSize];
        float frequency alignas(16)[VFXConfig::blockSize];
        float feedback alignas(16)[VFXConfig::blockSize];

        levelLerpOne.store_block(level);
        pitchLerpOne.store_block(frequency);
        decayLerp.store_block(feedback);

        float tone = this->getFloatParam(fpStiffness);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto fromLine = line->read(frequency[i]);

            float toLine = datainL[i];

            toLine = toLine + feedback[i] * fromLine;

            float dummyR = 0.f;

            if (tone < 0)
            {
                lp.process_sample(toLine, dummyR, toLine, dummyR);
            }
            else if (tone > 0)
            {
                hp.process_sample(toLine, dummyR, toLine, dummyR);
            }

            line->write(toLine);

            dataoutL[i] = toLine * level[i];
        }
    }

    void setupFilters(float pitch)
    {
        // This is the surge corrected tuning tone control
        pitch += 69;
        float clo = 10, cmidhi = 60, cmid = 100, chi = -70;
        float hpCutoff = chi;
        float lpCutoff = cmid;
        // If you change this you also need to recallibrate the tuning corrections!
        auto tone = this->getFloatParam(fpStiffness);
        if (tone > 0)
        {
            // We want a smaller range but chi stays the same so cmidhi - chi = 60 + 70 = 130
            // and we want about 60% to knock us out so
            cmidhi = 10, chi = -70;
            // OK so cool scale the HP cutoff
            auto tv = tone;
            hpCutoff = tv * (cmidhi - chi) + chi + pitch - 60;
        }
        else
        {
            auto tv = -tone;
            lpCutoff = pitch - 40 * tv;
        }
        // Inefficient - copy coefficients later
        lp.coeff_LP(lp.calc_omega((lpCutoff / 12.0)), 0.707);
        hp.coeff_HP(hp.calc_omega((hpCutoff / 12.0)), 0.707);
    }

    float pitchAdjustmentForStiffness()
    {

        auto tv = this->getFloatParam(fpStiffness);
        if (tv == 0)
            return 0.f;

        if (tv < 0)
        {
            // I just whacked at it in a tuner at levels and came up with this. These are pitch
            // shifts so basically i ran A/69/440 into a tuner with the burst chirp and saw how far
            // we were off in frequency at 0, 25, 50 etc... then converted to notes using 12TET
            static constexpr float retunes[] = {-0.0591202, -0.122405, -0.225738, -0.406056,
                                                -0.7590243};
            float fidx = std::clamp(-4 * tv, 0.f, 4.f);
            int idx = std::clamp((int)fidx, 0, 3);
            float frac = fidx - idx;

            auto res = retunes[idx] * (1 - frac) + retunes[idx + 1] * frac;

            return -res;
        }
        else
        {
            // I just whacked at it in a tuner at levels and came up with this
            static constexpr float retunes[] = {0.02752047, 0.09026062, 0.31, 0.615, 0.87};
            float fidx = std::clamp(4 * tv, 0.f, 4.f);
            int idx = std::clamp((int)fidx, 0, 3);
            float frac = fidx - idx;

            auto res = retunes[idx] * (1 - frac) + retunes[idx + 1] * frac;

            return -res;
        }
        return 0;
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipDualString))
        {
            if (isShort)
            {
                stereoDualString(
                    std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                               lineSupport[1].template getLinePointer<shortLineSize>()},
                    datainL, datainR, dataoutL, dataoutR, pitch);
            }
            else
            {
                stereoDualString(std::array{lineSupport[0].template getLinePointer<longLineSize>(),
                                            lineSupport[1].template getLinePointer<longLineSize>()},
                                 datainL, datainR, dataoutL, dataoutR, pitch);
            }
        }
        else
        {
            if (isShort)
            {
                stereoSingleString(lineSupport[0].template getLinePointer<shortLineSize>(), datainL,
                                   datainR, dataoutL, dataoutR, pitch);
            }
            else
            {
                stereoSingleString(lineSupport[0].template getLinePointer<shortLineSize>(), datainL,
                                   datainR, dataoutL, dataoutR, pitch);
            }
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        if (this->getIntParam(ipDualString))
        {
            if (isShort)
            {
                monoDualString(std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                                          lineSupport[1].template getLinePointer<shortLineSize>()},
                               datainL, dataoutL, pitch);
            }
            else
            {
                monoDualString(std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                                          lineSupport[1].template getLinePointer<shortLineSize>()},
                               datainL, dataoutL, pitch);
            }
        }
        else
        {
            if (isShort)
            {
                monoSingleString(lineSupport[0].template getLinePointer<shortLineSize>(), datainL,
                                 dataoutL, pitch);
            }
            else
            {
                monoSingleString(lineSupport[0].template getLinePointer<longLineSize>(), datainL,
                                 dataoutL, pitch);
            }
        }
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool checkParameterConsistency() const { return true; }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }

  protected:
    bool keytrackOn{true};
    std::array<details::DelayLineSupport, 2> lineSupport;
    bool isShort{true};
    bool firstPitch{false};

    std::array<float, numFloatParams> mLastParam{};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> levelLerpOne, levelLerpTwo,
        panLerpOne, panLerpTwo, pitchLerpOne, pitchLerpTwo, decayLerp;

    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType lp, hp;

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

} // namespace sst::voice_effects::delay

#endif // SHORTCIRCUITXT_EqNBandParametric_H
