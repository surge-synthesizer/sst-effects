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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_FMFILTER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_FMFILTER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <ratio>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct FMFilter : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"FM Filter"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{4};

    enum FloatParams
    {
        fpFreqL,
        fpFreqR,
        fpDepth,
        fpRes,
    };

    enum IntParams
    {
        ipStereo,
        ipMode,
        ipNum,
        ipDenom,
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case fpFreqL:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName(std::string("Offset") + (stereo ? " L" : ""))
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(std::string("Frequency") + (stereo ? " L" : ""))
                .withDefault(0);
        case fpFreqR:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName(!stereo ? std::string() : "Offset R")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName(!stereo ? std::string() : "Frequency R");
        case fpDepth:
            return pmd().asPercent().withRange(0.f, 1.f).withDefault(0.f).withName("FM Depth");
        case fpRes:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(0.7f)
                .withName("Resonance")
                .withDimensionlessFormatting();
        }
        return pmd().withName("error ");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        using md = sst::filters::CytomicSVF::Mode;

        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);

        case ipMode:
            return pmd()
                .asInt()
                .withRange(0, 4)
                .withName("Mode")
                .withUnorderedMapFormatting({
                    {(int)md::Lowpass, "Lowpass"},
                    {(int)md::Highpass, "Highpass"},
                    {(int)md::Bandpass, "Bandpass"},
                    {(int)md::Notch, "Notch"},
                    {(int)md::Allpass, "Allpass"},
                })
                .withDefault((int)md::Lowpass);
        case ipNum:
            return pmd()
                .asInt()
                .withRange(1, 16)
                .withDefault(1)
                .withName("Numerator")
                .withDimensionlessFormatting();
        case ipDenom:
            return pmd()
                .asInt()
                .withRange(1, 16)
                .withDefault(1)
                .withName("Denominator")
                .withDimensionlessFormatting();
        }
        return pmd().withName("error");
    }

    FMFilter() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~FMFilter() {}

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float getRatio(int num, int denom)
    {
        if (num == denom)
        {
            return 0.f;
        }

        auto ratio = static_cast<float>(num) / static_cast<float>(denom);
        return 12 * std::log2(ratio);
    }

    void whatMode()
    {
        switch (this->getIntParam(ipMode))
        {
        case 0:
            mode = sst::filters::CytomicSVF::Mode::Lowpass;
            break;
        case 1:
            mode = sst::filters::CytomicSVF::Mode::Highpass;
            break;
        case 2:
            mode = sst::filters::CytomicSVF::Mode::Bandpass;
            break;
        case 3:
            mode = sst::filters::CytomicSVF::Mode::Notch;
            break;
        case 4:
            mode = sst::filters::CytomicSVF::Mode::Allpass;
            break;
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (isFirst)
        {
            DCfilter.template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::Highpass, 18.f, 18.f, 0.5f, 0.5f,
                VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            isFirst = false;
        }
        else
        {
            DCfilter.retainCoeffForBlock<VFXConfig::blockSize>();
        }
        auto res = std::clamp(this->getFloatParam(fpRes), 0.f, 1.f);
        bool stereo = this->getIntParam(ipStereo);
        auto depth = this->getFloatParam(fpDepth);

        auto num = (this->getIntParam(ipNum));
        auto denom = (this->getIntParam(ipDenom));
        if (num != priorNum || denom != priorDenom)
        {
            ratio = getRatio(num, denom);
            priorNum = num;
            priorDenom = denom;
        }

        auto freqL = this->getFloatParam(fpFreqL);
        auto freqR = this->getFloatParam((stereo) ? fpFreqR : fpFreqL);
        if (keytrackOn)
        {
            freqL += pitch;
            freqR += pitch;
        }
        freqL = 440.0f * this->note_to_pitch_ignoring_tuning(freqL);
        freqR = 440.0f * this->note_to_pitch_ignoring_tuning(freqR);

        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pitch + ratio) *
                        this->getSampleRateInv());

        auto outputL = 0.f;
        auto outputR = 0.f;

        if (this->getIntParam(ipMode) != priorMode)
        {
            whatMode();
            priorMode = this->getIntParam(ipMode);
        }
        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float inL = datainL[i];
            float inR = datainR[i];

            mSinOsc.step();

            float modL = mSinOsc.v * freqL * 3 * depth;
            float modR = mSinOsc.v * freqR * 3 * depth;
            auto modFreqL = freqL + modL;
            auto modFreqR = freqR + modR;

            filter.setCoeff(mode, modFreqL, modFreqR, res, res, VFXConfig::getSampleRateInv(this),
                            0.f, 0.f);
            sst::filters::CytomicSVF::step(filter, inL, inR);

            DCfilter.processBlockStep(inL, inR);

            dataoutL[i] = inL;
            dataoutR[i] = inR;
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        if (isFirst)
        {
            DCfilter.template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::Highpass, 18.f, 0.5f,
                VFXConfig::getSampleRateInv(this), 0.f);
            isFirst = false;
        }
        else
        {
            DCfilter.retainCoeffForBlock<VFXConfig::blockSize>();
        }
        auto res = std::clamp(this->getFloatParam(fpRes), 0.f, 1.f);
        auto depth = std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f);

        auto num = (this->getIntParam(ipNum));
        auto denom = (this->getIntParam(ipDenom));
        if (num != priorNum || denom != priorDenom)
        {
            ratio = getRatio(num, denom);
            priorNum = num;
            priorDenom = denom;
        }

        auto freq = this->getFloatParam(fpFreqL);
        if (keytrackOn)
        {
            freq += pitch;
        }
        freq = 440.0f * this->note_to_pitch_ignoring_tuning(freq);

        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pitch + ratio) *
                        this->getSampleRateInv());

        if (this->getIntParam(ipMode) != priorMode)
        {
            whatMode();
            priorMode = this->getIntParam(ipMode);
        }
        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float input = datainL[i];

            mSinOsc.step();

            float mod = mSinOsc.v * freq * 3 * depth;
            auto modFreq = mod + freq;

            filter.setCoeff(mode, modFreq, res, VFXConfig::getSampleRateInv(this), 0.f);
            auto dummyR = 0.f;
            sst::filters::CytomicSVF::step(filter, input, dummyR);

            DCfilter.processBlockStep(input);

            dataoutL[i] = input;
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

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
    sst::filters::CytomicSVF filter;
    sst::filters::CytomicSVF DCfilter;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOsc;
    bool isFirst = true;

    sst::filters::CytomicSVF::Mode mode{filters::CytomicSVF::Mode::Lowpass};
    int priorMode{-1};
    int priorNum{-1};
    int priorDenom{-1};
    float ratio{-1.f};

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
} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_FMFILTER_H
