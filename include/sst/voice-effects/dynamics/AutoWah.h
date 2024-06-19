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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DYNAMICS_AUTOWAH_H
#define INCLUDE_SST_VOICE_EFFECTS_DYNAMICS_AUTOWAH_H

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FollowSlewAndSmooth.h"

namespace sst::voice_effects::dynamics
{
template <typename VFXConfig> struct AutoWah : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Auto Wah"};

    static constexpr size_t rmsBufferSize{1024}; // TODO: SR invariance...
    float *rmsBlock{nullptr};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{1};

    enum FloatParams
    {
        fpSens,
        fpDepth,
        fpSpeed,
        fpCenterFreq,
        fpRes
    };

    enum IntParams
    {
        ipMode
    };

    AutoWah() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        this->preReservePool(rmsBufferSize * sizeof(float));
    }

    ~AutoWah()
    {
        if (rmsBlock)
        {
            VFXConfig::returnBlock(this, (uint8_t *)rmsBlock, rmsBufferSize * sizeof(float));
            rmsBlock = nullptr;
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpSens:
            return pmd().asFloat().withRange(0.f, 1.f).withDefault(.5f).withName("Sensitivity");
        case fpDepth:
            return pmd().asFloat().withRange(0.f, 1.f).withDefault(.3f).withName("Depth");
        case fpSpeed:
            return pmd()
                .asFloat()
                .withRange(0.015f, .3f)
                .withDefault(0.1f)
                .withDecimalPlaces(3)
                .withLinearScaleFormatting("ms", 1000.f)
                .withName("Speed");
        case fpCenterFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("Freq Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Center Freq");
        case fpRes:
            return pmd().asFloat().withRange(0.f, 1.f).withDefault(0.7f).withName("Resonance");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd()
            .asBool()
            .withDefault(false)
            .withUnorderedMapFormatting({{false, "LP"}, {true, "BP"}})
            .withName("Mode");
    }

    void initVoiceEffect()
    {
        if (!rmsBlock)
        {
            auto block = VFXConfig::checkoutBlock(this, rmsBufferSize * sizeof(float));
            memset(block, 0, rmsBufferSize * sizeof(float));
            rmsBlock = (float *)block;
            RA.setStorage(rmsBlock, rmsBufferSize);
        }
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float decibelsToAmplitude(float db) { return powf(10.0f, db * 0.05f); }

    float amplitudeToDecibels(float amplitude)
    {
        if (amplitude < 0.000001f)
        {
            return -120.0f;
        }
        return 20.0f * log10f(amplitude);
    }

    void saturateNext(float &L, float &R)
    {
        L = std::clamp(L, -1.5f, 1.5f);
        L = L - 4.0 / 27.0 * L * L * L;

        R = std::clamp(R, -1.5f, 1.5f);
        R = R - 4.0 / 27.0 * R * R * R;
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto sens = 1 + -1 * this->getFloatParam(fpSens);
        sens *= sens;
        auto thresholdDecibel = amplitudeToDecibels(sens);
        float envDecibel = 0.f;

        float speed = this->getFloatParam(fpSpeed) * 1000.f;
        float samplerate = this->getSampleRate();
        speedLimiter.setParams(speed, 1.f, samplerate);

        bool modeSwitch = this->getIntParam(ipMode);
        sst::filters::CytomicSVF::Mode mode = sst::filters::CytomicSVF::Mode::LP;
        if (modeSwitch)
        {
            mode = sst::filters::CytomicSVF::Mode::BP;
        }
        auto centerFreqParam = this->getFloatParam(fpCenterFreq);
        auto centerFreq = (keytrackOn) ? centerFreqParam + pitch : centerFreqParam;
        auto res = this->getFloatParam(fpRes);
        res = (res / 2) + .5f;

        if (first)
        {
            RA.reset();
            speedLimiter.reset();
            filter[0].init();
            filter[1].init();
            DCfilter.template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::HP, 10.f, 0.5f, VFXConfig::getSampleRateInv(this),
                0.f);
            first = false;
        }
        else
        {
            DCfilter.template retainCoeffForBlock<VFXConfig::blockSize>();
        }

        auto depth = this->getFloatParam(fpDepth);
        auto modFreq = 0.f;
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            float inL = datainL[i];
            float inR = datainR[i];
            modFreq = centerFreq;

            float env = RA.step(fabsf(inL + inR));

            env = speedLimiter.step(env);

            envDecibel = amplitudeToDecibels(env);
            if (envDecibel > thresholdDecibel)
            {
                auto over = envDecibel - thresholdDecibel;
                auto modAmount = over * this->getFloatParam(fpDepth) * 5;
                modFreq += modAmount;
            }
            modFreq = 440.f * this->note_to_pitch_ignoring_tuning(modFreq);

            filter[0].setCoeff(mode, modFreq, res, VFXConfig::getSampleRateInv(this), 0.f);
            sst::filters::CytomicSVF::step(filter[0], inL, inR);

            inL += .2f; // some bias for assymetrical saturation
            inR += .2f;
            saturateNext(inL, inR);

            filter[1].fetchCoeffs(filter[0]);
            sst::filters::CytomicSVF::step(filter[1], inL, inR);

            saturateNext(inL, inR);

            DCfilter.processBlockStep(inL, inR); // Filter out the bias DC
            inL *= 1.17f;                        // Compensate for some lost level
            inR *= 1.17f;

            dataoutL[i] = inL;
            dataoutR[i] = inR;
        }
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    bool first = true;
    bool keytrackOn = false;
    sst::basic_blocks::dsp::SlewLimiter speedLimiter;

    sst::basic_blocks::dsp::RunningAverage RA;

    float FreqPrior = -1.f;
    std::array<sst::filters::CytomicSVF, 2> filter;
    sst::filters::CytomicSVF DCfilter;
};
} // namespace sst::voice_effects::dynamics
#endif // SCXT_AUTOWAH_H
