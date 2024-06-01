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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_FM_FILTER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_FM_FILTER_H

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

        switch (idx)
        {
        case fpFreqL:
            return pmd().asAudibleFrequency().withName("Frequency L");
        case fpFreqR:
            return pmd().asAudibleFrequency().withName("Frequency R");
        case fpDepthL:
            return pmd().asFloat().withRange(0.f, 1.f).withDefault(0.f).withName("FM Depth L");
        case fpRes:
            return pmd().asFloat().withRange(0.f, 1.f).withDefault(0.7f).withName("Resonance");
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
            return pmd().asBool().withDefault(false).withName("Stereo");

        case ipMode:
            return pmd()
                .asInt()
                .withRange(0, 4)
                .withName("Mode")
                .withUnorderedMapFormatting({
                    {md::LP, "Low Pass"},
                    {md::HP, "High Pass"},
                    {md::BP, "Band Pass"},
                    {md::NOTCH, "Notch"},
                    {md::ALL, "All Pass"},
                })
                .withDefault(md::LP);
        case ipNum:
            return pmd().asInt().withRange(1, 8).withDefault(1).withName("Numerator");
        case ipDenom:
            return pmd().asInt().withRange(1, 8).withDefault(1).withName("Denominator");
        }
        return pmd().withName("error");
    }

    FMFilter() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~FMFilter() {}

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float getRatio()
    {
        auto num = (float)(this->getIntParam(ipNum));
        auto denom = (float)(this->getIntParam(ipDenom));
        auto ratio = num / denom;
        return 12 * std::log2(ratio);
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto mode = (sst::filters::CytomicSVF::Mode)(this->getIntParam(ipMode));
        auto res = std::clamp(this->getFloatParam(fpRes), 0.f, 1.f);
        bool stereo = this->getIntParam(ipStereo);

        auto depthL = this->getFloatParam(fpDepthL);
        auto depthR = this->getFloatParam(/*(stereo) ? fpDepthR : */ fpDepthL);

        auto freqL = this->getFloatParam(fpFreqL);
        auto freqR = this->getFloatParam((stereo) ? fpFreqR : fpFreqL);
        if (keytrackOn)
        {
            freqL += pitch;
            freqR += pitch;
        }
        freqL = 440.0f * this->note_to_pitch_ignoring_tuning(freqL);
        freqR = 440.0f * this->note_to_pitch_ignoring_tuning(freqR);

        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pitch + getRatio()) *
                        this->getSampleRateInv());

        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float inL = datainL[i];
            float inR = datainR[i];

            mSinOsc.step();

            float modL = mSinOsc.v * freqL * 3 * depthL;
            float modR = mSinOsc.v * freqR * 3 * depthR;
            auto modFreqL = freqL + modL;
            auto modFreqR = freqR + modR;

            filter.setCoeff(mode, modFreqL, modFreqR, res, res, VFXConfig::getSampleRateInv(this),
                            0.f, 0.f);
            sst::filters::CytomicSVF::step(filter, inL, inR);

            dataoutL[i] = inL;
            dataoutR[i] = inR;
        }
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        auto mode = (sst::filters::CytomicSVF::Mode)(this->getIntParam(ipMode));
        auto res = std::clamp(this->getFloatParam(fpRes), 0.f, 1.f);
        auto depth = std::clamp(this->getFloatParam(fpDepthL), 0.f, 1.f);

        auto freq = this->getFloatParam(fpFreqL);
        if (keytrackOn)
        {
            freq += pitch;
        }
        freq = 440.0f * this->note_to_pitch_ignoring_tuning(freq);

        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pitch + getRatio()) *
                        this->getSampleRateInv());

        auto dummyR = 0.f;
        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float input = datainL[i];

            mSinOsc.step();

            float mod = mSinOsc.v * freq * 3 * depth;
            auto modFreq = mod + freq;

            filter.setCoeff(mode, modFreq, res, VFXConfig::getSampleRateInv(this), 0.f);
            sst::filters::CytomicSVF::step(filter, input, dummyR);

            dataoutL[i] = input;
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
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

  protected:
    bool keytrackOn{false};
    sst::filters::CytomicSVF filter;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOsc;
};
} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_FMFILTER_H
