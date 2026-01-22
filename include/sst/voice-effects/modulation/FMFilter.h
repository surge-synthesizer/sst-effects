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

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct FMFilter : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"FM Filter"};
    static constexpr const char *streamingName{"filt-fm"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    enum FloatParams
    {
        fpFreqL,
        fpFreqR,
        fpDepth,
        fpRes,
        fpModFreq,
    };

    enum IntParams
    {
        ipStereo,
        ipMode,
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
                    .withSemitoneFormatting();
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
                    .withSemitoneFormatting();
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
        case fpModFreq:
            return pmd()
                .asFloat()
                .withRange(-48, 48)
                .withDefault(0.f)
                .withName("Modulator Tuning")
                .withSemitoneFormatting();
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
                    {0, "Lowpass"},
                    {1, "Highpass"},
                    {2, "Bandpass"},
                    {3, "Notch"},
                    {4, "Allpass"},
                })
                .withDefault((int)md::Lowpass);
        }
        return pmd().withName("error");
    }

    FMFilter() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~FMFilter() {}

    void initVoiceEffect()
    {
        depthLerp.set_target_instant(std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f));
        resLerp.set_target_instant(std::clamp(this->getFloatParam(fpRes), 0.f, 1.f));

        DCfilter.setCoeff(sst::filters::CytomicSVF::Mode::Highpass, 18.f, 18.f, 0.5f, 0.5f,
                          VFXConfig::getSampleRateInv(this), 0.f, 0.f);
        DCfilter.retainCoeffForBlock<VFXConfig::blockSize>();
    }
    void initVoiceEffectPitch(float pitch)
    {
        auto freqL = this->getFloatParam(fpFreqL) + pitch * keytrackOn;
        auto freqR = this->getFloatParam((this->getIntParam(ipStereo)) ? fpFreqR : fpFreqL) +
                     pitch * keytrackOn;
        freqL = 440.0f * this->note_to_pitch_ignoring_tuning(freqL);
        freqR = 440.0f * this->note_to_pitch_ignoring_tuning(freqR);

        freqLerpL.set_target_instant(freqL);
        freqLerpR.set_target_instant(freqR);
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        bool stereo = this->getIntParam(ipStereo);

        resLerp.set_target(std::clamp(this->getFloatParam(fpRes), 0.f, 1.f));
        float res alignas(16)[VFXConfig::blockSize];
        resLerp.store_block(res);

        depthLerp.set_target(std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f));
        float depth alignas(16)[VFXConfig::blockSize];
        depthLerp.store_block(depth);

        auto fL = this->getFloatParam(fpFreqL) + pitch * keytrackOn;
        auto fR = this->getFloatParam((stereo) ? fpFreqR : fpFreqL) + pitch * keytrackOn;
        fL = 440.0f * this->note_to_pitch_ignoring_tuning(fL);
        fR = 440.0f * this->note_to_pitch_ignoring_tuning(fR);
        freqLerpL.set_target(fL);
        freqLerpR.set_target(fR);
        float freqL alignas(16)[VFXConfig::blockSize];
        float freqR alignas(16)[VFXConfig::blockSize];
        freqLerpL.store_block(freqL);
        freqLerpR.store_block(freqR);

        auto tune = this->getFloatParam(fpModFreq);
        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pitch + tune) *
                        this->getSampleRateInv());

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

        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float inL = datainL[i];
            float inR = datainR[i];

            mSinOsc.step();

            float modL = mSinOsc.v * freqL[i] * 3 * depth[i];
            float modR = mSinOsc.v * freqR[i] * 3 * depth[i];
            auto modFreqL = freqL[i] + modL;
            auto modFreqR = freqR[i] + modR;

            filter.setCoeff(mode, modFreqL, modFreqR, res[i], res[i],
                            VFXConfig::getSampleRateInv(this), 0.f, 0.f);
            sst::filters::CytomicSVF::step(filter, inL, inR);

            DCfilter.processBlockStep(inL, inR);

            dataoutL[i] = inL;
            dataoutR[i] = inR;
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        resLerp.set_target(std::clamp(this->getFloatParam(fpRes), 0.f, 1.f));
        float res alignas(16)[VFXConfig::blockSize];
        resLerp.store_block(res);

        depthLerp.set_target(std::clamp(this->getFloatParam(fpDepth), 0.f, 1.f));
        float depth alignas(16)[VFXConfig::blockSize];
        depthLerp.store_block(depth);

        auto f = this->getFloatParam(fpFreqL) + pitch * keytrackOn;
        f = 440.0f * this->note_to_pitch_ignoring_tuning(f);
        freqLerpL.set_target(f);
        float freq alignas(16)[VFXConfig::blockSize];
        freqLerpL.store_block(freq);

        auto tune = this->getFloatParam(fpModFreq);
        mSinOsc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(tune + pitch) *
                        this->getSampleRateInv());

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

        for (auto i = 0; i < VFXConfig::blockSize; ++i)
        {
            float input = datainL[i];

            mSinOsc.step();

            float mod = mSinOsc.v * freq[i] * 3 * depth[i];
            auto modFreq = mod + freq[i];

            filter.setCoeff(mode, modFreq, res[i], VFXConfig::getSampleRateInv(this), 0.f);
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
    bool getKeytrackDefault() const { return true; }
    bool checkParameterConsistency() const { return true; }
    size_t silentSamplesLength() const { return 10; }

  protected:
    bool keytrackOn{false};
    sst::filters::CytomicSVF filter;
    sst::filters::CytomicSVF DCfilter;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOsc;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> freqLerpL, freqLerpR, resLerp,
        depthLerp, tuneLerp;

    sst::filters::CytomicSVF::Mode mode{filters::CytomicSVF::Mode::Lowpass};
    int priorMode{-1};

  public:
    static constexpr int16_t streamingVersion{2};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        assert(streamedFrom <= 2);
        if (streamedFrom == 1)
        {
            // removed n/d params and replaced with float carrier freq param.
            if (iparam[3] == 0)
                return;

            fparam[4] = std::log2(1.f * iparam[2] / iparam[3]) * 12.f;
        }
    }
};
} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_FMFILTER_H
