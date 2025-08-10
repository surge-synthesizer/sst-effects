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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_SUPERSAW_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_SUPERSAW_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/DPWSawPulseOscillator.h"


#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/OscillatorDriftUnisonCharacter.h"
#include "sst/effects-shared/WidthProvider.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct SuperSaw : core::VoiceEffectTemplateBase<VFXConfig>,
                                                effects_shared::WidthProvider<SuperSaw<VFXConfig>, VFXConfig::blockSize, true>
{
    static constexpr const char *effectName{"VA Oscillator"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    static constexpr int maxVoices{9};

    enum FloatParams
    {
        fpOffset,
        fpLevel,
        fpDetune,
        fpDrift,
        fpStereoWidth
    };

    enum IntParams
    {
        ipUnisonVoices,
        ipStereo
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        auto stereo = this->getIntParam(ipStereo);

        auto ot = this->getIntParam(0);
        switch (idx)
        {
        case fpOffset:
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
        case fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
        case fpDetune:
            return pmd()
            .asFloat()
            .withRange(0.f,1.f)
            .withDefault(.1)
            .withLinearScaleFormatting( "cents", 100)
            .withName("Detune");
        case fpDrift:
            return pmd()
            .asPercent()
            .withDefault(0.f)
            .withName("Drift");
        case fpStereoWidth:
            return this->getWidthParam()
            .withName(!stereo ? std::string() : "Width")
            .withRange(0.f, 2.f);

        }
        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipUnisonVoices:
            return pmd()
            .asInt()
            .withRange(1, maxVoices)
            .withDefault(3)
            .withLinearScaleFormatting("voices")
            .withName("Unison");
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        }
        return pmd().withName("error");
    }

    SuperSaw() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~SuperSaw() {}

    void initVoiceEffect()
    {
        calcUnison(true);
        for (int i = 0; i < maxVoices; i++)
        {
            driftLfos[i].init(true);
        }
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void calcUnison(bool init = false)
    {
        int voices = this->getIntParam(ipUnisonVoices);
        if (init) voices = maxVoices;
        basic_blocks::dsp::UnisonSetup<float> uniSet(voices);

        for (int v = 0; v < voices; v++)
        {
            uniSet.attenuatedPanLaw(v, panL[v], panR[v]);
            voiceDetune[v] = uniSet.detune(v);
            priorVoiceCount = this->getIntParam(ipUnisonVoices);
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        auto voiceCount = this->getIntParam(ipUnisonVoices);

        if (voiceCount != priorVoiceCount)
        {
            calcUnison();
        }

        auto tune = this->getFloatParam(fpOffset);
        if (keytrackOn) tune += pitch;

        for (int v = 0; v < voiceCount; v++)
        {
            auto vDrift = driftLfos[v].next() * this->getFloatParam(fpDrift);
            auto vDet = voiceDetune[v] * this->getFloatParam(fpDetune);
            auto voiceFreq = 440 * this->note_to_pitch_ignoring_tuning(tune + vDrift + vDet);
            sawOscillators[v].setFrequency(voiceFreq, this->getSampleRateInv());
        }


        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataoutL[i] = 0.f;

            for (int v = 0; v < voiceCount; v++)
            {
                dataoutL[i] += sawOscillators[v].step();
            }
        }
        levelLerp.set_target(this->getFloatParam(fpLevel));
        levelLerp.multiply_block(dataoutL);;
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        auto voiceCount = this->getIntParam(ipUnisonVoices);

        if (voiceCount != priorVoiceCount)
        {
            calcUnison();
        }

        auto tune = this->getFloatParam(fpOffset);
        if (keytrackOn) tune += pitch;

        for (int v = 0; v < voiceCount; v++)
        {
            auto vDrift = driftLfos[v].next() * this->getFloatParam(fpDrift);
            auto vDet = voiceDetune[v] * this->getFloatParam(fpDetune);
            auto voiceFreq = 440 * this->note_to_pitch_ignoring_tuning(tune + vDrift + vDet);
            sawOscillators[v].setFrequency(voiceFreq, this->getSampleRateInv());
        }


        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            dataoutL[i] = 0.f;
            dataoutR[i] = 0.f;

            for (int v = 0; v < voiceCount; v++)
            {
                auto s = sawOscillators[v].step();
                dataoutL[i] += s * panL[v];
                dataoutR[i] += s * panR[v];
            }
        }

        levelLerp.set_target(this->getFloatParam(fpLevel));
        levelLerp.multiply_2_blocks(dataoutL, dataoutR);;

        if (this->getIntParam(ipStereo))
        {
            this->setWidthTarget(widthLerpS, widthLerpM, fpStereoWidth);
            this->applyWidth(dataoutL, dataoutR, widthLerpS, widthLerpM);
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

  protected:
    bool keytrackOn{true};

    float priorVoiceCount{-1};

    std::array<basic_blocks::dsp::DPWSawOscillator<
        basic_blocks::dsp::BlockInterpSmoothingStrategy<VFXConfig::blockSize>>, maxVoices> sawOscillators;
    std::array<basic_blocks::dsp::DriftLFO, maxVoices> driftLfos;
    std::array<float, maxVoices> panL;
    std::array<float, maxVoices> panR;
    std::array<float, maxVoices> voiceDetune;

    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> levelLerp, widthLerpS, widthLerpM;


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

#endif // SHORTCIRCUITXT_SUPERSAW_H
