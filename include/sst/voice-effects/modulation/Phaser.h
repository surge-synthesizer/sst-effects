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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct Phaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Phaser"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{2};

    static constexpr int maxPhases{6};

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpFeedback,
        fpSpacing,
        fpResonance,
        fpRate,
        fpDepth,
        fpCenterFreq,
    };

    enum IntParams
    {
        ipShape,
        ipStereo,
    };

    Phaser() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~Phaser() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpFeedback:
            return pmd().asPercent().withDefault(0.5f).withName("Feedback");
        case fpSpacing:
            return pmd()
                .asFloat()
                .withRange(-48, 48)
                .withDefault(12)
                .withName("Spacing")
                .withLinearScaleFormatting("semitones");
        case fpResonance:
            return pmd().asPercent().withDefault(0.707).withName("Resonance");
        case fpRate:
            return pmd().asLfoRate(-3, 4).withName("Rate");
        case fpDepth:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(1.f)
                .withLinearScaleFormatting("%", 100.f)
                .withName("Depth");
        case fpCenterFreq:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("Center Freq Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Center Frequency").withDefault(0);
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipStereo:
            return pmd().asBool().withDefault(false).withName("Stereo");
        case ipShape:
            return pmd()
                .asInt()
                .withRange(0, 6)
                .withUnorderedMapFormatting({
                    {0, "Sine"},
                    {1, "Triangle"},
                    {2, "Ramp Up"},
                    {3, "Ramp Down"},
                    {4, "Square"},
                    {5, "Noise"},
                    {6, "S&H"},
                })
                .withName("LFO shape");
        }
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect()
    {
        lipolFb.set_target_instant(
            std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<Phaser, VFXConfig::blockSize>;
    lfo_t actualLFO{this, 1};
    typename lfo_t::Shape lfoShape = lfo_t::Shape::SINE;

    void shapeCheck()
    {
        switch (this->getIntParam(ipShape))
        {
        case 0:
            lfoShape = lfo_t::SINE;
            break;
        case 1:
            lfoShape = lfo_t::TRI;
            break;
        case 2:
            lfoShape = lfo_t::RAMP;
            break;
        case 3:
            lfoShape = lfo_t::DOWN_RAMP;
            break;
        case 4:
            lfoShape = lfo_t::PULSE;
            break;
        case 5:
            lfoShape = lfo_t::SMOOTH_NOISE;
            break;
        case 6:
            lfoShape = lfo_t::SH_NOISE;
            break;
        }
    }

    bool phaseSet = false;

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);

        if (!phaseSet)
        {
            auto phase = rng.unif01();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }
        shapeCheck();
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = actualLFO.lastTarget * lfoDepth;

        calc_coeffs(pitch, lfoValue);

        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        lipolFb.set_target(std::sqrt(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f)));
        float fb alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(fb);

        for (int k = 0; k < VFXConfig::blockSize; ++k)
        {
            auto dL = dataoutL[k] + fbAmt[0];
            auto dR = dataoutR[k] + fbAmt[1];
            for (int i = 0; i < 4; ++i)
            {
                filters[i].processBlockStep(dL, dR);
            }
            fbAmt[0] = dL * fb[k];
            fbAmt[1] = dR * fb[k];
            dataoutL[k] = dL;
            dataoutR[k] = dR;
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    void processMonoToMono(float *dataIn, float *dataOut, float pitch)
    {
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);

        if (!phaseSet)
        {
            auto phase = rng.unif01();
            actualLFO.applyPhaseOffset(phase);
            phaseSet = true;
        }

        shapeCheck();
        actualLFO.process_block(lfoRate, 0.f, lfoShape);

        float lfoValue = actualLFO.lastTarget * lfoDepth;

        this->calc_coeffs(pitch, lfoValue);

        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataIn, dataOut);

        this->lipolFb.set_target(
            std::sqrt(std::clamp(this->getFloatParam(this->fpFeedback), 0.f, 0.97f)));
        float fb alignas(16)[VFXConfig::blockSize];
        this->lipolFb.store_block(fb);

        for (int k = 0; k < VFXConfig::blockSize; ++k)
        {
            auto dL = dataOut[k] + this->fbAmt[0];
            for (int i = 0; i < 4; ++i)
            {
                float tmp{0};
                this->filters[i].processBlockStep(dL, tmp);
            }
            this->fbAmt[0] = dL * fb[k];
            dataOut[k] = dL;
        }
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

    bool isFirst = true;

    void calc_coeffs(float pitch = 0.f, float LFO = 0.f)
    {
        auto resonance = this->getFloatParam(fpResonance);
        auto baseFreq = this->getFloatParam(fpCenterFreq);

        if (keytrackOn)
        {
            baseFreq += pitch;
        }

        auto spread{0.f};
        auto mode = sst::filters::CytomicSVF::Mode::ALL;
        auto stereo = this->getIntParam(ipStereo);

        if (isFirst)
        {
            for (int i = 0; i < 4; ++i)
            {
                filters[i].init();
            }
            isFirst = false;
        }

        spread = this->getFloatParam(fpSpacing);
        auto halfStage = 4 * 0.5;
        baseFreq -= halfStage * spread;

        float lfoValueL = LFO * (baseFreq / 2);
        float lfoValueR = LFO * (baseFreq / 2) * (stereo ? -1 : 1);

        for (int i = 0; i < 4; ++i)
        {
            auto freqL =
                440.f * this->note_to_pitch_ignoring_tuning((baseFreq + spread * i) + lfoValueL);
            auto freqR =
                440.f * this->note_to_pitch_ignoring_tuning((baseFreq + spread * i) + lfoValueR);

            auto res = std::clamp(resonance, 0.f, 1.f);
            filters[i].template setCoeffForBlock<VFXConfig::blockSize>(
                mode, freqL, freqR, res, res, this->getSampleRateInv(), 1.f, 1.f);
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
    bool keytrackOn{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<sst::filters::CytomicSVF, 4> filters;
    float fbAmt[2]{0.f, 0.f};
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb;
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_PHASER_H
