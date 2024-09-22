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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_FREQSHIFTMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_FREQSHIFTMOD_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/HilbertTransform.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct FreqShiftMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Freq Shift Mod"};

    enum struct FreqShiftModFloatParams : uint32_t
    {
        fine,
        coarse,
        feedback,
        num_params
    };

    enum struct FreqShiftModIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)FreqShiftModFloatParams::num_params};
    static constexpr int numIntParams{(int)FreqShiftModIntParams::num_params};

    FreqShiftMod() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~FreqShiftMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((FreqShiftModFloatParams)idx)
        {
        case FreqShiftModFloatParams::fine:
            return pmd()
                .asFloat()
                .withName("Fine")
                .withLinearScaleFormatting("hz")
                .withRange(-10, 10)
                .withDefault(0);
        case FreqShiftModFloatParams::coarse:
            return pmd()
                .asFloat()
                .withName("Coarse")
                .withLinearScaleFormatting("hz")
                .withRange(-1000, 1000)
                .withDefault(0);
        case FreqShiftModFloatParams::feedback:
            return pmd().asPercentBipolar().withName("Feedback").withDefault(0);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect()
    {
        mHilbertStereo.setSampleRate(this->getSampleRate());
        mHilbertMono.setSampleRate(this->getSampleRate());
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        auto rate = this->getFloatParam((int)FreqShiftModFloatParams::coarse) +
                    this->getFloatParam((int)FreqShiftModFloatParams::fine);
        mSinOsc.setRate(2.0 * M_PI * rate * this->getSampleRateInv());

        auto fbp = this->getFloatParam((int)FreqShiftModFloatParams::feedback) * 0.75;
        mFeedbackLerp.newValue(fbp);

        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto fb = mFeedbackLerp.v;

            auto iL = datainL[i] + fb * mPrior[0];
            auto iR = datainR[i] + fb * mPrior[1];

            auto [L, R] = mHilbertStereo.stepToPair(iL, iR);
            auto [Lre, Lim] = L;
            auto [Rre, Rim] = R;

            mSinOsc.step();

            mPrior[0] = (Lre * mSinOsc.v - Lim * mSinOsc.u);
            mPrior[1] = (Rre * mSinOsc.v - Rim * mSinOsc.u);

            dataoutL[i] = mPrior[0];
            dataoutR[i] = mPrior[1];

            mFeedbackLerp.process();
        }
    }

    // void processMonoToMono(float *datainL, float *dataoutL, float pitch) {}

  protected:
    float mSampleRate{1};
    float mPrior[2]{0.f, 0.f};
    sst::basic_blocks::dsp::HilbertTransformStereoSSE mHilbertStereo;
    sst::basic_blocks::dsp::HilbertTransformMonoFloat mHilbertMono;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOsc;

    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mFeedbackLerp;

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

#endif // SHORTCIRCUITXT_FreqShiftMod_H
