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

#ifndef INCLUDE_SST_VOICE_EFFECTS_PITCH_PITCHRING_H
#define INCLUDE_SST_VOICE_EFFECTS_PITCH_PITCHRING_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/HilbertTransform.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
namespace mech = sst::basic_blocks::mechanics;

namespace sst::voice_effects::pitch
{
template <typename VFXConfig> struct PitchRing : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"PitchRing"};

    enum struct PitchRingFloatParams : uint32_t
    {
        fine,
        coarse,
        feedback,
        num_params
    };

    enum struct PitchRingIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)PitchRingFloatParams::num_params};
    static constexpr int numIntParams{(int)PitchRingIntParams::num_params};

    PitchRing() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~PitchRing() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)PitchRingFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((PitchRingFloatParams)idx)
        {
        case PitchRingFloatParams::fine:
            return pmd()
                .asFloat()
                .withName("Fine")
                .withLinearScaleFormatting("hz")
                .withRange(-10, 10)
                .withDefault(0);
        case PitchRingFloatParams::coarse:
            return pmd()
                .asFloat()
                .withName("Coarse")
                .withLinearScaleFormatting("hz")
                .withRange(-1000, 1000)
                .withDefault(0);
        case PitchRingFloatParams::feedback:
            return pmd().asPercentBipolar().withName("Feedback").withDefault(0);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect()
    {
        auto lr = 0.006f;
        mRateLag.setRate(lr);
        mFeedbackLag.setRate(lr);

        mHilbertStereo.setSampleRate(this->getSampleRate());
        mHilbertMono.setSampleRate(this->getSampleRate());
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        mRateLag.newValue(this->getFloatParam((int)PitchRingFloatParams::coarse) +
                          this->getFloatParam((int)PitchRingFloatParams::fine));
        mFeedbackLag.newValue(this->getFloatParam((int)PitchRingFloatParams::feedback));

        mSinOsc.setRate(2.0 * M_PI * mRateLag.v * this->getSampleRateInv());

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto fb = mFeedbackLag.v;
            fb = fb * fb * fb;

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

            mRateLag.process();
            mFeedbackLag.process();
        }
    }

    // void processMonoToMono(float *datainL, float *dataoutL, float pitch) {}

  protected:
    float mSampleRate{1};
    float mPrior[2]{0.f, 0.f};
    sst::basic_blocks::dsp::HilbertTransformStereoSSE mHilbertStereo;
    sst::basic_blocks::dsp::HilbertTransformMonoFloat mHilbertMono;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOsc;

    sst::basic_blocks::dsp::SurgeLag<float, true> mRateLag, mFeedbackLag;
};
} // namespace sst::voice_effects::pitch

#endif // SHORTCIRCUITXT_PitchRing_H
