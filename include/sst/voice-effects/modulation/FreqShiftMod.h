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
        stereo,
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
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withName("Offset")
                    .withRange(-48.f, 48.f)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
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

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        return pmd().asStereoSwitch().withDefault(false);
    }

    void initVoiceEffect()
    {
        mHilbertStereo.setSampleRate(this->getSampleRate());
        mHilbertMono.setSampleRate(this->getSampleRate());

        DCfilter.setCoeff(filters::CytomicSVF::Mode::Highpass, 15.f, .5f, this->getSampleRateInv(),
                          0.f);
        shelf.setCoeff(filters::CytomicSVF::Mode::HighShelf, 5000.f, .1f, this->getSampleRateInv(),
                       .85f);
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        auto coarse = this->getFloatParam((int)FreqShiftModFloatParams::coarse);
        if (keytrackOn)
        {
            coarse = 440 * this->note_to_pitch_ignoring_tuning(pitch + coarse);
        }
        auto fine = this->getFloatParam((int)FreqShiftModFloatParams::fine);
        mSinOscL.setRate(2.0 * M_PI * (coarse + fine) * this->getSampleRateInv());

        auto fbp = this->getFloatParam((int)FreqShiftModFloatParams::feedback);
        mFeedbackLerp.newValue(fbp);

        auto dummy{0.f};
        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto fb = mFeedbackLerp.v * mPrior[0];
            filters::CytomicSVF::step(shelf, fb, dummy);
            fb = sqrtOfTwo * fb / (1 + fb * fb);
            auto iL = datainL[i] + fb;

            auto [re, im] = mHilbertMono.stepPair(iL);
            mSinOscL.step();

            auto o = re * mSinOscL.v - im * mSinOscL.u;
            filters::CytomicSVF::step(DCfilter, o, dummy);
            mPrior[0] = o;

            dataoutL[i] = mPrior[0];

            mFeedbackLerp.process();
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam((int)FreqShiftModIntParams::stereo))
        {
            processStereoImpl<true>(datainL, datainR, dataoutL, dataoutR, pitch);
        }
        else
        {
            processStereoImpl<false>(datainL, datainR, dataoutL, dataoutR, pitch);
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutR, float *dataoutL,
                             float pitch)
    {
        processStereoImpl<true>(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    template <bool stereo>
    void processStereoImpl(const float *const datainL, const float *const datainR, float *dataoutL,
                           float *dataoutR, float pitch)
    {
        auto coarse = this->getFloatParam((int)FreqShiftModFloatParams::coarse);
        if (keytrackOn)
        {
            coarse = 440 * this->note_to_pitch_ignoring_tuning(pitch + coarse);
        }

        auto fine = this->getFloatParam((int)FreqShiftModFloatParams::fine);
        auto rateL = coarse + fine;
        mSinOscL.setRate(2.0 * M_PI * rateL * this->getSampleRateInv());
        if constexpr (stereo)
        {
            auto rateR = coarse - fine;
            mSinOscR.setRate(2.0 * M_PI * rateR * this->getSampleRateInv());
        }

        auto fbp = this->getFloatParam((int)FreqShiftModFloatParams::feedback);

        mFeedbackLerp.newValue(fbp);

        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto fbL = mFeedbackLerp.v * mPrior[0];
            auto fbR = mFeedbackLerp.v * mPrior[1];
            filters::CytomicSVF::step(shelf, fbL, fbR);
            fbL = sqrtOfTwo * fbL / (1 + fbL * fbL);
            fbR = sqrtOfTwo * fbR / (1 + fbR * fbR);

            auto iL = datainL[i] + fbL;
            auto iR = datainR[i] + fbR;

            auto [L, R] = mHilbertStereo.stepToPair(iL, iR);
            auto [Lre, Lim] = L;
            auto [Rre, Rim] = R;

            mSinOscL.step();
            mSinOscR.step();

            auto oL{0.f}, oR{0.f};

            oL = (Lre * mSinOscL.v - Lim * mSinOscL.u);
            if constexpr (stereo)
            {
                oR = (Rre * mSinOscR.v - Rim * mSinOscR.u);
            }
            else
            {
                oR = (Rre * mSinOscL.v - Rim * mSinOscL.u);
            }

            filters::CytomicSVF::step(DCfilter, oL, oR);

            mPrior[0] = oL;
            mPrior[1] = oR;

            dataoutL[i] = mPrior[0];
            dataoutR[i] = mPrior[1];

            mFeedbackLerp.process();
        }
    }

    bool getMonoToStereoSetting() const
    {
        return this->getIntParam((int)FreqShiftModIntParams::stereo);
    }
    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool checkParameterConsistency() const { return true; }
    size_t silentSamplesLength() const { return 10; }

  protected:
    bool keytrackOn{false};
    float mSampleRate{1};
    float mPrior[2]{0.f, 0.f};
    sst::basic_blocks::dsp::HilbertTransformStereoSSE mHilbertStereo;
    sst::basic_blocks::dsp::HilbertTransformMonoFloat mHilbertMono;
    sst::basic_blocks::dsp::QuadratureOscillator<float> mSinOscL, mSinOscR;
    sst::filters::CytomicSVF DCfilter, shelf;

    static constexpr float sqrtOfTwo{1.4142136};

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
