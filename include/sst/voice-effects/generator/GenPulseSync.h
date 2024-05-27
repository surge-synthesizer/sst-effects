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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENPULSESYNC_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENPULSESYNC_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"

#include "../VoiceEffectCore.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenPulseSync : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"GenPulseSync"};

    using SincTable = sst::basic_blocks::tables::ShortcircuitSincTableProvider;

    const SincTable &sSincTable;

    enum struct GenPulseSyncFloatParams : uint32_t
    {
        tune,
        width,
        sync,
        level,
        num_params
    };

    enum struct GenPulseSyncIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)GenPulseSyncFloatParams::num_params};
    static constexpr int numIntParams{(int)GenPulseSyncIntParams::num_params};

    GenPulseSync(const SincTable &st) : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>()
    {
        mFirstRun = true;
        mOscState = 0;
        mSyncState = 0;
        mOscOut = 0;
        mPolarity = false;
        mBufPos = 0;

        for (float &v : mOscBuffer)
            v = 0.f;
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        /*
                tune,
                    width,
                    sync,
                    level,*/
        switch ((GenPulseSyncFloatParams)idx)
        {
        case GenPulseSyncFloatParams::tune:
            return pmd()
                .asFloat()
                .withRange(-96, 96)
                .withName("Tune")
                .withDefault(0)
                .withLinearScaleFormatting("semitones");
        case GenPulseSyncFloatParams::width:
            return pmd().asPercent().withName("Width").withDefault(0.5);
        case GenPulseSyncFloatParams::sync:
            return pmd()
                .asFloat()
                .withRange(0, 96)
                .withName("Sync")
                .withDefault(0)
                .withLinearScaleFormatting("semitones");
        case GenPulseSyncFloatParams::level:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float mPitch)
    {
        processMonoToMono(datainL, dataoutL, mPitch);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        mTuneLerp.newValue(this->getFloatParam((int)GenPulseSyncFloatParams::tune));
        mWidthLerp.newValue(this->getFloatParam((int)GenPulseSyncFloatParams::width));
        mSyncLerp.newValue(this->getFloatParam((int)GenPulseSyncFloatParams::sync));
        mLevelLerp.newValue(this->getFloatParam((int)GenPulseSyncFloatParams::level));
        mPitchLerp.newValue(pitch);

        if (mFirstRun)
        {
            mFirstRun = false;

            // initial antipulse
            convolute();
            mOscState -= kLarge;
            for (auto i = 0U; i < VFXConfig::blockSize; i++)
            {
                mOscBuffer[i] *= -0.5f;
            }
            mOscState = 0;
            mPolarity = 0;
        }

        for (auto k = 0U; k < VFXConfig::blockSize; k++)
        {
            mOscState -= kLarge;
            mSyncState -= kLarge;
            while (mSyncState < 0)
                this->convolute();
            while (mOscState < 0)
                this->convolute();
            mOscOut = mOscOut * kIntegratorHPF + mOscBuffer[mBufPos];
            dataoutL[k] = mOscOut * mLevelLerp.v * mLevelLerp.v * mLevelLerp.v;
            mOscBuffer[mBufPos] = 0.f;

            mBufPos++;
            mBufPos = mBufPos & (VFXConfig::blockSize - 1);

            mWidthLerp.process();
            mSyncLerp.process();
            mTuneLerp.process();
            mLevelLerp.process();
            mPitchLerp.process();
        }
    }

  protected:
    void convolute()
    {
        auto samplerate = this->getSampleRate();
        int32_t ipos = (int32_t)(((kLarge + mOscState) >> 16) & 0xFFFFFFFF);
        bool sync = false;
        if (mSyncState < mOscState)
        {
            ipos = (int32_t)(((kLarge + mSyncState) >> 16) & 0xFFFFFFFF);
            double t =
                std::max(0.5, samplerate / (440.0 * this->note_to_pitch_ignoring_tuning(
                                                        (double)mPitchLerp.v + mTuneLerp.v)));
            int64_t syncrate = (int64_t)(double)(65536.0 * 16777216.0 * t);
            mOscState = mSyncState;
            mSyncState += syncrate;
            sync = true;
        }
        // generate pulse
        float fpol = mPolarity ? -1.0f : 1.0f;
        int32_t m = ((ipos >> 16) & 0xff) * SincTable::FIRipol_N;
        float lipol = ((float)((uint32_t)(ipos & 0xffff)));

        if (!sync || !mPolarity)
        {
            for (auto k = 0U; k < SincTable::FIRipol_N; k++)
            {
                mOscBuffer[mBufPos + k & (VFXConfig::blockSize - 1)] +=
                    fpol *
                    (sSincTable.SincTableF32[m + k] + lipol * sSincTable.SincOffsetF32[m + k]);
            }
        }

        if (sync)
            mPolarity = false;

        // add time until next statechange
        double width = (0.5 - 0.499f * std::clamp(mWidthLerp.v, 0.01f, 0.99f));
        double t = std::max(
            0.5,
            samplerate / (440.0 * this->note_to_pitch_ignoring_tuning(
                                      (double)mPitchLerp.v +
                                      this->getFloatParam((int)GenPulseSyncFloatParams::tune) +
                                      this->getFloatParam((int)GenPulseSyncFloatParams::sync))));
        if (mPolarity)
        {
            width = 1 - width;
        }
        int64_t rate = (int64_t)(double)(65536.0 * 16777216.0 * t * width);

        mOscState += rate;
        mPolarity = !mPolarity;
    }

    static constexpr int64_t kLarge = 0x10000000000;
    static constexpr float kIntegratorHPF = 0.99999999f;

    float mOscBuffer alignas(16)[VFXConfig::blockSize];

    bool mFirstRun{true};
    int64_t mOscState{0}, mSyncState{0};
    bool mPolarity{false};
    float mOscOut{0};
    size_t mBufPos{0};

    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mPitchLerp, mTuneLerp,
        mWidthLerp, mSyncLerp, mLevelLerp;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenPulseSync_H
