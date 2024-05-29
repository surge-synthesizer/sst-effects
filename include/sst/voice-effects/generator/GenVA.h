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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENVA_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENVA_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/basic-blocks/dsp/DPWSawPulseOscillator.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenVA : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"VA Oscillator"};
    
    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{1};
    
    using SincTable = sst::basic_blocks::tables::ShortcircuitSincTableProvider;

    const SincTable &sSincTable;

    enum FloatParams
    {
        fpOffset,
        fpLevel,
        fpSync,
        fpWidth,
        fpHighpass,
        fpLowpass
    };

    enum IntParams
    {
        ipWaveform
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpOffset:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-96, 96)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones")
                    .withName("Tune");
            }
            return pmd().asAudibleFrequency().withName("Frequency");
        case fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
        case fpWidth:
            return pmd().asPercent().withName("Pulse Width").withDefault(0.5);
        case fpSync:
            return pmd()
                .asFloat()
                .withRange(0, 96)
                .withName("Sync")
                .withDefault(0)
                .withLinearScaleFormatting("semitones");
        case fpHighpass:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("Highpass Offset")
                    .withDefault(-48)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Highpass Frequency");
        case fpLowpass:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-24, 96)
                    .withName("Lowpass Offset")
                    .withDefault(96)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Lowpass Frequency");
        }
        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }
    
    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        
        switch (idx)
        {
            case ipWaveform:
                return pmd()
                .asInt()
                .withRange(0,2)
                .withUnorderedMapFormatting({
                    {0, "Sine"},
                    {1, "Saw"},
                    {2, "Pulse"}
                })
                .withName("Waveform");
        }
        return pmd().withName("error");
    }
    
    GenVA(const SincTable &st) : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>()
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

    ~GenVA() {}

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }
    
    void processSine(float *datainL, float *dataoutL, float pitch)
    {
        if (keytrackOn)
        {
            mQuadOsc.setRate(440.0 * 2 * M_PI *
                             this->note_to_pitch_ignoring_tuning(
                                 this->getFloatParam(fpOffset) + pitch) *
                             this->getSampleRateInv());
        }
        else
        {
            mQuadOsc.setRate(440.0 * 2 * M_PI *
                             this->note_to_pitch_ignoring_tuning(
                                 this->getFloatParam(fpOffset)) *
                             this->getSampleRateInv());
        }
        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;
        sLevelLerp.set_target(levT);
        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = mQuadOsc.v;
            mQuadOsc.step();
        }
        sLevelLerp.multiply_block(dataoutL);
    }
    
    void processSaw(float *datainL, float *dataoutL, float pitch)
    {
        auto tune = this->getFloatParam(fpOffset);
        auto lp = this->getFloatParam(fpLowpass);
        auto hp = this->getFloatParam(fpHighpass);
        
        auto baseFreq = 440.0 * this->note_to_pitch_ignoring_tuning((keytrackOn) ? tune + pitch : tune);
        auto lpFreq = 440 * this->note_to_pitch_ignoring_tuning((keytrackOn) ? lp + pitch : lp);
        auto hpFreq = 440 * this->note_to_pitch_ignoring_tuning((keytrackOn) ? hp + pitch : hp);

        mSawOsc.setFrequency(baseFreq, this->getSampleRateInv());
        
        filters[0].template setCoeffForBlock<VFXConfig::blockSize>(sst::filters::CytomicSVF::LP, lpFreq, 1.f, VFXConfig::getSampleRateInv(this), 0.f);
        filters[1].template setCoeffForBlock<VFXConfig::blockSize>(sst::filters::CytomicSVF::HP, hpFreq, 1.f, VFXConfig::getSampleRateInv(this), 0.f);
        
        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;
        sLevelLerp.set_target(levT);

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = mSawOsc.step();
            filters[0].processBlockStep(dataoutL[k]);
            filters[1].processBlockStep(dataoutL[k]);
        }
        sLevelLerp.multiply_block(dataoutL);
    }
    
    void processPulse(float *datainL, float *dataoutL, float pitch)
    {
        mTuneLerp.newValue(this->getFloatParam(fpOffset));
        mWidthLerp.newValue(this->getFloatParam(fpWidth));
        mSyncLerp.newValue(this->getFloatParam(fpSync));
        levelLerp.newValue(this->getFloatParam(fpLevel));
        if (keytrackOn)
        {
            mPitchLerp.newValue(pitch);
        }

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
            dataoutL[k] = mOscOut * levelLerp.v * levelLerp.v * levelLerp.v;
            mOscBuffer[mBufPos] = 0.f;

            mBufPos++;
            mBufPos = mBufPos & (VFXConfig::blockSize - 1);

            mWidthLerp.process();
            mSyncLerp.process();
            mTuneLerp.process();
            levelLerp.process();
            if (keytrackOn)
            {
                mPitchLerp.process();
            }
        }
    }
    
    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        int wave = this->getIntParam(ipWaveform);
        if (wave == 0)
        {
            processSine(datainL, dataoutL, pitch);
        }
        else if (wave == 1)
        {
            processSaw(datainL, dataoutL, pitch);
        }
        else
        {
            processPulse(datainL, dataoutL, pitch);
        }
    }
    
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {

        processMonoToMono(datainL, dataoutL, pitch);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    bool keytrackOn{true};
    sst::basic_blocks::dsp::QuadratureOscillator<float> mQuadOsc;
    sst::basic_blocks::dsp::DPWSawOscillator<
        sst::basic_blocks::dsp::BlockInterpSmoothingStrategy<VFXConfig::blockSize>>
        mSawOsc;
    
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> sLevelLerp;
    
    static constexpr int64_t kLarge = 0x10000000000;
    static constexpr float kIntegratorHPF = 0.99999999f;

    float mOscBuffer alignas(16)[VFXConfig::blockSize];
    
    bool mFirstRun{true};
    int64_t mOscState{0}, mSyncState{0};
    bool mPolarity{false};
    float mOscOut{0};
    size_t mBufPos{0};
    
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mPitchLerp, mTuneLerp,
    mWidthLerp, mSyncLerp, levelLerp;
    
    std::array<sst::filters::CytomicSVF, 2> filters;
    
    void convolute()
    {
        auto samplerate = this->getSampleRate();
        int32_t ipos = (int32_t)(((kLarge + mOscState) >> 16) & 0xFFFFFFFF);
        bool sync = false;
        double freq = (keytrackOn) ? (mPitchLerp.v + mTuneLerp.v) : mTuneLerp.v;

        if (mSyncState < mOscState)
        {
            ipos = (int32_t)(((kLarge + mSyncState) >> 16) & 0xFFFFFFFF);
            double t =
                std::max(0.5, samplerate / (440.0 * this->note_to_pitch_ignoring_tuning(freq)));
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
            0.5, samplerate /
                     (440.0 * this->note_to_pitch_ignoring_tuning(
                                  freq + this->getFloatParam(fpSync))));
        if (mPolarity)
        {
            width = 1 - width;
        }
        int64_t rate = (int64_t)(double)(65536.0 * 16777216.0 * t * width);

        mOscState += rate;
        mPolarity = !mPolarity;
    }
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GENVA_H
