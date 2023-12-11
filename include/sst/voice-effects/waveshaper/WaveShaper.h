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

#ifndef INCLUDE_SST_VOICE_EFFECTS_WAVESHAPER_WAVESHAPER_H
#define INCLUDE_SST_VOICE_EFFECTS_WAVESHAPER_WAVESHAPER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/waveshapers.h"
#include "sst/filters/HalfRateFilter.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
namespace mech = sst::basic_blocks::mechanics;

namespace sst::voice_effects::waveshaper
{
template <typename VFXConfig> struct WaveShaper : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"WaveShaper"};

    enum struct WaveShaperFloatParams : uint32_t
    {
        drive,
        bias,
        postgain,
        num_params
    };

    enum struct WaveShaperIntParams : uint32_t
    {
        type,
        oversample,
        num_params
    };

    static constexpr int numFloatParams{(int)WaveShaperFloatParams::num_params};
    static constexpr int numIntParams{(int)WaveShaperIntParams::num_params};

    WaveShaper()
        : core::VoiceEffectTemplateBase<VFXConfig>(), mHalfRateUp(6, true), mHalfRateDown(6, true)
    {
    }

    ~WaveShaper() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)WaveShaperFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((WaveShaperFloatParams)idx)
        {
        case WaveShaperFloatParams::drive:
            return pmd().asDecibel().withName("Drive");
        case WaveShaperFloatParams::bias:
            return pmd().asPercentBipolar().withName("Bias");
        case WaveShaperFloatParams::postgain:
            return pmd().asLinearDecibel().withName("Gain");
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((WaveShaperIntParams)idx)
        {
        case WaveShaperIntParams::type:
            return pmd()
                .asInt()
                .withRange(0, 2)
                .withName("Type")
                .withUnorderedMapFormatting({{0, "Soft"}, {1, "OJD"}, {2, "WCFold"}})
                .withDefault(0);
        case WaveShaperIntParams::oversample:
            return pmd().asBool().withName("OverSample").withDefault(0);
        default:
            break;
        }

        return pmd().asInt().withRange(0, 1).withName("Error");
    }

    void initVoiceEffectParams()
    {
        this->setFloatParam((int)WaveShaperFloatParams::drive, 0.f);
        this->setFloatParam((int)WaveShaperFloatParams::bias, 0.f);
        this->setFloatParam((int)WaveShaperFloatParams::postgain, 0.f);

        this->setIntParam((int)WaveShaperIntParams::type, 1); // force to ojd for now
    }

    template <bool stereo>
    void processInternalOS(float *datainL, float *datainR, float *dataoutL, float *dataoutR)
    {
        // Todo: Smooth
        auto drv = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::drive));
        auto bias = this->getFloatParam((int)WaveShaperFloatParams::bias);
        auto gain = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::postgain));

        mDriveLipOS.newValue(drv);
        mBiasLipOS.newValue(bias);
        mGainLipOS.newValue(gain);

        float inOSL alignas(16)[VFXConfig::blockSize * 2];
        float inOSR alignas(16)[VFXConfig::blockSize * 2];
        float outOSL alignas(16)[VFXConfig::blockSize * 2];
        float outOSR alignas(16)[VFXConfig::blockSize * 2];

        if (stereo)
        {
            mHalfRateUp.process_block_U2(datainL, datainR, inOSL, inOSR, VFXConfig::blockSize * 2);
        }
        else
        {
            mHalfRateUp.process_block_U2(datainL, datainL, inOSL, inOSR, VFXConfig::blockSize * 2);
        }

        float resa alignas(16)[4];
        for (auto i = 0U; i < VFXConfig::blockSize * 2; ++i)
        {
            auto mdrv = _mm_set1_ps(mDriveLipOS.v);

            __m128 val;

            val = _mm_set_ps(0.f, 0.f, 2 * inOSR[i] + mBiasLipOS.v, 2 * inOSL[i] + mBiasLipOS.v);
            auto res = mWSOp(&mWss, val, mdrv);
            res = _mm_mul_ps(res, _mm_set1_ps(mGainLipOS.v));
            _mm_store_ps(resa, res);
            outOSL[i] = resa[0];
            outOSR[i] = resa[1];
            mDriveLipOS.process();
            mBiasLipOS.process();
            mGainLipOS.process();
        }

        if (stereo)
        {
            mHalfRateDown.process_block_D2(outOSL, outOSR, VFXConfig::blockSize * 2, dataoutL,
                                           dataoutR);
        }
        else
        {
            // Use the OS input as a scratch output buffer
            mHalfRateDown.process_block_D2(outOSL, outOSR, VFXConfig::blockSize * 2, dataoutL,
                                           inOSL);
        }
    }

    template <bool stereo>
    void processInternal(float *datainL, float *datainR, float *dataoutL, float *dataoutR)
    {
        // Todo: Smooth
        auto drv = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::drive));
        auto bias = this->getFloatParam((int)WaveShaperFloatParams::bias);
        auto gain = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::postgain));

        mDriveLip.newValue(drv);
        mBiasLip.newValue(bias);
        mGainLip.newValue(gain);

        float resa alignas(16)[4];
        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto mdrv = _mm_set1_ps(mDriveLip.v);

            __m128 val;

            if constexpr (stereo)
            {
                val = _mm_set_ps(0.f, 0.f, datainR[i] + mBiasLip.v, datainL[i] + mBiasLip.v);
            }
            else
            {
                val = _mm_set_ps(0.f, 0.f, 0.f, datainL[i] + mBiasLip.v);
            }
            auto res = mWSOp(&mWss, val, mdrv);
            res = _mm_mul_ps(res, _mm_set1_ps(mGainLip.v));
            _mm_store_ps(resa, res);
            dataoutL[i] = resa[0];
            if constexpr (stereo)
            {
                dataoutR[i] = resa[1];
            }
            mDriveLip.process();
            mBiasLip.process();
            mGainLip.process();
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        checkType();

        if (!mWSOp)
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
            return;
        }

        if (mOversample)
        {
            processInternalOS<true>(datainL, datainR, dataoutL, dataoutR);
        }
        else
        {
            processInternal<true>(datainL, datainR, dataoutL, dataoutR);
        }
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        checkType();

        if (!mWSOp)
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            return;
        }

        if (mOversample)
        {
            processInternalOS<false>(datainL, nullptr, dataoutL, nullptr);
        }
        else
        {
            processInternal<false>(datainL, nullptr, dataoutL, nullptr);
        }
    }

    void checkType()
    {
        if (this->getIntParam((int)WaveShaperIntParams::type) != mTypeParamVal)
        {
            mTypeParamVal = this->getIntParam((int)WaveShaperIntParams::type);
            switch (mTypeParamVal)
            {
            case 0:
                mWSType = sst::waveshapers::WaveshaperType::wst_soft;
                break;
            case 1:
                mWSType = sst::waveshapers::WaveshaperType::wst_ojd;
                break;
            case 2:
                mWSType = sst::waveshapers::WaveshaperType::wst_westfold;
                break;
            }

            float R[sst::waveshapers::n_waveshaper_registers];
            sst::waveshapers::initializeWaveshaperRegister(mWSType, R);
            for (int i = 0; i < sst::waveshapers::n_waveshaper_registers; ++i)
            {
                mWss.R[i] = _mm_set1_ps(R[i]);
            }
            mWss.init = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_setzero_ps());
            mWSOp = sst::waveshapers::GetQuadWaveshaper(mWSType);
        }
        auto los = this->getIntParam((int)WaveShaperIntParams::oversample) != 0;
        if (los != mOversample)
        {
            mOversample = los;
            mHalfRateDown.reset();
            mHalfRateUp.reset();
        }
    }

  protected:
    int mTypeParamVal{-1};
    bool mOversample{false};
    sst::waveshapers::WaveshaperType mWSType;
    sst::waveshapers::QuadWaveshaperState mWss;
    sst::waveshapers::QuadWaveshaperPtr mWSOp{nullptr};
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mDriveLip, mBiasLip, mGainLip;
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize << 1, true> mDriveLipOS, mBiasLipOS,
        mGainLipOS;
    sst::filters::HalfRate::HalfRateFilter mHalfRateUp, mHalfRateDown;
};
} // namespace sst::voice_effects::waveshaper

#endif // SHORTCIRCUITXT_WAVESHAPER_H
