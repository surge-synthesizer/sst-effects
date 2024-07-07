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

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

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
        highpass,
        lowpass,
        num_params
    };

    enum struct WaveShaperIntParams : uint32_t
    {
        type,
        hpOn,
        lpOn,
        num_params
    };

    static constexpr int numFloatParams{(int)WaveShaperFloatParams::num_params};
    static constexpr int numIntParams{(int)WaveShaperIntParams::num_params};

    WaveShaper() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~WaveShaper() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool hp = this->getIntParam((int)WaveShaperIntParams::hpOn) > 0;
        bool lp = this->getIntParam((int)WaveShaperIntParams::lpOn) > 0;

        switch ((WaveShaperFloatParams)idx)
        {
        case WaveShaperFloatParams::drive:
            return pmd().asDecibel().withName("Drive");
        case WaveShaperFloatParams::bias:
            return pmd().asPercentBipolar().withName("Bias");
        case WaveShaperFloatParams::postgain:
            return pmd().asLinearDecibel().withName("Gain");
        case WaveShaperFloatParams::highpass:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(!hp ? "Highpass" : "HP Offset")
                    .withDefault(-48)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withDefault(-60).withName(!hp ? "Highpass"
                                                                            : "HP Frequency");
        case WaveShaperFloatParams::lowpass:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName(!lp ? "Lowpass" : "LP Offset")
                    .withDefault(48)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withDefault(70).withName(!lp ? "Lowpass"
                                                                           : "LP Frequency");
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
        {
            std::unordered_map<int, std::string> names;
            for (int i = 0; i < (int)sst::waveshapers::WaveshaperType::n_ws_types; ++i)
            {
                names[i] = sst::waveshapers::wst_names[i];
            }
            names[(int)sst::waveshapers::WaveshaperType::n_ws_types] = "Error";
            return pmd()
                .asInt()
                .withRange(0, (int)sst::waveshapers::WaveshaperType::n_ws_types - 1)
                .withName("Type")
                .withUnorderedMapFormatting(names)
                .withDefault(1);
        }
        case WaveShaperIntParams::hpOn:
            return pmd().asBool().withDefault(false).withName("Highpass");
        case WaveShaperIntParams::lpOn:
            return pmd().asBool().withDefault(false).withName("Lowpass");
        default:
            break;
        }

        return pmd().asInt().withRange(0, 1).withName("Error");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <bool stereo>
    void processInternal(float *datainL, float *datainR, float *dataoutL, float *dataoutR)
    {
        // Todo: Smooth
        auto drv = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::drive));
        auto bias = this->getFloatParam((int)WaveShaperFloatParams::bias);
        auto gain = this->dbToLinear(this->getFloatParam((int)WaveShaperFloatParams::postgain));

        mDriveLerp.newValue(drv);
        mBiasLerp.newValue(bias);
        mGainLerp.newValue(gain);

        float resa alignas(16)[4];
        for (auto i = 0U; i < VFXConfig::blockSize; ++i)
        {
            auto mdrv = _mm_set1_ps(mDriveLerp.v);

            __m128 val;

            if constexpr (stereo)
            {
                val = _mm_set_ps(0.f, 0.f, datainR[i] + mBiasLerp.v, datainL[i] + mBiasLerp.v);
            }
            else
            {
                val = _mm_set_ps(0.f, 0.f, 0.f, datainL[i] + mBiasLerp.v);
            }
            auto res = mWSOp(&mWss, val, mdrv);
            res = _mm_mul_ps(res, _mm_set1_ps(mGainLerp.v));
            _mm_store_ps(resa, res);
            dataoutL[i] = resa[0];
            if constexpr (stereo)
            {
                dataoutR[i] = resa[1];
            }
            mDriveLerp.process();
            mBiasLerp.process();
            mGainLerp.process();
        }
    }

    void setCoeffsHighpass(float pitch)
    {
        auto hpParam = this->getFloatParam((int)WaveShaperFloatParams::highpass);
        auto hpFreq =
            440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + hpParam : hpParam);

        if (hpFreq != hpFreqPrior)
        {
            filters[0].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::HP, hpFreq, 0.5f, VFXConfig::getSampleRateInv(this), 0.f);
            hpFreqPrior = hpFreq;
        }
        else
        {
            filters[0].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    void setCoeffsLowpass(float pitch)
    {
        auto lpParam = this->getFloatParam((int)WaveShaperFloatParams::lowpass);
        auto lpFreq =
            440.f * this->note_to_pitch_ignoring_tuning((keytrackOn) ? pitch + lpParam : lpParam);

        if (lpFreq != lpFreqPrior)
        {
            filters[1].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::LP, lpFreq, 0.5f, VFXConfig::getSampleRateInv(this), 0.f);
            lpFreqPrior = lpFreq;
        }
        else
        {
            filters[1].template retainCoeffForBlock<VFXConfig::blockSize>();
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        bool hpActive = this->getIntParam((int)WaveShaperIntParams::hpOn);
        bool lpActive = this->getIntParam((int)WaveShaperIntParams::lpOn);
        namespace mech = sst::basic_blocks::mechanics;

        checkType();

        if (!mWSOp)
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
            return;
        }

        if (hpActive)
        {
            setCoeffsHighpass(pitch);
            filters[0].template processBlock<VFXConfig::blockSize>(datainL, datainR, dataoutL,
                                                                   dataoutR);
        }
        else
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        }

        processInternal<true>(dataoutL, dataoutR, dataoutL, dataoutR);

        if (lpActive)
        {
            setCoeffsLowpass(pitch);
            filters[1].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL,
                                                                   dataoutR);
        }
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        bool hpActive = this->getIntParam((int)WaveShaperIntParams::hpOn);
        bool lpActive = this->getIntParam((int)WaveShaperIntParams::lpOn);
        namespace mech = sst::basic_blocks::mechanics;

        checkType();

        if (!mWSOp)
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
            return;
        }

        if (hpActive)
        {
            setCoeffsHighpass(pitch);
            filters[0].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutL);
        }
        else
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        }

        processInternal<false>(dataoutL, nullptr, dataoutL, nullptr);

        if (lpActive)
        {
            setCoeffsLowpass(pitch);
            filters[1].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutL);
        }
    }

    void checkType()
    {
        if (this->getIntParam((int)WaveShaperIntParams::type) != mTypeParamVal)
        {
            mTypeParamVal = this->getIntParam((int)WaveShaperIntParams::type);
            mWSType = (sst::waveshapers::WaveshaperType)mTypeParamVal;

            float R[sst::waveshapers::n_waveshaper_registers];
            sst::waveshapers::initializeWaveshaperRegister(mWSType, R);
            for (int i = 0; i < sst::waveshapers::n_waveshaper_registers; ++i)
            {
                mWss.R[i] = _mm_set1_ps(R[i]);
            }
            mWss.init = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_setzero_ps());
            mWSOp = sst::waveshapers::GetQuadWaveshaper(mWSType);
        }
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{false};
    float hpFreqPrior = -9999.f;
    float lpFreqPrior = -9999.f;
    int mTypeParamVal{-1};
    sst::waveshapers::WaveshaperType mWSType;
    sst::waveshapers::QuadWaveshaperState mWss;
    sst::waveshapers::QuadWaveshaperPtr mWSOp{nullptr};
    sst::basic_blocks::dsp::lipol<float, VFXConfig::blockSize, true> mDriveLerp, mBiasLerp,
        mGainLerp;
    std::array<sst::filters::CytomicSVF, 2> filters;
};
} // namespace sst::voice_effects::waveshaper

#endif // SHORTCIRCUITXT_WAVESHAPER_H
