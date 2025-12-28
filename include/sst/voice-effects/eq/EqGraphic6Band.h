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

#ifndef INCLUDE_SST_VOICE_EFFECTS_EQ_EQGRAPHIC6BAND_H
#define INCLUDE_SST_VOICE_EFFECTS_EQ_EQGRAPHIC6BAND_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::eq
{
template <typename VFXConfig> struct EqGraphic6Band : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"6 Band Graphic"};
    static constexpr const char *streamingName{"eq-grp-6"};

    static constexpr int nBands{6};
    static constexpr int numFloatParams{nBands};
    static constexpr int numIntParams{0};

    static constexpr float resonance{0.2f};

    EqGraphic6Band() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~EqGraphic6Band() {}

    static constexpr std::array<float, nBands> bands{100, 250, 630, 1600, 5000, 12000};
    static constexpr std::array<const char *, nBands> labels{"100Hz",  "250Hz", "630Hz",
                                                             "1.6kHz", "5kHz",  "12kHz"};

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        return pmd()
            .asFloat()
            .withRange(-24, 24)
            .withDefault(0)
            .withLinearScaleFormatting("db")
            .withName(labels[idx]);
    }

    void initVoiceEffect()
    {
        for (int i = 0; i < nBands; ++i)
        {
            mParametric[i].init();
        }
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        calc_coeffs();
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        for (int i = 0; i < nBands; ++i)
        {
            mParametric[i].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL,
                                                                       dataoutR);
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);

        for (int i = 0; i < nBands; ++i)
        {
            mParametric[i].template processBlock<VFXConfig::blockSize>(dataoutL, dataoutL);
        }
    }

    void calc_coeffs()
    {
        using md = sst::filters::CytomicSVF::Mode;
        md mode;

        for (int i = 0; i < nBands; ++i)
        {
            if (mLastParam[i] != this->getFloatParam(i))
            {
                mLastParam[i] = this->getFloatParam(i);

                if (i == 0)
                {
                    mode = md::LowShelf;
                }
                else if (i == nBands - 1)
                {
                    mode = md::HighShelf;
                }
                else
                {
                    mode = md::Bell;
                }

                mParametric[i].template setCoeffForBlock<VFXConfig::blockSize>(
                    mode, bands[i], resonance, this->getSampleRateInv(),
                    this->dbToLinear(this->getFloatParam(i) / 2));
            }
            else
            {
                mParametric[i].template retainCoeffForBlock<VFXConfig::blockSize>();
            }
        }
    }

    float getFrequencyGraph(float f)
    {
        using md = sst::filters::CytomicSVF::Mode;
        md mode;

        auto res = 1.f;

        for (int i = 0; i < nBands; ++i)
        {
            if (i == 0)
            {
                mode = md::LowShelf;
            }
            else if (i == nBands - 1)
            {
                mode = md::HighShelf;
            }
            else
            {
                mode = md::Bell;
            }

            res *= sst::filters::CytomicSVFGainAt(mode, bands[i], resonance,
                                                  this->dbToLinear(this->getFloatParam(i) / 2),
                                                  f * this->getSampleRate());
        }
        return res;
    }

    float getBandFrequencyGraph(int band, float f) { return getFrequencyGraph(f); }

  protected:
    std::array<float, nBands> mLastParam{};
    std::array<sst::filters::CytomicSVF, nBands> mParametric;

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // We did update this one, but the original had the wrong frequencies entirely
        // and there's not really any way to stream that.
        assert(streamedFrom == 1);
    }
};

} // namespace sst::voice_effects::eq

#endif // SHORTCIRCUITXT_EqNBandParametric_H
