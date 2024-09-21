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
    static constexpr const char *effectName{"EQ 6 Band"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{0};

    EqGraphic6Band() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
        std::fill(mParametric.begin(), mParametric.end(), this);
    }

    ~EqGraphic6Band() {}

    static constexpr int nBands{6};
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

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        calc_coeffs();
        auto *inL = datainL;
        auto *inR = datainR;
        for (int i = 0; i < nBands; ++i)
        {
            mParametric[i].process_block_to(inL, inR, dataoutL, dataoutR);
            inL = dataoutL;
            inR = dataoutR;
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();
        auto *inL = datainL;
        for (int i = 0; i < nBands; ++i)
        {
            mParametric[i].process_block_to(inL, dataoutL);
            inL = dataoutL;
        }
    }

    void calc_coeffs()
    {
        std::array<float, nBands * 3> param;
        std::array<int, nBands> iparam;
        bool diff{false};
        for (int i = 0; i < nBands * 3; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }

        if (diff)
        {
            double a = M_PI_2 * this->getSampleRateInv();
            const double bw = 3;
            for (int i = 0; i < nBands; ++i)
                mParametric[i].coeff_peakEQ(bands[i] * a, bw, param[i]);

            mLastParam = param;
        }
    }

    float getFrequencyGraph(float f)
    {
        auto res = 1.f;
        for (int i = 0; i < nBands; ++i)
        {
            res *= mParametric[i].plot_magnitude(f);
        }
        return res;
    }

    float getBandFrequencyGraph(int band, float f) { return getFrequencyGraph(f); }

  protected:
    std::array<float, nBands * 3> mLastParam{};
    std::array<typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType, nBands>
        mParametric;
};

} // namespace sst::voice_effects::eq

#endif // SHORTCIRCUITXT_EqNBandParametric_H
