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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITIES_GAINMATRIX_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITIES_GAINMATRIX_H

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/PanLaws.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::utilities
{
template <typename VFXConfig> struct GainMatrix : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Gain Matrix"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpLeftToLeft,
        fpLeftToRight,
        fpRightToLeft,
        fpRightToRight
    };

    enum IntParams
    {
    };

    GainMatrix() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~GainMatrix() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpLeftToLeft:
            return pmd().withDefault(1.f).withRange(-1.f, 1.f).withName("L -> L");
        case fpLeftToRight:
            return pmd().withDefault(0.f).withRange(-1.f, 1.f).withName("L -> R");
        case fpRightToLeft:
            return pmd().withDefault(0.f).withRange(-1.f, 1.f).withName("R -> L");
        case fpRightToRight:
            return pmd().withDefault(1.f).withRange(-1.f, 1.f).withName("R -> R");
        }
        return pmd().asFloat().withName("Error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto sL = datainL[i];
            auto sR = datainR[i];

            dataoutL[i] =
                sL * this->getFloatParam(fpLeftToLeft) + sR * this->getFloatParam(fpRightToLeft);
            dataoutR[i] =
                sR * this->getFloatParam(fpRightToRight) + sL * this->getFloatParam(fpLeftToRight);
        }
    }

  protected:
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> volLerp, leftLerp, rightLerp;
};
} // namespace sst::voice_effects::utilities
#endif // SCXT_GAINMATRIX_H
