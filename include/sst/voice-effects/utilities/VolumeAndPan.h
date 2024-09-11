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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITIES_VOLUMEANDPAN_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITIES_VOLUMEANDPAN_H

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/PanLaws.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::utilities
{
template <typename VFXConfig> struct VolumeAndPan : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Vol/Pan Utility"};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpVolume,
        fpPan,
    };

    enum IntParams
    {
    };

    VolumeAndPan() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~VolumeAndPan() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpVolume:
            return pmd().asLinearDecibel(-60.f, 24.f).withDefault(0.f).withName("Volume");
        case fpPan:
            return pmd()
                .asPercentBipolar()
                .withCustomMinDisplay("L")
                .withCustomMaxDisplay("R")
                .withCustomDefaultDisplay("C")
                .withDefault(0.f)
                .withName("Pan");
        }
        return pmd().asFloat().withName("Error");
    }

    //    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    //    {
    //        using pmd = basic_blocks::params::ParamMetaData;
    //        return pmd().asInt().withName("Error");
    //    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto pan = (this->getFloatParam(fpPan) + 1) / 2;
        basic_blocks::dsp::pan_laws::panmatrix_t pmat{1, 1, 0, 0};
        basic_blocks::dsp::pan_laws::stereoEqualPower(pan, pmat);

        leftLerp.set_target(pmat[0]);
        rightLerp.set_target(pmat[1]);

        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        leftLerp.multiply_block(dataoutL);
        rightLerp.multiply_block(dataoutR);

        volLerp.set_target(this->dbToLinear(this->getFloatParam(fpVolume)));
        volLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

  protected:
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> volLerp, leftLerp, rightLerp;
};
} // namespace sst::voice_effects::utilities
#endif // SCXT_VOLUMEANDPAN_H
