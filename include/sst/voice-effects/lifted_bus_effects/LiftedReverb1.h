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

#ifndef INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_LIFTEDREVERB1_H
#define INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_LIFTEDREVERB1_H

#include "../VoiceEffectCore.h"
#include "sst/effects/Reverb1.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "FXConfigFromVFXConfig.h"

namespace sst::voice_effects::liftbus
{

namespace mech = sst::basic_blocks::mechanics;
template <typename VFXConfig> struct LiftedReverb1 : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Reverb1"};

    static constexpr int numFloatParams{8};
    static constexpr int numIntParams{1};

    using reverb1_t =
        sst::effects::reverb1::Reverb1<FXConfigFromVFXConfig<LiftedReverb1<VFXConfig>>>;
    LiftHelper<LiftedReverb1, reverb1_t> helper;

    LiftedReverb1() : helper(this), core::VoiceEffectTemplateBase<VFXConfig>() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        auto udx = idx;
        if (idx > 0) // we skip shape
            udx = idx + 1;
        return helper.busFX->paramAt(udx);
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        return helper.busFX->paramAt(reverb1_t::rev1_shape);
    }

    void initVoiceEffect() { helper.init(); }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    size_t tailLength() const { return helper.busFX->getRingoutDecay() * VFXConfig::blockSize; }

    void setupValues()
    {
        for (int i = 0; i < numFloatParams; ++i)
        {
            auto udx = i;
            if (i > 0) // we skip shape
                udx = i + 1;
            helper.valuesForFX[udx] = this->getFloatParam(i);
        }
        helper.valuesForFX[reverb1_t::rev1_shape] = this->getIntParam(0);

        helper.valuesForFX[reverb1_t::rev1_mix] = 1.0;
        helper.valuesForFX[reverb1_t::rev1_width] = helper.busFX->getDefaultWidth();
    }
    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        setupValues();
        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        helper.busFX->processBlock(dataoutL, dataoutR);
    }

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
} // namespace sst::voice_effects::liftbus
#endif // SHORTCIRCUITXT_LIFTEDREVERB2_H
