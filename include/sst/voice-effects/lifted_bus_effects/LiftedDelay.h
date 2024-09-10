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

#ifndef INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_LIFTEDDELAY_H
#define INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_LIFTEDDELAY_H

#include "../VoiceEffectCore.h"
#include "sst/effects/Delay.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "FXConfigFromVFXConfig.h"

namespace sst::voice_effects::liftbus
{

namespace mech = sst::basic_blocks::mechanics;
template <typename VFXConfig> struct LiftedDelay : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Delay"};

    static constexpr int numFloatParams{8};
    static constexpr int numIntParams{0};

    using delay_t = sst::effects::delay::Delay<FXConfigFromVFXConfig<LiftedDelay<VFXConfig>>>;
    LiftHelper<LiftedDelay, delay_t> helper;

    LiftedDelay() : helper(this), core::VoiceEffectTemplateBase<VFXConfig>() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        return helper.busFX->paramAt(idx);
    }

    void setupValues()
    {
        for (int i = 0; i < numFloatParams; ++i)
        {
            helper.valuesForFX[i] = this->getFloatParam(i);
        }
        helper.valuesForFX[delay_t::dly_width] = helper.busFX->getDefaultWidth();

        helper.valuesForFX[delay_t::dly_reserved] = 0.f;
        helper.valuesForFX[delay_t::dly_mix] = 1.f;
        helper.valuesForFX[delay_t::dly_input_channel] = 0.f;
    }

    void initVoiceEffect() { helper.init(); }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    size_t tailLength() const
    {
        return -1;
        // return helper.busFX->getRingoutDecay() * VFXConfig::blockSize;
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        setupValues();
        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        helper.busFX->processBlock(dataoutL, dataoutR);
    }
};
} // namespace sst::voice_effects::liftbus
#endif // SHORTCIRCUITXT_LIFTEDREVERB2_H
