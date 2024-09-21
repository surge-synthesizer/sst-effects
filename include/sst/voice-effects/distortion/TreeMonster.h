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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DISTORTION_TREEMONSTER_H
#define INCLUDE_SST_VOICE_EFFECTS_DISTORTION_TREEMONSTER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"
#include "sst/effects-shared/TreemonsterCore.h"

namespace sst::voice_effects::distortion
{
template <typename VFXConfig> struct TreeMonster : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"TreeMonster"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        tmvThreshold,
        tmvSpeed,
        tmvPitch,
        tmvRingMix
    };

    struct BusToVoiceAdapter
    {
        core::VoiceEffectTemplateBase<VFXConfig> *that;
        BusToVoiceAdapter(core::VoiceEffectTemplateBase<VFXConfig> *t) { that = t; }
        using FXConfig_t = VFXConfig;
        float dbToLinear(float f) const { return that->dbToLinear(f); }
        float floatValue(int idx) const;
        bool isDeactivated(int idx) const;
        double sampleRate() const { return that->getSampleRate(); }
    };

    using core_t = effects_shared::TreemonsterCore<
        BusToVoiceAdapter, typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType,
        false>;
    core_t coreProc{this};
    TreeMonster() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~TreeMonster() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case tmvThreshold:
            return pmd().asDecibelWithRange(-96, 0, -24).withName("Threshold");
        case tmvSpeed:
            return pmd().asPercent().withName("Speed").withDefault(0.5f);
        case tmvPitch:
            return pmd()
                .asFloat()
                .withRange(-60.f, 60.f)
                .withName("Pitch")
                .withDefault(0.f)
                .withLinearScaleFormatting("keys");
        case tmvRingMix:
            return pmd().asPercent().withDefault(0.5f).withName("Ring Modulation");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    void initVoiceEffect() { coreProc.initialize(); }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        coreProc.processWithoutMixOrWith(datainL, datainR, dataoutL, dataoutR);
    }

  protected:
    float time[2]{0.f, 0.f}, level[2]{0.f, 0.f};
    float priorFreq = -1.f;
    sst::filters::CytomicSVF filter;
};

template <typename VFXConfig>
inline float TreeMonster<VFXConfig>::BusToVoiceAdapter::floatValue(int idx) const
{
    auto ti = (typename core_t::tm_params)idx;

    switch (ti)
    {
    case core_t::tm_params::tm_threshold:
        return that->getFloatParam(tmvThreshold);
    case core_t::tm_params::tm_speed:
        return that->getFloatParam(tmvSpeed);
    case core_t::tm_params::tm_pitch:
        return that->getFloatParam(tmvPitch);
    case core_t::tm_params::tm_ring_mix:
        return that->getFloatParam(tmvRingMix);
    default:
        break;
    }

    return 0.f;
}
template <typename VFXConfig>
inline bool TreeMonster<VFXConfig>::BusToVoiceAdapter::isDeactivated(int idx) const
{
    auto ti = (typename core_t::tm_params)idx;
    if (ti == core_t::tm_params::tm_lp || ti == core_t::tm_params::tm_hp)
        return true;
    return false;
}
} // namespace sst::voice_effects::distortion

#endif // SHORTCIRCUITXT_BITCRUSHER_H
