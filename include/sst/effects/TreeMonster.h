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

#ifndef INCLUDE_SST_EFFECTS_TREEMONSTER_H
#define INCLUDE_SST_EFFECTS_TREEMONSTER_H

#include <cstring>
#include <cmath>
#include <utility>

#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"

#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/Clippers.h"

#include "sst/basic-blocks/mechanics/simd-ops.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

#include "sst/basic-blocks/tables/SincTableProvider.h"
#include "sst/effects-shared/TreemonsterCore.h"

namespace sst::effects::treemonster
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig>
struct TreeMonster
    : effects_shared::TreemonsterCore<core::EffectTemplateBase<FXConfig>,
                                      typename core::EffectTemplateBase<FXConfig>::BiquadFilterType>
{
    using parent_t = effects_shared::TreemonsterCore<
        core::EffectTemplateBase<FXConfig>,
        typename core::EffectTemplateBase<FXConfig>::BiquadFilterType>;

    static constexpr int numParams{parent_t::tm_num_ctrls};
    static constexpr const char *streamingName{"treemonster"};
    static constexpr const char *displayName{"Treemonster"};

    TreeMonster(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                typename FXConfig::ValueStorage *p)
        : parent_t(s, e, p)
    {
        static_assert(core::ValidEffect<TreeMonster>);
        this->lp.storage = s;
        this->hp.storage = s;
    }

    // need this for the sifnae check which doesn't see through the CRTP
    void initialize() { parent_t::initialize(); }
    void suspendProcessing() { this->initialize(); }
    int getRingoutDecay() const { return 1000; }
    void onSampleRateChanged() { this->initialize(); }

    void processBlock(float *__restrict L, float *__restrict R)
    {
        this->processWithMixAndWidth(L, R);
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        auto pidx = (typename parent_t::tm_params)idx;
        switch (pidx)
        {
        case parent_t::tm_threshold:
            return pmd().asDecibelWithRange(-96, 0, -24).withName("Threshold");
        case parent_t::tm_speed:
            return pmd().asPercent().withName("Speed").withDefault(0.5f);
        case parent_t::tm_pitch:
            return pmd()
                .asFloat()
                .withRange(-60.f, 60.f)
                .withName("Pitch")
                .withDefault(0.f)
                .withLinearScaleFormatting("keys");
        case parent_t::tm_ring_mix:
            return pmd().asPercent().withDefault(0.5f).withName("Ring Modulation");
        case parent_t::tm_width:
            return pmd().asPercentBipolar().withName("Width");
        case parent_t::tm_mix:
            return pmd().asPercent().withDefault(1.f).withName("Mix");
        case parent_t::tm_lp:
            return pmd().asAudibleFrequency().deactivatable(true).withName("Low Cut");
        case parent_t::tm_hp:
            return pmd().asAudibleFrequency().deactivatable(true).withName("High Cut");
        default:
            break;
        }

        return pmd().withName("ERROR").asPercent();
    }

    size_t silentSamplesLength() const { return FXConfig::blockSize; }

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

} // namespace sst::effects::treemonster
#endif // SURGE_DELAY_H
