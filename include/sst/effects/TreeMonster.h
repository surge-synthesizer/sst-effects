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

#ifndef INCLUDE_SST_EFFECTS_TM_H
#define INCLUDE_SST_EFFECTS_TM_H

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

template <typename FXConfig> struct TreeMonster : effects_shared::TreemonsterCore<core::EffectTemplateBase<FXConfig>>
{
    using parent_t = effects_shared::TreemonsterCore<core::EffectTemplateBase<FXConfig>>;

    static constexpr int numParams{parent_t::tm_num_ctrls};
    static constexpr const char *effectName{"treemonster"};

    TreeMonster(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
          typename FXConfig::ValueStorage *p)
        : parent_t(s, e, p)
    {

    }

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return 1000; }
    void onSampleRateChanged() { initialize(); }

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd().withName("ERROR").asPercent();
    }

  protected:

    void setvars(bool b);
};

template <typename FXConfig> inline void TreeMonster<FXConfig>::initialize()
{
}

template <typename FXConfig> inline void TreeMonster<FXConfig>::setvars(bool init)
{

}

template <typename FXConfig> inline void TreeMonster<FXConfig>::processBlock(float *dataL, float *dataR)

{

}
} // namespace sst::effects::delay
#endif // SURGE_DELAY_H
