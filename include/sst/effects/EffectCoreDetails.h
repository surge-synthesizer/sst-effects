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

#ifndef INCLUDE_SST_EFFECTS_EFFECTCOREDETAILS_H
#define INCLUDE_SST_EFFECTS_EFFECTCOREDETAILS_H

#include <functional>
#include <type_traits>
#include <iostream>

namespace sst::effects::details
{

// Thanks Jatin!
// https://github.com/Chowdhury-DSP/chowdsp_utils/blob/master/modules/common/chowdsp_core/Types/chowdsp_TypeTraits.h#L12

#define HAS_MEMBER(M)                                                                              \
    template <typename T> class Has_##M                                                            \
    {                                                                                              \
        using No = uint8_t;                                                                        \
        using Yes = uint64_t;                                                                      \
        static_assert(sizeof(No) != sizeof(Yes));                                                  \
        template <typename C> static Yes test(decltype(C::M) *);                                   \
        template <typename C> static No test(...);                                                 \
                                                                                                   \
      public:                                                                                      \
        enum                                                                                       \
        {                                                                                          \
            value = sizeof(test<T>(nullptr)) == sizeof(Yes)                                        \
        };                                                                                         \
    };

HAS_MEMBER(floatValueExtendedAt);
HAS_MEMBER(temposyncInitialized);
HAS_MEMBER(temposyncRatioInv);
HAS_MEMBER(deformType);

} // namespace sst::effects::details

#endif // SURGE_EFFECTCOREDETAILS_H
