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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_DELAYSUPPORT_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_DELAYSUPPORT_H

#include "sst/basic-blocks/dsp/SSESincDelayLine.h"
#include <cassert>

namespace sst::voice_effects::delay::details
{
struct DelayLineSupport
{
    static constexpr int shortestN{12}, longestN{20};
    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    template <size_t N> using LineN = sst::basic_blocks::dsp::SSESincDelayLine<1 << N>;

    template <size_t N, typename MemoryPoolProvider> void preReserveLines(MemoryPoolProvider *mp)
    {
        mp->preReservePool(sizeof(LineN<N>));
    }

    template <size_t N, typename MemoryPoolProvider>
    void prepareLine(MemoryPoolProvider *mp, const SincTable &st)
    {
        lineBuffer = mp->checkoutBlock(sizeof(LineN<N>));
        auto lp = new (lineBuffer) LineN<N>(st);
        std::get<N - shortestN>(linePointers) = lp;
    }

    template <size_t N, typename MemoryPoolProvoder> void returnLines(MemoryPoolProvoder *mp)
    {
        auto *p = std::get<N - shortestN>(linePointers);
        if (p)
        {
            p->~LineN<N>();
            mp->returnBlock(lineBuffer, sizeof(LineN<N>));
            std::get<N - shortestN>(linePointers) = nullptr;
            lineBuffer = nullptr;
        }
    }

    template <size_t N> LineN<N> *getLinePointer()
    {
        auto res = std::get<N - shortestN>(linePointers);
        assert(res);
        return res;
    }

  protected:
    uint8_t *lineBuffer;

    std::tuple<LineN<12> *, LineN<13> *, LineN<14> *, LineN<15> *, LineN<16> *, LineN<17> *,
               LineN<18> *, LineN<19> *, LineN<20> *>
        linePointers{nullptr, nullptr, nullptr, nullptr, nullptr,
                     nullptr, nullptr, nullptr, nullptr};
};
} // namespace sst::voice_effects::delay::details

#endif // SHORTCIRCUITXT_DELAYSUPPORT_H
