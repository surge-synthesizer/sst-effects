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
template <template<int N> typename lineType> struct DelayLineSupport
{
    static constexpr int shortestN{12}, longestN{20};
    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    template <size_t N> using LineN = lineType<1 << N>;

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
    // TODO: enable_if shenanigans
    template <size_t N, typename MemoryPoolProvider>
    void prepareLine(MemoryPoolProvider *mp)
    {
        lineBuffer = mp->checkoutBlock(sizeof(LineN<N>));
        auto lp = new (lineBuffer) LineN<N>();
        std::get<N - shortestN>(linePointers) = lp;
    }

    template <size_t N, typename MemoryPoolProvider> void returnLines(MemoryPoolProvider *mp)
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

    template <size_t N> void clearLine()
    {
        auto res = std::get<N - shortestN>(linePointers);
        if (res)
        {
            res->clear();
        }
    }

    template <size_t N> bool hasLinePointer()
    {
        auto res = std::get<N - shortestN>(linePointers);
        return res != nullptr;
    }

    template <size_t N> LineN<N> *getLinePointer()
    {
        auto res = std::get<N - shortestN>(linePointers);
        assert(res);
        return res;
    }

    /*
     * Runtime Dispatch by Size
     *
     * Example 1 - Calling hasLinePointer<N>():
     *   bool result = dispatch(runtimeN, [this](auto N_val) {
     *       return hasLinePointer<N_val>();
     *   });
     *
     * Example 2 - Calling prepareLine<N>(mp, st):
     *   dispatch(runtimeN, [this](auto N_val, auto* mp, auto& st) {
     *       return prepareLine<N_val>(mp, st);
     *   }, mp, st);
     *
     * Example 3 - Calling returnLines<N>(mp):
     *   dispatch(runtimeN, [this](auto N_val, auto* mp) {
     *       return returnLines<N_val>(mp);
     *   }, mp);
     *
     * Example 4 - Using getLinePointer<N>() (which returns different pointer types):
     *   dispatch(runtimeN, [&](auto N_val) {
     *       auto* line = getLinePointer<N_val>();
     *       // Use the line pointer directly in the lambda body
     *       processWithLine(line);
     *   });
     *   // The lambda returns void, avoiding return type deduction issues
     *
     * Note: N_val is std::integral_constant<size_t, N>, use it directly as a template argument
     * Note: For functions returning different types per N (e.g., getLinePointer), call them
     *       inside the lambda and use the result there, rather than returning from the lambda.
     */
    template <typename Func, typename... Args> auto dispatch(size_t N, Func func, Args &&...args)
    {
        return dispatchImpl<shortestN>(N, func, std::forward<Args>(args)...);
    }

    template <size_t CurrentN, typename Func, typename... Args>
    auto dispatchImpl(size_t N, Func &&func, Args &&...args)
    {
        if (N == CurrentN)
        {
            return func(std::integral_constant<size_t, CurrentN>{}, std::forward<Args>(args)...);
        }
        else if constexpr (CurrentN < longestN)
        {
            return dispatchImpl<CurrentN + 1>(N, std::forward<Func>(func),
                                              std::forward<Args>(args)...);
        }
        else
        {
            assert(false && "N value out of range for dispatch");
            using DeducedReturnType = decltype(func(std::integral_constant<size_t, CurrentN>{},
                                                    std::forward<Args>(args)...));
            if constexpr (!std::is_void_v<DeducedReturnType>)
                return DeducedReturnType{};
        }
    }

    void returnAllExcept(size_t N, auto *mp)
    {
        for (int i = shortestN; i <= longestN; ++i)
        {
            if (i != N)
            {
                dispatch(i, [this, mp, i](auto NV) {
                    if (hasLinePointer<NV>())
                    {
                        returnLines<NV>(mp);
                    }
                });
            }
        }
    }

    void returnAll(auto *mp)
    {
        for (int i = shortestN; i <= longestN; ++i)
        {
            dispatch(i, [this, mp, i](auto NV) {
                if (hasLinePointer<NV>())
                {
                    returnLines<NV>(mp);
                }
            });
        }
    }

    void reservePrepareAndClear(size_t Nrt, auto *mp, const SincTable &sSincTable)
    {
        dispatch(Nrt, [this, mp, &sSincTable](auto N) {
            if (!hasLinePointer<N>())
            {
                preReserveLines<N>(mp);
                prepareLine<N>(mp, sSincTable);
            }
            clearLine<N>();
        });
    }
    void reservePrepareAndClear(size_t Nrt, auto *mp)
    {
        dispatch(Nrt, [this, mp](auto N) {
            if (!hasLinePointer<N>())
            {
                preReserveLines<N>(mp);
                prepareLine<N>(mp);
            }
            clearLine<N>();
        });
    }

  protected:
    uint8_t *lineBuffer{nullptr};

    std::tuple<LineN<12> *, LineN<13> *, LineN<14> *, LineN<15> *, LineN<16> *, LineN<17> *,
               LineN<18> *, LineN<19> *, LineN<20> *>
        linePointers{nullptr, nullptr, nullptr, nullptr, nullptr,
                     nullptr, nullptr, nullptr, nullptr};
};

struct quadDelayLineSupport
{
  protected:
    uint8_t *lineBuffer{nullptr};
};
} // namespace sst::voice_effects::delay::details

#endif // SHORTCIRCUITXT_DELAYSUPPORT_H
