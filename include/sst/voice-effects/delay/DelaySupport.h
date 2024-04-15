//
// Created by Paul Walker on 4/15/24.
//

#ifndef SHORTCIRCUITXT_DELAYSUPPORT_H
#define SHORTCIRCUITXT_DELAYSUPPORT_H

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
