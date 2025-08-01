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

#ifndef INCLUDE_SST_EFFECTS_SHARED_WIDTHPROVIDER_H
#define INCLUDE_SST_EFFECTS_SHARED_WIDTHPROVIDER_H

#include "sst/basic-blocks/dsp/MidSide.h"

namespace sst::effects_shared
{
template <typename T, size_t blockSize, bool useGetFloatValue = false> struct WidthProvider
{
    T *asT() { return static_cast<T *>(this); }

    template <typename lipol>
    inline void applyWidth(float *__restrict L, float *__restrict R, lipol &widthS, lipol &widthM)
    {
        namespace sdsp = sst::basic_blocks::dsp;
        float M alignas(16)[blockSize], S alignas(16)[blockSize];
        sdsp::encodeMS<blockSize>(L, R, M, S);
        widthS.multiply_block(S, blockSize >> 2);
        if constexpr (T::useLinearWidth())
            widthM.multiply_block(M, blockSize >> 2);

        sdsp::decodeMS<blockSize>(M, S, L, R);
    }

    basic_blocks::params::ParamMetaData getWidthParam() const
    {
        auto res = basic_blocks::params::ParamMetaData().withName("Width");
        if constexpr (!T::useLinearWidth())
            return res.asDecibelNarrow().withDefault(0.f);
        else
            return res.asPercentBipolar().withRange(-2.f, 2.f).withDefault(1.f);
    }

    int getDefaultWidth() const
    {
        if constexpr (!T::useLinearWidth())
            return 0.f;
        else
            return 1.f;
    }

    inline float internalValue(int idx)
    {
        if constexpr (useGetFloatValue)
            return asT()->getFloatParam(idx);
        else
            return asT()->floatValue(idx);
    }
    template <typename lipol>
    void setWidthTarget(lipol &widthS, lipol &widthM, int idx, float scale = 1.f)
    {
        if constexpr (!T::useLinearWidth())
            widthS.set_target_smoothed(asT()->dbToLinear(internalValue(idx)) * scale);
        else
        {
            auto iv = internalValue(idx);
            widthS.set_target_smoothed(iv);

            // basically up the mid by up to 2x
            auto mv = 1.0f / (std::max(0.5f, std::fabs(iv)));
            widthM.set_target_smoothed(mv);
        }
    }
};
} // namespace sst::effects_shared
#endif // WIDTHPROVIDER_H
