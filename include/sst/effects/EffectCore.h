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

#ifndef INCLUDE_SST_EFFECTS_EFFECTCORE_H
#define INCLUDE_SST_EFFECTS_EFFECTCORE_H

/*
 * The sst-effects provides robust effects which are adaptable to a variety of situations, but
 * that adaptation requires some indirection. We chose a template strategy here to avoid virtual
 * functions and to still retain compile time checking of interface compliance.
 *
 * The core design of an sst-effect is
 *
 * 1. It provides stereo processing on a block
 * 2. It has parameters with values and features and
 * 3. It runs in an environment which can answer questions and provide values about those
 *    features and runtime features (like sample rate)
 *
 * To do that, each sst-effect has a single template parameter, `FXConfig`, which provides
 * the ability to answer questions and provide types, and has a small non-virtual interface
 * it provides to clients.
 *
 * The FX provided interface is
 *
 * 1. A 3 arg constructor
 * 2. A member for numParams and effect name
 * 3. An initialize, suspennd and ringout decay method and
 * 4. a process method
 *
 * The regtest constraints in the tests/check-compile tester document this api.
 *
 * The FX Configuration provides type definitions for the core type and then methods
 * which allow you to probe it. The core types are a Global storage (things like sample
 * rate and tuning) and Effect storage (thigns like configuration of params) and an
 * Value storage (things like what is the float value or int value of a particular param
 * at this moment). It also provides a block size, since all surge effects work on a power
 * of 2 block of at least size 4.
 *
 * We make no assumptions about these classes other than (1) they have pointer semantics
 * and are owned outside us longer than us and (2) they can be passed as pointers to a template
 * type which then probes them.
 *
 * That template type is `FXConfig`. You can see in this base class the questions we
 * ask of it, but for instance, to get the value of a particular param we ask
 * `FXConfig::floatValueAt(ValueStorage *, int idx)` and to see if an item is
 * tempo synced we ask `FXConfig::tempoSyncRation(GlobalStorage*, FXStorage *, int)`.
 * This forms an API which means we can be neutral as to how these three bundles work.
 *
 * With that in hand, you can refactor the effects to match this interface and then
 * present them in a variety of context. The test runners here provide two basic
 * contexts and Surge currently provides a third. One day soon, ShortCircuit will
 * provide a fourth.
 *
 * There's still some big TODOs here especially around param metadata. Still a bit of
 * a work in flight right now.
 *
 */

// For shared width calculations
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/filters/BiquadFilter.h"

#include "EffectCoreDetails.h"

#include <type_traits>

namespace sst::effects::core
{
// Todo: as we port consider this FXConfig::BaseClass being a bit more configurable.
template <typename FXConfig> struct EffectTemplateBase : public FXConfig::BaseClass
{
    static_assert(std::is_integral<decltype(FXConfig::blockSize)>::value);
    static_assert(!(FXConfig::blockSize & (FXConfig::blockSize - 1))); // 2^n
    static_assert(FXConfig::blockSize >= 4);                           // > simd register length
    static_assert(std::is_class<typename FXConfig::BaseClass>::value);
    static_assert(std::is_pointer<typename FXConfig::GlobalStorage *>::value);
    static_assert(std::is_pointer<typename FXConfig::EffectStorage *>::value);
    static_assert(std::is_class<typename FXConfig::BiquadAdapter>::value);
    static_assert(std::is_pointer<typename FXConfig::ValueStorage *>::value);
    static_assert(std::is_same<decltype(FXConfig::floatValueAt),
                               float(const typename FXConfig::BaseClass *const,
                                     const typename FXConfig::ValueStorage *const, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::intValueAt),
                               int(const typename FXConfig::BaseClass *const,
                                   const typename FXConfig::ValueStorage *const, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::envelopeRateLinear),
                               float(typename FXConfig::GlobalStorage *, float)>::value);
    static_assert(std::is_same<decltype(FXConfig::temposyncRatio),
                               float(typename FXConfig::GlobalStorage *,
                                     typename FXConfig::EffectStorage *, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::isDeactivated),
                               bool(typename FXConfig::EffectStorage *, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::isExtended),
                               bool(typename FXConfig::EffectStorage *, int)>::value);
    static_assert(
        std::is_same<decltype(FXConfig::rand01), float(typename FXConfig::GlobalStorage *)>::value);
    static_assert(std::is_same<decltype(FXConfig::sampleRate),
                               double(typename FXConfig::GlobalStorage *)>::value);
    static_assert(std::is_same<decltype(FXConfig::noteToPitch),
                               float(typename FXConfig::GlobalStorage *, float)>::value);
    static_assert(std::is_same<decltype(FXConfig::noteToPitchIgnoringTuning),
                               float(typename FXConfig::GlobalStorage *, float)>::value);
    static_assert(std::is_same<decltype(FXConfig::noteToPitchInv),
                               float(typename FXConfig::GlobalStorage *, float)>::value);
    static_assert(std::is_same<decltype(FXConfig::dbToLinear),
                               float(typename FXConfig::GlobalStorage *, float)>::value);

    typename FXConfig::GlobalStorage *globalStorage{nullptr};
    typename FXConfig::EffectStorage *fxStorage{nullptr};
    typename FXConfig::ValueStorage *valueStorage{nullptr};

    EffectTemplateBase(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                       typename FXConfig::ValueStorage *p)
        : FXConfig::BaseClass(s, e, p), globalStorage(s), fxStorage(e), valueStorage(p)
    {
    }

    using lipol_ps_blocksz = sst::basic_blocks::dsp::lipol_sse<FXConfig::blockSize, false>;
    static constexpr float blockSize_inv{1.f / FXConfig::blockSize};
    static constexpr float blockSize_quad{FXConfig::blockSize >> 2};

    using BiquadFilterType =
        sst::filters::Biquad::BiquadFilter<typename FXConfig::GlobalStorage, FXConfig::blockSize,
                                           typename FXConfig::BiquadAdapter>;

    inline typename FXConfig::BaseClass *asBase()
    {
        return static_cast<typename FXConfig::BaseClass *>(this);
    }

    inline typename FXConfig::BaseClass *const asBase() const
    {
        return static_cast<typename FXConfig::BaseClass *const>(this);
    }

    inline float floatValue(int idx) const
    {
        return FXConfig::floatValueAt(asBase(), valueStorage, idx);
    }

    inline float floatValueExtended(int idx) const
    {
        if constexpr (details::Has_floatValueExtendedAt<FXConfig>::value)
        {
            return FXConfig::floatValueExtendedAt(asBase(), valueStorage, idx);
        }
        else
        {
            return FXConfig::floatValueAt(asBase(), valueStorage, idx);
        }
    }

    inline float intValue(int idx) const
    {
        return FXConfig::intValueAt(asBase(), valueStorage, idx);
    }

    inline float temposyncRatio(int idx) const
    {
        return FXConfig::temposyncRatio(globalStorage, fxStorage, idx);
    }

    inline float temposyncRatioInv(int idx) const
    {
        if constexpr (details::Has_temposyncRatioInv<FXConfig>::value)
        {
            return FXConfig::temposyncRatioInv(globalStorage, fxStorage, idx);
        }
        else
        {
            return 1.f / temposyncRatio(idx);
        }
    }

    inline bool temposyncInitialized() const
    {
        if constexpr (details::Has_temposyncInitialized<FXConfig>::value)
            return FXConfig::temposyncInitialized(globalStorage);
        else
            return true; // assume folks who support it will have it
    }

    inline bool isDeactivated(int idx) const { return FXConfig::isDeactivated(fxStorage, idx); }
    inline bool isExtended(int idx) const { return FXConfig::isExtended(fxStorage, idx); }
    inline int deformType(int idx) const
    {
        if constexpr (details::Has_deformType<FXConfig>::value)
        {
            return FXConfig::deformType(fxStorage, idx);
        }
        else
        {
            return 0;
        }
    }

    inline float envelopeRateLinear(float f) const
    {
        return FXConfig::envelopeRateLinear(globalStorage, f);
    }

    inline float storageRand01() { return FXConfig::rand01(globalStorage); }

    inline double sampleRate() { return FXConfig::sampleRate(globalStorage); }

    inline float noteToPitch(float p) { return FXConfig::noteToPitch(globalStorage, p); }

    inline float noteToPitchIgnoringTuning(float p)
    {
        return FXConfig::noteToPitchIgnoringTuning(globalStorage, p);
    }

    inline float noteToPitchInv(float p) { return FXConfig::noteToPitchInv(globalStorage, p); }

    inline float dbToLinear(float f) { return FXConfig::dbToLinear(globalStorage, f); }

    template <typename lipol>
    inline void applyWidth(float *__restrict L, float *__restrict R, lipol &width)
    {
        namespace sdsp = sst::basic_blocks::dsp;
        float M alignas(16)[FXConfig::blockSize], S alignas(16)[FXConfig::blockSize];
        sdsp::encodeMS<FXConfig::blockSize>(L, R, M, S);
        width.multiply_block(S, FXConfig::blockSize >> 2);
        sdsp::decodeMS<FXConfig::blockSize>(M, S, L, R);
    }

    static constexpr int slowrate{8}, slowrate_m1{slowrate - 1};
};
} // namespace sst::effects::core

#endif