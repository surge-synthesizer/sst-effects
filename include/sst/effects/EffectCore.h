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
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/filters/BiquadFilter.h"
#include "sst/effects-shared/WidthProvider.h"

#include "EffectCoreDetails.h"

#include <type_traits>
#include <concepts>

static_assert(__cplusplus >= 202002L, "Surge team libraries have moved to C++ 20");

namespace sst::effects::core
{

template <int S>
concept ValidBlockSize = (S & (S - 1)) == 0 && S >= 4;

template <typename FXConfig>
concept ValidEffectConfiguration =
    requires {
        // required constants and types
        { FXConfig::blockSize } -> std::convertible_to<const int>;
        typename FXConfig::BaseClass;
        typename FXConfig::GlobalStorage;
        typename FXConfig::EffectStorage;
        typename FXConfig::ValueStorage;
        typename FXConfig::BiquadAdapter;
    } && ValidBlockSize<FXConfig::blockSize> && std::is_class_v<typename FXConfig::BaseClass> &&
    std::is_class_v<typename FXConfig::BiquadAdapter> &&
    requires(typename FXConfig::BaseClass *bc, const typename FXConfig::BaseClass *const cbc,
             typename FXConfig::GlobalStorage *gs, typename FXConfig::EffectStorage *es,
             typename FXConfig::ValueStorage *vs, int idx, float f) {
        // value accessors
        { FXConfig::floatValueAt(cbc, vs, idx) } -> std::convertible_to<float>;
        { FXConfig::intValueAt(cbc, vs, idx) } -> std::convertible_to<int>;
        // rate and sync
        { FXConfig::envelopeRateLinear(gs, f) } -> std::convertible_to<float>;
        { FXConfig::temposyncRatio(gs, es, idx) } -> std::convertible_to<float>;
        // flags
        { FXConfig::isDeactivated(es, idx) } -> std::convertible_to<bool>;
        { FXConfig::isExtended(es, idx) } -> std::convertible_to<bool>;
        // environment
        { FXConfig::rand01(gs) } -> std::convertible_to<float>;
        { FXConfig::sampleRate(gs) } -> std::convertible_to<double>;
        { FXConfig::noteToPitch(gs, f) } -> std::convertible_to<float>;
        { FXConfig::noteToPitchIgnoringTuning(gs, f) } -> std::convertible_to<float>;
        { FXConfig::noteToPitchInv(gs, f) } -> std::convertible_to<float>;
        { FXConfig::dbToLinear(gs, f) } -> std::convertible_to<float>;
    };

// Requirements for a concrete, instantiated effect type T
template <typename T>
concept ValidEffect = requires(T t, const T ct, int idx, float *L, float *R) {
    // static members
    { T::streamingName } -> std::convertible_to<const char *>;
    { T::displayName } -> std::convertible_to<const char *>;
    { T::numParams };
    // methods
    { t.initialize() } -> std::same_as<void>;
    { t.processBlock(L, R) } -> std::same_as<void>;
    { t.suspendProcessing() } -> std::same_as<void>;
    { ct.getRingoutDecay() } -> std::convertible_to<int>;
    { ct.paramAt(idx) };
    { t.onSampleRateChanged() } -> std::same_as<void>;
};

template <ValidEffectConfiguration FXConfig>
struct EffectTemplateBase
    : public FXConfig::BaseClass,
      effects_shared::WidthProvider<EffectTemplateBase<FXConfig>, FXConfig::blockSize>
{
    using FXConfig_t = FXConfig;

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

    inline const typename FXConfig::BaseClass *const asBase() const
    {
        return static_cast<const typename FXConfig::BaseClass *const>(this);
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

    inline int intValue(int idx) const { return FXConfig::intValueAt(asBase(), valueStorage, idx); }

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

    inline double sampleRate() const { return FXConfig::sampleRate(globalStorage); }

    inline float noteToPitch(float p) { return FXConfig::noteToPitch(globalStorage, p); }

    inline float noteToPitchIgnoringTuning(float p)
    {
        return FXConfig::noteToPitchIgnoringTuning(globalStorage, p);
    }

    inline float noteToPitchInv(float p) { return FXConfig::noteToPitchInv(globalStorage, p); }

    inline float dbToLinear(float f) { return FXConfig::dbToLinear(globalStorage, f); }

    static constexpr int slowrate{8}, slowrate_m1{slowrate - 1};

    static constexpr bool useLinearWidth()
    {
        if constexpr (details::Has_widthIsLinear<FXConfig>::value)
        {
            return FXConfig::widthIsLinear;
        }
        else
        {
            return false;
        }
    }

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};
} // namespace sst::effects::core

#endif