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
 * OK so here's the deal
 *
 * in basic-blocks::params we have parameter medatadat. its just a class describing a param
 * and pretty boring. The core is really here
 *
 * We say that all fx have a global storage, which answers questions like sample rate,
 * an fx storage, which answers questions about particular confiugration, and a value storage
 * which answers questions about actual float or int values. We make no assumptions about these
 * classes other than (1) they have pointer semantics and are owned outside us longer than us and
 * (2) they can be passed as pointers to a template type which then probes them.
 *
 * That template type is `FXConfig`. You can see in this base class the questions we
 * ask of it, but for instance, to get the value of a particular param we ask
 * `FXConfig::floatValueAt(ValueStorage *, int idx)` and to see if an item is
 * tempo synced we ask `FXConfig::tempoSyncRation(GlobalStorage*, FXStorage *, int)`.
 * This forms an API which means we can be neutral as to how these three bundles work.
 *
 * WIth that in hand, we can rewrite the flanger. Check out Flanger.h. It's pretty obvious
 * but basically (1) advertise param metadata, (2) handle process, init, etc.. and
 * (3) use the FXConfig to probe values and state.
 *
 * Now once we have that we have to go back to surge. The next header you want to
 * read ins src/common/dsp/effects/SurgeSSTFXAdapter.h. This gives us two things
 *
 * 1. It gives us an instance of FXCOnfig with the appropriate typedefs and methods and
 * 2. It gives us a base class which inherits from the virtual base class Effect and
 *    also inherits from a T, which is intended to be something the shape of Flanger.
 *
 *  WIth those two inheritances it can then implement a vast subset of Effect in a
 *  generic fashion just redirecting to the template.
 *
 *  And finally, you retain FlangerEffect.h and FLangerEfffect.cpp but these are massively
 *  reduced. They basically implement only the things which are surge specific. For
 *  instance the group pos api remains. And init_ctrltypes is still there with the old
 *  surge parameter, but we check that those parameters are configured to be consisten
 *  tiwht the metadata (obvioulsy more to do there). See the 'configureControlsFromFXMetadata`
 *  call at the end of init_ctrltypes.
 *
 *  And voila. Most of the effect is reuasable and decoupled.
 *
 */

// For shared width calculations
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/filters/BiquadFilter.h"
#include <type_traits>

namespace sst::effects
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
    static_assert(std::is_same<decltype(FXConfig::temposyncRatio),
                               float(typename FXConfig::GlobalStorage *,
                                     typename FXConfig::EffectStorage *, int)>::value);

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

    inline float intValue(int idx) const
    {
        return FXConfig::intValueAt(asBase(), valueStorage, idx);
    }

    inline float temposyncRatio(int idx) const
    {
        return FXConfig::temposyncRatio(globalStorage, fxStorage, idx);
    }

    inline bool isDeactivated(int idx) const { return FXConfig::isDeactivated(fxStorage, idx); }

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
};
} // namespace sst::effects

#endif