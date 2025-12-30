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

#ifndef INCLUDE_SST_VOICE_EFFECTS_VOICEEFFECTCORE_H
#define INCLUDE_SST_VOICE_EFFECTS_VOICEEFFECTCORE_H

/*
 * Document This
 *
 */

// For shared width calculations
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/filters/BiquadFilter.h"
#include "sst/filters/CytomicSVF.h"

#include <type_traits>
#include <concepts>
#include <cstdint>

static_assert(__cplusplus >= 202002L, "Surge team libraries have moved to C++ 20");

namespace sst::voice_effects::core
{

enum struct StreamingFlags : uint32_t
{
    NONE = 0,
    IS_TEMPOSYNCED = 1 << 0,
    IS_DEACTIVATED = 1 << 1,
    IS_KEYTRACKED = 1 << 2
};

// Concept to validate a Voice Effect configuration type for VoiceEffectTemplateBase
template <typename C>
concept VoiceEffectConfig = requires {
    // BaseClass must be a type and class
    typename C::BaseClass;
    requires std::is_class_v<typename C::BaseClass>;

    // blockSize must be an integral constant, power of two and >= 4
    { C::blockSize } -> std::convertible_to<const int>;
    requires std::integral<std::remove_cvref_t<decltype(C::blockSize)>>;
    requires(C::blockSize & (C::blockSize - 1)) == 0;
    requires(C::blockSize >= 4);
} && requires(typename C::BaseClass *b, size_t idx, float f, int i, uint8_t *p) {
    // Parameter accessors
    { C::setFloatParam(b, idx, f) } -> std::same_as<void>;
    { C::getFloatParam(b, idx) } -> std::same_as<float>;
    { C::setIntParam(b, idx, i) } -> std::same_as<void>;
    { C::getIntParam(b, idx) } -> std::same_as<int>;

    // Conversions
    { C::dbToLinear(b, f) } -> std::same_as<float>;
    { C::equalNoteToPitch(b, f) } -> std::same_as<float>;

    // Sample rate helpers
    { C::getSampleRate(b) } -> std::same_as<float>;
    { C::getSampleRateInv(b) } -> std::same_as<float>;

    // Memory pool interface
    { C::preReservePool(b, idx) } -> std::same_as<void>;
    { C::checkoutBlock(b, idx) } -> std::same_as<uint8_t *>;
    { C::returnBlock(b, p, idx) } -> std::same_as<void>;
};

// Todo: as we port consider this FXConfig::BaseClass being a bit more configurable.
template <VoiceEffectConfig VFXConfig> struct VoiceEffectTemplateBase : public VFXConfig::BaseClass
{
    using config_t = VFXConfig;
    // All core constraints are now enforced by the VoiceEffectConfig concept

    typename VFXConfig::BaseClass *asBase()
    {
        return static_cast<typename VFXConfig::BaseClass *>(this);
    }

    const typename VFXConfig::BaseClass *asBase() const
    {
        return static_cast<const typename VFXConfig::BaseClass *>(this);
    }

    /*
     * Params
     */
    void setFloatParam(int i, float f) { VFXConfig::setFloatParam(asBase(), i, f); }
    float getFloatParam(int i) const { return VFXConfig::getFloatParam(asBase(), i); }

    void setIntParam(int i, int f) { VFXConfig::setIntParam(asBase(), i, f); }
    int getIntParam(int i) const { return VFXConfig::getIntParam(asBase(), i); }

    /*
     * Conversions
     */
    float dbToLinear(float f) { return VFXConfig::dbToLinear(asBase(), f); }
    static float dbToLinear(typename VFXConfig::BaseClass *b, float f)
    {
        return VFXConfig::dbToLinear(b, f);
    }

    float equalNoteToPitch(float f) { return VFXConfig::equalNoteToPitch(asBase(), f); }
    float note_to_pitch_ignoring_tuning(float f) { return equalNoteToPitch(f); }
    static float noteToPitchIgnoringTuning(VoiceEffectTemplateBase<VFXConfig> *that, float f)
    {
        return that->note_to_pitch_ignoring_tuning(f);
    }
    static float sampleRateInv(VoiceEffectTemplateBase<VFXConfig> *that)
    {
        return that->getSampleRateInv();
    }

    float envelope_rate_linear_nowrap(float f)
    {
        auto res = VFXConfig::blockSize * VFXConfig::getSampleRateInv(this) * std::pow(2, -f);
        if (getIsTemposync())
        {
            updateTempo();
            res = res * temposyncratio;
        }
        return res;
    }

    float getSampleRate() const { return VFXConfig::getSampleRate(asBase()); }
    float getSampleRateInv() const { return VFXConfig::getSampleRateInv(asBase()); }

    /*
     * We have to provide a memory pool
     */
    void preReservePool(size_t s) { VFXConfig::preReservePool(asBase(), s); }

    uint8_t *checkoutBlock(size_t s) { return VFXConfig::checkoutBlock(asBase(), s); }
    void returnBlock(uint8_t *d, size_t s) { VFXConfig::returnBlock(asBase(), d, s); }

    template <typename T> void initToParamMetadataDefault(T *that)
    {
        for (int i = 0; i < T::numFloatParams; ++i)
        {
            that->setFloatParam(i, that->paramAt(i).defaultVal);
        }
        if constexpr (T::numIntParams > 0)
        {
            for (int i = 0; i < T::numIntParams; ++i)
            {
                that->setIntParam(i, that->intParamAt(i).defaultVal);
            }
        }
    }

    static constexpr bool useLinearWidth()
    {
        return true; // all voice effects use new style width
    }

#define HASMEM(M, GetSig, DVAL, PARENS)                                                            \
    template <typename T, typename = void> struct has_##M : std::false_type                        \
    {                                                                                              \
    };                                                                                             \
    template <typename T> struct has_##M<T, std::void_t<decltype(&T::M)>> : std::true_type         \
    {                                                                                              \
    };                                                                                             \
    GetSig                                                                                         \
    {                                                                                              \
        if constexpr (has_##M<VFXConfig>::value)                                                   \
        {                                                                                          \
            return VFXConfig::M PARENS;                                                            \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            DVAL;                                                                                  \
        }                                                                                          \
    }

    HASMEM(oversamplingRatio, constexpr int16_t getOversamplingRatio(), return 1, );
    HASMEM(getTempoPointer, double *getBaseTempoPointer(), return &defaultTempo, (asBase()));
    HASMEM(isTemposync, bool getIsTemposync(), return false, (asBase()));
    HASMEM(isDeactivated, bool getIsDeactivated(int index), return false, (asBase(), index));
    HASMEM(preReserveSingleInstancePool, void preReserveSingleInstancePool(size_t s),
           throw std::logic_error("this effect requires single instance pools"), (asBase(), s));

#undef HASMEM

    using BiquadFilterType =
        sst::filters::Biquad::BiquadFilter<VoiceEffectTemplateBase<VFXConfig>, VFXConfig::blockSize,
                                           VoiceEffectTemplateBase<VFXConfig>>;

    void updateTempo()
    {
        auto tempoPointer = getBaseTempoPointer();
        assert(tempoPointer);
        if (tempo != *tempoPointer)
        {
            tempo = *tempoPointer;
            temposyncratio = tempo / 120.0;
            temposyncratioinv = 1.0 / temposyncratio;
        }
    }

    float getTempoSyncRatio() { return temposyncratio; }

  private:
    double defaultTempo{120.0};

  protected:
    double tempo{defaultTempo}, temposyncratio{defaultTempo / 120.0},
        temposyncratioinv{120.0 / defaultTempo};
};
} // namespace sst::voice_effects::core

#include "VoiceEffectsPresetSupport.h"
#endif
