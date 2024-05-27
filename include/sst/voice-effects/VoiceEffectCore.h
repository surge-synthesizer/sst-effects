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

#include <type_traits>
#include <concepts>

// For shared width calculations
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"

#include "sst/basic-blocks/concepts/concepts.h"

#include "sst/filters/BiquadFilter.h"

#include <type_traits>

namespace sst::voice_effects::core
{

template <typename VFXConfig>
concept baseClassAPI =
    requires(typename VFXConfig::BaseClass *b, size_t st, float f, int i, uint8_t *u8) {
        {
            VFXConfig::setFloatParam(b, st, f)
        } -> std::convertible_to<void>;
        {
            VFXConfig::getFloatParam(b, st)
        } -> std::floating_point;

        {
            VFXConfig::setIntParam(b, st, i)
        } -> std::convertible_to<void>;
        {
            VFXConfig::getIntParam(b, st)
        } -> std::integral;

        {
            VFXConfig::dbToLinear(b, f)
        } -> std::floating_point;

        {
            VFXConfig::equalNoteToPitch(b, f)
        } -> std::floating_point;

        {
            VFXConfig::getSampleRate(b)
        } -> std::floating_point;
        {
            VFXConfig::getSampleRateInv(b)
        } -> std::floating_point;

        {
            VFXConfig::preReservePool(b, st)
        } -> std::convertible_to<void>;
        {
            VFXConfig::checkoutBlock(b, st)
        } -> std::same_as<uint8_t *>;
        {
            VFXConfig::returnBlock(b, u8, st)
        } -> std::convertible_to<void>;
    };

template <typename VFXConfig>
concept optionalOversampling = std::is_integral_v<typename VFXConfig::oversamplingRatio>;

template <typename VFXConfig>
concept optionalEnvelopeTime = requires(typename VFXConfig::BaseClass *b, float f) {
    {
        VFXConfig::envelope_rate_linear_nowrap(b, f)
    } -> std::convertible_to<float>;
};

template <typename VFXConfig>
concept supportsVoiceConfig =
    std::is_class_v<typename VFXConfig::BaseClass> &&
    sst::basic_blocks::concepts::is_power_of_two_ge(VFXConfig::blockSize, 4) &&
    baseClassAPI<VFXConfig>;

// Todo: as we port consider this FXConfig::BaseClass being a bit more configurable.
template <typename VFXConfig>
    requires(supportsVoiceConfig<VFXConfig>)
struct VoiceEffectTemplateBase : public VFXConfig::BaseClass
{
    typename VFXConfig::BaseClass *asBase()
    {
        return static_cast<typename VFXConfig::BaseClass *>(this);
    }

    const typename VFXConfig::BaseClass *asBase() const
    {
        return static_cast<const typename VFXConfig::BaseClass *>(this);
    }

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

    float getSampleRate() { return VFXConfig::getSampleRate(asBase()); }
    float getSampleRateInv() { return VFXConfig::getSampleRateInv(asBase()); }

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

    constexpr int16_t getOversamplingRatio()
    {
        if constexpr (optionalOversampling<VFXConfig>)
        {
            return VFXConfig::oversamplingRatio;
        }
        else
        {
            return 1;
        }
    }

    float envelope_rate_linear_nowrap(float f)
    {
        if constexpr (optionalEnvelopeTime<VFXConfig>)
        {
            return VFXConfig::envelope_rate_linear_nowrap(this, f);
        }
        else
        {
            return VFXConfig::blockSize * VFXConfig::getSampleRateInv(this) * std::pow(2, -f);
        }
    }

    using BiquadFilterType = sst::filters::Biquad::BiquadFilter<VoiceEffectTemplateBase<VFXConfig>,
                                                                VFXConfig::blockSize>;
};
} // namespace sst::voice_effects::core

#endif