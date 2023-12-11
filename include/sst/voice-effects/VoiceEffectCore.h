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

#include <type_traits>

namespace sst::voice_effects::core
{
// Todo: as we port consider this FXConfig::BaseClass being a bit more configurable.
template <typename VFXConfig> struct VoiceEffectTemplateBase : public VFXConfig::BaseClass
{
    static_assert(std::is_integral<decltype(VFXConfig::blockSize)>::value);
    static_assert(!(VFXConfig::blockSize & (VFXConfig::blockSize - 1))); // 2^n
    static_assert(VFXConfig::blockSize >= 4);                            // > simd register length
    static_assert(std::is_class<typename VFXConfig::BaseClass>::value);

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
    static_assert(std::is_same<decltype(VFXConfig::setFloatParam(
                                   std::declval<typename VFXConfig::BaseClass *>(),
                                   std::declval<size_t>(), std::declval<float>())),
                               void>::value,
                  "Implement setFloatParam");

    static_assert(
        std::is_same<decltype(VFXConfig::getFloatParam(
                         std::declval<typename VFXConfig::BaseClass *>(), std::declval<size_t>())),
                     float>::value,
        "Implement getFloatParam");

    void setFloatParam(int i, float f) { VFXConfig::setFloatParam(asBase(), i, f); }
    float getFloatParam(int i) const { return VFXConfig::getFloatParam(asBase(), i); }

    static_assert(std::is_same<decltype(VFXConfig::setIntParam(
                                   std::declval<typename VFXConfig::BaseClass *>(),
                                   std::declval<size_t>(), std::declval<int>())),
                               void>::value,
                  "Implement setIntParam");

    static_assert(
        std::is_same<decltype(VFXConfig::getIntParam(
                         std::declval<typename VFXConfig::BaseClass *>(), std::declval<size_t>())),
                     int>::value,
        "Implement getIntParam");

    void setIntParam(int i, int f) { VFXConfig::setIntParam(asBase(), i, f); }
    int getIntParam(int i) const { return VFXConfig::getIntParam(asBase(), i); }

    /*
     * Conversions
     */
    static_assert(
        std::is_same<decltype(VFXConfig::dbToLinear(std::declval<typename VFXConfig::BaseClass *>(),
                                                    std::declval<float>())),
                     float>::value,
        "Implement dbToLinear");
    float dbToLinear(float f) { return VFXConfig::dbToLinear(asBase(), f); }

    static_assert(
        std::is_same<decltype(VFXConfig::equalNoteToPitch(
                         std::declval<typename VFXConfig::BaseClass *>(), std::declval<float>())),
                     float>::value,
        "Implement getEqualNoteToPitch");
    float equalNoteToPitch(float f) { return VFXConfig::equalNoteToPitch(asBase(), f); }
    float note_to_pitch_ignoring_tuning(float f) { return equalNoteToPitch(f); }

    static_assert(std::is_same<decltype(VFXConfig::getSampleRate(
                                   std::declval<typename VFXConfig::BaseClass *>())),
                               float>::value,
                  "Implement getSampleRate");
    float getSampleRate() { return VFXConfig::getSampleRate(asBase()); }
    static_assert(std::is_same<decltype(VFXConfig::getSampleRateInv(
                                   std::declval<typename VFXConfig::BaseClass *>())),
                               float>::value,
                  "Implement getSampleRateInv");
    float getSampleRateInv() { return VFXConfig::getSampleRateInv(asBase()); }

    /*
     * We have to provide a memory pool
     */
    static_assert(
        std::is_same<decltype(VFXConfig::preReservePool(
                         std::declval<typename VFXConfig::BaseClass *>(), std::declval<size_t>())),
                     void>::value,
        "void preReservePool(base*, size_t) not defined");

    static_assert(
        std::is_same<decltype(VFXConfig::checkoutBlock(
                         std::declval<typename VFXConfig::BaseClass *>(), std::declval<size_t>())),
                     uint8_t *>::value,
        "uint8_t* checkoutBlock(base *,size_t) not defined");

    static_assert(std::is_same<decltype(VFXConfig::returnBlock(
                                   std::declval<typename VFXConfig::BaseClass *>(),
                                   std::declval<uint8_t *>(), std::declval<size_t>())),
                               void>::value,
                  "void returnBlock(base *, uint8_t *, size_t) not defined");

    void preReservePool(size_t s) { VFXConfig::preReservePool(asBase(), s); }
    uint8_t *checkoutBlock(size_t s) { return VFXConfig::checkoutBlock(asBase(), s); }
    void returnBlock(uint8_t *d, size_t s) { VFXConfig::returnBlock(asBase(), d, s); }
};
} // namespace sst::voice_effects::core

#endif