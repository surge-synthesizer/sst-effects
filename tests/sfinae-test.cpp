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

#include <memory>
#include <type_traits>
#include "catch2.hpp"

#include "sst/basic-blocks/simd/setup.h"

#include "sst/effects/EffectCore.h"

struct NoExtraConfig
{
    struct BC
    {
        template <typename... Types> BC(Types...) {}
    };

    struct GS
    {
    };
    struct ES
    {
        float values alignas(16)[20]; // 20 is arbitrary
    };

    using BaseClass = BC;
    using GlobalStorage = GS;
    using FilterStorage = GS;
    using EffectStorage = ES;
    using ValueStorage = float *;
    using BiquadAdapter = GS;

    static constexpr int blockSize{16};

    static inline float floatValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return 17.2;
    }
    static inline int intValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return 0;
    }

    static inline float envelopeRateLinear(GlobalStorage *s, float f) { return 0; }

    static inline float temposyncRatio(GlobalStorage *s, EffectStorage *e, int idx) { return 1.5; }

    static inline bool isDeactivated(EffectStorage *e, int idx) { return false; }
    static inline bool isTemposynced(EffectStorage *e, int idx) { return false; }

    static bool isExtended(EffectStorage *e, int idx) { return false; }

    static inline float rand01(GlobalStorage *s) { return (float)rand() / (float)RAND_MAX; }

    static inline double sampleRate(GlobalStorage *s) { return 48000; }

    static inline float noteToPitch(GlobalStorage *s, float p) { return 1; }
    static inline float noteToPitchIgnoringTuning(GlobalStorage *s, float p)
    {
        return noteToPitch(s, p);
    }

    static inline float noteToPitchInv(GlobalStorage *s, float p)
    {
        return 1.0 / noteToPitch(s, p);
    }

    static inline float dbToLinear(GlobalStorage *s, float f) { return 1; }
};

struct ExtraConfig : NoExtraConfig
{
    static inline float floatValueExtendedAt(const BaseClass *const e, const ValueStorage *const v,
                                             int idx)
    {
        return 37.4f;
    }

    static inline bool temposyncInitialized(GlobalStorage *) { return false; }
    static float temposyncRatioInv(GlobalStorage *s, EffectStorage *e, int idx) { return 0.5f; }
    static int deformType(EffectStorage *s, int idx) { return 7; }
};

TEST_CASE("SFINAE gives us extras")
{
    SECTION("On Missing")
    {
        auto ec = std::make_unique<sst::effects::core::EffectTemplateBase<NoExtraConfig>>(
            nullptr, nullptr, nullptr);
        REQUIRE(ec->floatValue(0) == 17.2f);
        REQUIRE(ec->floatValueExtended(0) == 17.2f);
        REQUIRE(ec->temposyncInitialized() == true);
        REQUIRE(ec->temposyncRatio(0) == 1.5f);
        REQUIRE(ec->temposyncRatioInv(0) == 1.f / 1.5f);
        REQUIRE(ec->deformType(0) == 0);
    }

    SECTION("On Not Missing")
    {
        auto ec = std::make_unique<sst::effects::core::EffectTemplateBase<ExtraConfig>>(
            nullptr, nullptr, nullptr);
        REQUIRE(ec->floatValue(0) == 17.2f);
        REQUIRE(ec->floatValueExtended(0) == 37.4f);
        REQUIRE(ec->temposyncInitialized() == false);
        REQUIRE(ec->temposyncRatioInv(0) == 0.5f);
        REQUIRE(ec->deformType(0) == 7);
    }
}