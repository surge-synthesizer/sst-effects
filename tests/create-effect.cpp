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
#include "simd-test-include.h"

#include "sst/effects/Delay.h"
#include "sst/effects/FloatyDelay.h"
#include "sst/effects/Flanger.h"
#include "sst/effects/Reverb1.h"
#include "sst/effects/Bonsai.h"
#include "sst/effects/Phaser.h"
#include "sst/effects/Reverb2.h"
#include "sst/effects/TreeMonster.h"
#include "sst/effects/Nimbus.h"
#include "sst/effects/NimbusImpl.h"

struct TestConfig
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
        return 0;
    }
    static inline int intValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return 0;
    }

    static inline float envelopeRateLinear(GlobalStorage *s, float f) { return 0; }

    static inline float temposyncRatio(GlobalStorage *s, EffectStorage *e, int idx) { return 1; }

    static inline bool isDeactivated(EffectStorage *e, int idx) { return false; }

    static inline bool isExtended(EffectStorage *s, int idx) { return false; }

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

template <typename T> struct Tester
{
    static_assert(std::is_same<decltype(T::effectName), const char *const>::value);
    static_assert(std::is_integral<decltype(T::numParams)>::value);
    static_assert(std::is_same<decltype(&T::initialize), void (T::*)()>::value);
    static_assert(std::is_same<decltype(&T::processBlock),
                               void (T::*)(float *__restrict, float *__restrict)>::value);
    static_assert(std::is_same<decltype(&T::suspendProcessing), void (T::*)()>::value);
    static_assert(std::is_same<decltype(&T::getRingoutDecay), int (T::*)() const>::value);
    static_assert(std::is_same<decltype(&T::paramAt),
                               sst::basic_blocks::params::ParamMetaData (T::*)(int) const>::value);
    static_assert(std::is_same<decltype(&T::onSampleRateChanged), void (T::*)()>::value);

    static void TestFX()
    {
        INFO("Starting test with instantiation : " << T::effectName);
        auto fx = std::make_unique<T>(nullptr, nullptr, nullptr);
        REQUIRE(fx);
    };
};

TEST_CASE("Can Create Types")
{
    SECTION("Flanger") { Tester<sst::effects::flanger::Flanger<TestConfig>>::TestFX(); }
    SECTION("Reverb1") { Tester<sst::effects::reverb1::Reverb1<TestConfig>>::TestFX(); }
    SECTION("Delay") { Tester<sst::effects::delay::Delay<TestConfig>>::TestFX(); }
    SECTION("Bonsai") { Tester<sst::effects::bonsai::Bonsai<TestConfig>>::TestFX(); }
    SECTION("Phaser") { Tester<sst::effects::phaser::Phaser<TestConfig>>::TestFX(); }
    SECTION("Reverb2") { Tester<sst::effects::reverb2::Reverb2<TestConfig>>::TestFX(); }
    SECTION("TreeMonster") { Tester<sst::effects::treemonster::TreeMonster<TestConfig>>::TestFX(); }
    SECTION("Nimbus") { Tester<sst::effects::nimbus::Nimbus<TestConfig>>::TestFX(); }
    SECTION("Floaty Delay") { Tester<sst::effects::nimbus::FloatyDelay<TestConfig>>::TestFX(); }
}
