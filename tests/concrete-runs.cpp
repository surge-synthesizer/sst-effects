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

#include "catch2.hpp"
#include "sst/basic-blocks/simd/setup.h"

#include "sst/effects/ConcreteConfig.h"

#include "sst/effects/Delay.h"
#include "sst/effects/Flanger.h"
#include "sst/effects/Reverb1.h"
#include "sst/effects/Bonsai.h"
#include "sst/effects/Phaser.h"
#include "sst/effects/Reverb2.h"
#include "sst/effects/TreeMonster.h"
#include "sst/effects/Nimbus.h"
#include "sst/effects/NimbusImpl.h"
#include "sst/effects/RotarySpeaker.h"

namespace sfx = sst::effects;

template <typename T> struct Tester
{
    static_assert(std::is_same<decltype(T::streamingName), const char *const>::value);
    static_assert(std::is_integral<decltype(T::numParams)>::value);
    static_assert(std::is_same<decltype(&T::initialize), void (T::*)()>::value);
    static_assert(std::is_same<decltype(&T::processBlock),
                               void (T::*)(float *__restrict, float *__restrict)>::value);
    static_assert(std::is_same<decltype(&T::suspendProcessing), void (T::*)()>::value);
    static_assert(std::is_same<decltype(&T::getRingoutDecay), int (T::*)() const>::value);
    static_assert(std::is_same<decltype(&T::paramAt),
                               sst::basic_blocks::params::ParamMetaData (T::*)(int) const>::value);

    static void TestFX()
    {
        using FX = T;

        INFO("Starting test with concrete implementation " << T::streamingName);

        auto gs = sst::effects::core::ConcreteConfig::GlobalStorage(48000);
        auto es = sfx::core::ConcreteConfig::EffectStorage();

        auto fx = std::make_unique<FX>(&gs, &es, nullptr);

        // By using the concrete configuration you get a collection of params you can address
        for (int i = 0; i < FX::numParams; ++i)
            fx->paramStorage[i] = fx->paramAt(i).defaultVal;

        fx->initialize();

        float L alignas(16)[sfx::core::ConcreteConfig::blockSize],
            R alignas(16)[sfx::core::ConcreteConfig::blockSize];

        float phase = 0.f;
        float dphase = 1.0 / 317.4;
        for (int blocks = 0; blocks < 1000; ++blocks)
        {
            for (int s = 0; s < sfx::core::ConcreteConfig::blockSize; ++s)
            {
                L[s] = 0.6 * (phase * 2 - 1);
                R[s] = 0.57 * (phase > 0.7 ? 1 : -1);
                phase += dphase;
                if (phase > 1)
                    phase -= 1;
            }

            fx->processBlock(L, R);

            for (int s = 0; s < sfx::core::ConcreteConfig::blockSize; ++s)
            {
                REQUIRE(fabs(L[s]) < 2.0);
                REQUIRE(fabs(R[s]) < 2.0);
            }
        }
    };
};

TEST_CASE("Can Run Types with Concrete Config")
{
    SECTION("Flanger") { Tester<sfx::flanger::Flanger<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("Reverb1") { Tester<sfx::reverb1::Reverb1<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("Reverb2") { Tester<sfx::reverb2::Reverb2<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("Delay") { Tester<sfx::delay::Delay<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("Bonsai") { Tester<sfx::bonsai::Bonsai<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("Phaser") { Tester<sfx::phaser::Phaser<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("TreeMonster")
    {
        Tester<sst::effects::treemonster::TreeMonster<sfx::core::ConcreteConfig>>::TestFX();
    }
    SECTION("Nimbus") { Tester<sst::effects::nimbus::Nimbus<sfx::core::ConcreteConfig>>::TestFX(); }
    SECTION("RotarySpeaker")
    {
        Tester<sst::effects::rotaryspeaker::RotarySpeaker<sfx::core::ConcreteConfig>>::TestFX();
    }
}