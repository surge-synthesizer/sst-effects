//
// Created by Paul Walker on 4/23/23.
//


#include "catch2.hpp"
#include "simd-test-include.h"

#include "sst/effects/ConcreteConfig.h"
#include "sst/effects/Flanger.h"
#include "sst/effects/Reverb1.h"


namespace sfx = sst::effects;


template<typename T>
struct Tester
{
    static void TestFX() {
        using FX = T;

        INFO( "Starting test with instantiation" );

        auto gs = sst::effects::ConcreteConfig::GlobalStorage(48000);
        auto es = sfx::ConcreteConfig::EffectStorage();

        auto fx = FX(&gs, &es, nullptr);

        // By using the concrete configuration you get a collection of params you can address
        for (int i=0; i< FX::numParams; ++i)
            fx.paramStorage[i] = fx.paramAt(i).defaultVal;

        fx.initialize();

        float L alignas(16) [sfx::ConcreteConfig::blockSize],
            R alignas(16) [sfx::ConcreteConfig::blockSize];

        float phase = 0.f;
        float dphase = 1.0 / 317.4;
        for (int blocks=0; blocks < 1000; ++blocks)
        {
            for (int s=0; s<sfx::ConcreteConfig::blockSize; ++s)
            {
                L[s] = phase * 2 - 1;
                R[s] = phase > 0.7 ? 1 : -1;
                phase += dphase;
                if (phase > 1)
                    phase -= 1;
            }

            fx.processBlock(L, R);

            for (int s=0; s<sfx::ConcreteConfig::blockSize; ++s)
            {
                REQUIRE(fabs(L[s]) < 1.1);
                REQUIRE(fabs(R[s]) < 1.1);
            }
        }

    };
};

TEST_CASE( "Can Run Types with Concrete Config" )
{
    SECTION("Flanger")
    {
        Tester<sfx::Flanger<sfx::ConcreteConfig>>::TestFX();
    }
    SECTION("Reverb1")
    {
        Tester<sfx::Reverb1<sfx::ConcreteConfig>>::TestFX();
    }
}