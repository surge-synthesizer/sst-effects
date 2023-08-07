//
// Created by Paul Walker on 8/7/23.
//

#include "catch2.hpp"
#include "simd-test-include.h"

#include "sst/voice-effects/distortion/BitCrusher.h"
#include "sst/voice-effects/distortion/Microgate.h"

struct VTestConfig
{
    struct BaseClass
    {
    };
    static constexpr int blockSize{16};
    static void setFloatParam(BaseClass *, int, float) {}
    static float getFloatParam(BaseClass *, int) { return 0.f; }
    static float dbToLinear(BaseClass *, float f) { return 1.f; }
    static float equalNoteToPitch(BaseClass *, float f) { return 0.f; }
    static float getSampleRate(BaseClass *) { return 48000.f; }
    static float getSampleRateInv(BaseClass *) { return 1.0 / 48000.f; }

    static void preReservePool(BaseClass *, size_t) {}
    static uint8_t *checkoutBlock(BaseClass *, size_t) { return nullptr; }
    static void returnBlock(BaseClass *, uint8_t *, size_t) {}
};
template <typename T> struct VTester
{
    static void TestVFX()
    {
        INFO("Starting test with instantiation : " << T::effectName);
        auto fx = std::make_unique<T>();
        REQUIRE(fx);
    };
};

TEST_CASE("Can Create Voice FX")
{
    SECTION("MicroGate")
    {
        VTester<sst::voice_effects::distortion::MicroGate<VTestConfig>>::TestVFX();
    }
    SECTION("BitCrusher")
    {
        VTester<sst::voice_effects::distortion::BitCrusher<VTestConfig>>::TestVFX();
    }
}