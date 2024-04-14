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

#include "catch2.hpp"
#include "simd-test-include.h"

#include "sst/voice-effects/distortion/BitCrusher.h"
#include "sst/voice-effects/distortion/Microgate.h"
#include "sst/voice-effects/distortion/Slewer.h"
#include "sst/voice-effects/waveshaper/WaveShaper.h"
#include "sst/voice-effects/pitch/PitchRing.h"
#include "sst/voice-effects/generator/GenPulseSync.h"
#include "sst/voice-effects/generator/GenSin.h"
#include "sst/voice-effects/generator/GenSaw.h"
#include "sst/voice-effects/generator/GenPhaseMod.h"
#include "sst/voice-effects/generator/GenCorrelatedNoise.h"
#include "sst/voice-effects/eq/EqNBandParametric.h"
#include "sst/voice-effects/eq/MorphEQ.h"
#include "sst/voice-effects/eq/EqGraphic6Band.h"
#include "sst/voice-effects/delay/FauxStereo.h"
#include "sst/voice-effects/filter/CytomicSVF.h"
#include "sst/voice-effects/filter/SurgeBiquads.h"
#include "sst/voice-effects/filter/SSTFilters.h"

struct VTestConfig
{
    struct BaseClass
    {
        std::array<float, 256> fb{};
        std::array<int, 256> ib{};
    };
    static constexpr int blockSize{16};
    static void setFloatParam(BaseClass *b, int i, float f) { b->fb[i] = f; }
    static float getFloatParam(const BaseClass *b, int i) { return b->fb[i]; }

    static void setIntParam(BaseClass *b, int i, int v) { b->ib[i] = v; }
    static int getIntParam(const BaseClass *b, int i) { return b->ib[i]; }

    static float dbToLinear(const BaseClass *, float f) { return 1.f; }
    static float equalNoteToPitch(const BaseClass *, float f) { return pow(2.f, (f + 69) / 12.f); }
    static float getSampleRate(const BaseClass *) { return 48000.f; }
    static float getSampleRateInv(const BaseClass *) { return 1.0 / 48000.f; }

    static void preReservePool(BaseClass *, size_t) {}
    static uint8_t *checkoutBlock(BaseClass *, size_t) { return nullptr; }
    static void returnBlock(BaseClass *, uint8_t *, size_t) {}
};
template <typename T> struct VTester
{
    template <class... Args> static void TestVFX(Args &&...a)
    {
        INFO("Starting test with instantiation : " << T::effectName);
        auto fx = std::make_unique<T>(std::forward<Args>(a)...);
        REQUIRE(fx);
        fx->initVoiceEffectParams();
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

    SECTION("Slewer") { VTester<sst::voice_effects::distortion::Slewer<VTestConfig>>::TestVFX(); }
    SECTION("WaveShaper")
    {
        VTester<sst::voice_effects::waveshaper::WaveShaper<VTestConfig>>::TestVFX();
    }
    SECTION("PitchRing") { VTester<sst::voice_effects::pitch::PitchRing<VTestConfig>>::TestVFX(); }
    SECTION("GenSin") { VTester<sst::voice_effects::generator::GenSin<VTestConfig>>::TestVFX(); }
    SECTION("GenSaw") { VTester<sst::voice_effects::generator::GenSaw<VTestConfig>>::TestVFX(); }
    SECTION("GenPhaseMod")
    {
        VTester<sst::voice_effects::generator::GenPhaseMod<VTestConfig>>::TestVFX();
    }
    SECTION("GenCorrelatedNoise")
    {
        VTester<sst::voice_effects::generator::GenCorrelatedNoise<VTestConfig>>::TestVFX();
    }
    SECTION("GenPulseSync")
    {
        sst::basic_blocks::tables::ShortcircuitSincTableProvider s;
        VTester<sst::voice_effects::generator::GenPulseSync<VTestConfig>>::TestVFX(s);
    }
    SECTION("ParmEQ")
    {
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 1>>::TestVFX();
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 2>>::TestVFX();
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 3>>::TestVFX();
    }
    SECTION("MorphEQ") { VTester<sst::voice_effects::eq::MorphEQ<VTestConfig>>::TestVFX(); }
    SECTION("GraphicEQ")
    {
        VTester<sst::voice_effects::eq::EqGraphic6Band<VTestConfig>>::TestVFX();
    }

    SECTION("FauxStereo")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::FauxStereo<VTestConfig>>::TestVFX(s);
    }

    SECTION("CytomicSVF")
    {
        VTester<sst::voice_effects::filter::CytomicSVF<VTestConfig>>::TestVFX();
    }

    SECTION("SurgeBiquads")
    {
        VTester<sst::voice_effects::filter::SurgeBiquads<VTestConfig>>::TestVFX();
    }

    SECTION("SSTFilters")
    {
        VTester<sst::voice_effects::filter::SSTFilters<VTestConfig>>::TestVFX();
    }
}