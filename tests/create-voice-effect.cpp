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
#include "sst/voice-effects/delay/Microgate.h"
#include "sst/voice-effects/distortion/Slewer.h"
#include "sst/voice-effects/distortion/TreeMonster.h"
#include "sst/voice-effects/modulation/RingMod.h"
#include "sst/voice-effects/waveshaper/WaveShaper.h"
#include "sst/voice-effects/modulation/FreqShiftMod.h"
#include "sst/voice-effects/generator/GenVA.h"
#include "sst/voice-effects/modulation/PhaseMod.h"
#include "sst/voice-effects/generator/GenCorrelatedNoise.h"
#include "sst/voice-effects/eq/EqNBandParametric.h"
#include "sst/voice-effects/eq/MorphEQ.h"
#include "sst/voice-effects/eq/EqGraphic6Band.h"
#include "sst/voice-effects/delay/Widener.h"
#include "sst/voice-effects/delay/ShortDelay.h"
#include "sst/voice-effects/delay/StringResonator.h"
#include "sst/voice-effects/filter/CytomicSVF.h"
#include "sst/voice-effects/filter/SurgeBiquads.h"
#include "sst/voice-effects/filter/SSTFilters.h"
#include "sst/voice-effects/filter/StaticPhaser.h"
#include "sst/voice-effects/modulation/ShepardPhaser.h"
#include "sst/voice-effects/modulation/Tremolo.h"
#include "sst/voice-effects/modulation/Phaser.h"
#include "sst/voice-effects/modulation/FMFilter.h"
#include "sst/voice-effects/generator/TiltNoise.h"
#include "sst/voice-effects/modulation/NoiseAM.h"

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
    SECTION("MicroGate") { VTester<sst::voice_effects::delay::MicroGate<VTestConfig>>::TestVFX(); }
    SECTION("BitCrusher")
    {
        VTester<sst::voice_effects::distortion::BitCrusher<VTestConfig>>::TestVFX();
    }

    SECTION("Slewer") { VTester<sst::voice_effects::distortion::Slewer<VTestConfig>>::TestVFX(); }
    SECTION("TreeMonster")
    {
        VTester<sst::voice_effects::distortion::TreeMonster<VTestConfig>>::TestVFX();
    }
    SECTION("RingMod") { VTester<sst::voice_effects::modulation::RingMod<VTestConfig>>::TestVFX(); }
    SECTION("WaveShaper")
    {
        VTester<sst::voice_effects::waveshaper::WaveShaper<VTestConfig>>::TestVFX();
    }
    SECTION("PitchRing")
    {
        VTester<sst::voice_effects::modulation::FreqShiftMod<VTestConfig>>::TestVFX();
    }
    SECTION("GenPhaseMod")
    {
        VTester<sst::voice_effects::modulation::PhaseMod<VTestConfig>>::TestVFX();
    }
    SECTION("GenCorrelatedNoise")
    {
        VTester<sst::voice_effects::generator::GenCorrelatedNoise<VTestConfig>>::TestVFX();
    }
    SECTION("GenVA")
    {
        sst::basic_blocks::tables::ShortcircuitSincTableProvider s;
        VTester<sst::voice_effects::generator::GenVA<VTestConfig>>::TestVFX(s);
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
        VTester<sst::voice_effects::delay::Widener<VTestConfig>>::TestVFX(s);
    }

    SECTION("ShortDelay")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::ShortDelay<VTestConfig>>::TestVFX(s);
    }

    SECTION("StringExciter")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::StringResonator<VTestConfig>>::TestVFX(s);
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
    SECTION("Static Phaser")
    {
        VTester<sst::voice_effects::filter::StaticPhaser<VTestConfig>>::TestVFX();
    }
    SECTION("Shepard Phaser")
    {
        VTester<sst::voice_effects::modulation::ShepardPhaser<VTestConfig>>::TestVFX();
    }
    SECTION("Tremolo") { VTester<sst::voice_effects::modulation::Tremolo<VTestConfig>>::TestVFX(); }
    SECTION("Phaser") { VTester<sst::voice_effects::modulation::Phaser<VTestConfig>>::TestVFX(); }
    SECTION("FM Filter")
    {
        VTester<sst::voice_effects::modulation::FMFilter<VTestConfig>>::TestVFX();
    }
    SECTION("Tilt Noise")
    {
        VTester<sst::voice_effects::generator::TiltNoise<VTestConfig>>::TestVFX();
    }
    SECTION("Phaser") { VTester<sst::voice_effects::modulation::NoiseAM<VTestConfig>>::TestVFX(); }
}
