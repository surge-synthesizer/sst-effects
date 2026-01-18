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
#include "sst/basic-blocks/dsp/RNG.h"

// delay
#include "sst/voice-effects/delay/Microgate.h"
#include "sst/voice-effects/delay/ShortDelay.h"
#include "sst/voice-effects/delay/Widener.h"

// distortion
#include "sst/voice-effects/distortion/Slewer.h"
#include "sst/voice-effects/distortion/TreeMonster.h"
#include "sst/voice-effects/distortion/BitCrusher.h"

// dynamics
#include "sst/voice-effects/dynamics/Compressor.h"
#include "sst/voice-effects/dynamics/AutoWah.h"

// eq
#include "sst/voice-effects/eq/EqGraphic6Band.h"
#include "sst/voice-effects/eq/EqNBandParametric.h"
#include "sst/voice-effects/eq/MorphEQ.h"
#include "sst/voice-effects/eq/TiltEQ.h"

// filter
#include "sst/voice-effects/filter/FiltersPlusPlus.h"
#include "sst/voice-effects/filter/StaticPhaser.h"
#include "sst/voice-effects/filter/UtilityFilters.h"

// generator
#include "sst/voice-effects/generator/3opPhaseMod.h"
#include "sst/voice-effects/generator/EllipticBlepWaveforms.h"
#include "sst/voice-effects/generator/FourVoiceResonator.h"
#include "sst/voice-effects/generator/GenCorrelatedNoise.h"
#include "sst/voice-effects/generator/SinePlus.h"
#include "sst/voice-effects/generator/StringResonator.h"
#include "sst/voice-effects/generator/TiltNoise.h"

// lifted_bus_effects
#include "sst/voice-effects/lifted_bus_effects/LiftedDelay.h"
#include "sst/voice-effects/lifted_bus_effects/LiftedReverb1.h"
#include "sst/voice-effects/lifted_bus_effects/LiftedReverb2.h"

// modulation
#include "sst/voice-effects/modulation/Chorus.h"
#include "sst/voice-effects/modulation/Flanger.h"
#include "sst/voice-effects/modulation/FMFilter.h"
#include "sst/voice-effects/modulation/FreqShiftMod.h"
#include "sst/voice-effects/modulation/NoiseAM.h"
#include "sst/voice-effects/modulation/PhaseMod.h"
#include "sst/voice-effects/modulation/Phaser.h"
#include "sst/voice-effects/modulation/RingMod.h"
#include "sst/voice-effects/modulation/ShepardPhaser.h"
#include "sst/voice-effects/modulation/Tremolo.h"

// utilities
#include "sst/voice-effects/utilities/StereoTool.h"
#include "sst/voice-effects/utilities/VolumeAndPan.h"
#include "sst/voice-effects/utilities/GainMatrix.h"

// waveshaper
#include "sst/voice-effects/waveshaper/WaveShaper.h"

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
    static void preReserveSingleInstancePool(BaseClass *, size_t) {}
    static uint8_t *checkoutBlock(BaseClass *, size_t s) { return (uint8_t *)malloc(s); }
    static void returnBlock(BaseClass *, uint8_t *p, size_t) { free(p); }
};
template <typename T> struct VTester
{
    template <class... Args> static void TestVFX(Args &&...a)
    {
        INFO("Starting test with instantiation : " << T::displayName);
        auto fx = std::make_unique<T>(std::forward<Args>(a)...);
        REQUIRE(fx);
        fx->initVoiceEffectParams();
    };
};

TEST_CASE("Can Create Voice FX")
{
    // delay
    SECTION("MicroGate")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::MicroGate<VTestConfig>>::TestVFX(s);
    }
    SECTION("ShortDelay")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::ShortDelay<VTestConfig>>::TestVFX(s);
    }
    SECTION("Widener")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::delay::Widener<VTestConfig>>::TestVFX(s);
    }

    // distortion
    SECTION("Slewer") { VTester<sst::voice_effects::distortion::Slewer<VTestConfig>>::TestVFX(); }
    SECTION("BitCrusher")
    {
        VTester<sst::voice_effects::distortion::BitCrusher<VTestConfig>>::TestVFX();
    }
    SECTION("TreeMonster")
    {
        VTester<sst::voice_effects::distortion::TreeMonster<VTestConfig>>::TestVFX();
    }

    // dynamics
    SECTION("Compressor")
    {
        VTester<sst::voice_effects::dynamics::Compressor<VTestConfig>>::TestVFX();
    }
    SECTION("AutoWah") { VTester<sst::voice_effects::dynamics::AutoWah<VTestConfig>>::TestVFX(); }

    // eq
    SECTION("GraphicEQ")
    {
        VTester<sst::voice_effects::eq::EqGraphic6Band<VTestConfig>>::TestVFX();
    }
    SECTION("ParmEQ")
    {
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 1>>::TestVFX();
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 2>>::TestVFX();
        VTester<sst::voice_effects::eq::EqNBandParametric<VTestConfig, 3>>::TestVFX();
    }
    SECTION("MorphEQ") { VTester<sst::voice_effects::eq::MorphEQ<VTestConfig>>::TestVFX(); }
    SECTION("TiltEQ") { VTester<sst::voice_effects::eq::TiltEQ<VTestConfig>>::TestVFX(); }

    // filter
    SECTION("FiltersPlusPlus")
    {
        using fmd = sst::filtersplusplus::FilterModel;
        VTester<
            sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::CytomicSVF>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig,
                                                            fmd::VemberClassic>>::TestVFX();
        VTester<
            sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::DiodeLadder>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::TriPole>>::TestVFX();
        VTester<
            sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::OBXD_4Pole>>::TestVFX();
        VTester<
            sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::OBXD_Xpander>>::TestVFX();
        VTester<
            sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::CutoffWarp>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig,
                                                            fmd::ResonanceWarp>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig,
                                                            fmd::SampleAndHold>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::K35>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig, fmd::Comb>>::TestVFX();
        VTester<sst::voice_effects::filter::FiltersPlusPlus<VTestConfig,
                                                            fmd::VintageLadder>>::TestVFX();
    }
    SECTION("Static Phaser")
    {
        VTester<sst::voice_effects::filter::StaticPhaser<VTestConfig>>::TestVFX();
    }
    SECTION("Utility Filters")
    {
        VTester<sst::voice_effects::filter::UtilityFilters<VTestConfig>>::TestVFX();
    }

    // generator
    SECTION("3-op PM")
    {
        sst::basic_blocks::tables::TwoToTheXProvider t;
        sst::basic_blocks::tables::SixSinesWaveProvider s(false);
        VTester<sst::voice_effects::generator::ThreeOpPhaseMod<VTestConfig>>::TestVFX(t, s);
    }
    SECTION("Elliptic Blep Waveforms")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::generator::EllipticBlepWaveforms<VTestConfig>>::TestVFX(rng);
    }
    SECTION("FourVoiceRes")
    {
        sst::basic_blocks::tables::SimpleSineProvider s;
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::generator::FourVoiceResonator<VTestConfig>>::TestVFX(s, rng);
    }
    SECTION("GenCorrelatedNoise")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::generator::GenCorrelatedNoise<VTestConfig>>::TestVFX(rng);
    }
    SECTION("Sine Plus")
    {
        VTester<sst::voice_effects::generator::SinePlus<VTestConfig>>::TestVFX();
    }
    SECTION("StringResonator")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        VTester<sst::voice_effects::generator::StringResonator<VTestConfig>>::TestVFX(s);
    }
    SECTION("Tilt Noise")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::generator::TiltNoise<VTestConfig>>::TestVFX(rng);
    }

    // lifted_bus_effects
    SECTION("Lifted Reverb 1")
    {
        VTester<sst::voice_effects::liftbus::LiftedReverb1<VTestConfig>>::TestVFX();
    }
    SECTION("Lifted Reverb 2")
    {
        VTester<sst::voice_effects::liftbus::LiftedReverb2<VTestConfig>>::TestVFX();
    }
    SECTION("Lifted Delay")
    {
        VTester<sst::voice_effects::liftbus::LiftedDelay<VTestConfig>>::TestVFX();
    }

    // modulation
    SECTION("Chorus")
    {
        sst::basic_blocks::tables::SurgeSincTableProvider s;
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::Chorus<VTestConfig>>::TestVFX(s, rng);
    }
    SECTION("Voice Flanger")
    {
        sst::basic_blocks::tables::SimpleSineProvider s;
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::VoiceFlanger<VTestConfig>>::TestVFX(s, rng);
    }
    SECTION("FM Filter")
    {
        VTester<sst::voice_effects::modulation::FMFilter<VTestConfig>>::TestVFX();
    }
    SECTION("FreqShiftMod")
    {
        VTester<sst::voice_effects::modulation::FreqShiftMod<VTestConfig>>::TestVFX();
    }
    SECTION("Noise AM")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::NoiseAM<VTestConfig>>::TestVFX(rng);
    }
    SECTION("Phase Mod")
    {
        VTester<sst::voice_effects::modulation::PhaseMod<VTestConfig>>::TestVFX();
    }
    SECTION("Phaser")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::Phaser<VTestConfig>>::TestVFX(rng);
    }
    SECTION("RingMod") { VTester<sst::voice_effects::modulation::RingMod<VTestConfig>>::TestVFX(); }
    SECTION("Shepard Phaser")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::ShepardPhaser<VTestConfig>>::TestVFX(rng);
    }
    SECTION("Tremolo")
    {
        sst::basic_blocks::dsp::RNG rng;
        VTester<sst::voice_effects::modulation::Tremolo<VTestConfig>>::TestVFX(rng);
    }

    // utilities
    SECTION("VolumeAndPan")
    {
        VTester<sst::voice_effects::utilities::VolumeAndPan<VTestConfig>>::TestVFX();
    }
    SECTION("StereoTool")
    {
        VTester<sst::voice_effects::utilities::StereoTool<VTestConfig>>::TestVFX();
    }
    SECTION("GainMatrix")
    {
        VTester<sst::voice_effects::utilities::GainMatrix<VTestConfig>>::TestVFX();
    }

    // waveshaper
    SECTION("WaveShaper")
    {
        VTester<sst::voice_effects::waveshaper::WaveShaper<VTestConfig>>::TestVFX();
    }
}
