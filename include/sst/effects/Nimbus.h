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

#ifndef INCLUDE_SST_EFFECTS_NIMBUS_H
#define INCLUDE_SST_EFFECTS_NIMBUS_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/LanczosResampler.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"

/*
 * Unlike other effects, Nimbus is split into Nimbus and NimbusImpl.h to allow
 * inclusion of this in header files without pulling in the eurorack entire core
 * into your header as opposed to TU space
 *
 * For you, using NimbusImpl may be fine, or you may want to mix and match and
 * do an explicit instantiation or so on. If you are reading this and you don't
 * know what to do, "just include NimbusImpl.h". And if you do know what to do,
 * then do that!
 */

namespace clouds
{
class GranularProcessor;
}

namespace sst::effects::nimbus
{
#if !SST_EFFECTS_EURORACK
template <typename FXConfig> struct Nimbus : core::EffectTemplateBase<FXConfig>
{
    Nimbus(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
           typename FXConfig::ValueStorage *p)
        : core::EffectTemplateBase<FXConfig>(s, e, p)
    {
        std::cerr << "Warning: Using nimbus without eurorack module" << std::endl;
    }

    static constexpr const char *streamingName{"nimbus"};
    static constexpr const char *displayName{"Nimbus - Deactivated"};
    static constexpr int numParams{0};
    void initialize() {}
    void processBlock(float *__restrict, float *__restrict) {}
    void suspendProcessing() {}
    int getRingoutDecay() const { return -1; }
    sst::basic_blocks::params::ParamMetaData paramAt(int i) const { return {}; }
    void onSampleRateChanged() {}

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};
#else
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct Nimbus : core::EffectTemplateBase<FXConfig>
{
    enum nmb_params
    {
        nmb_mode,
        nmb_quality,

        nmb_position,
        nmb_size,
        nmb_pitch,
        nmb_density,
        nmb_texture,
        nmb_spread,

        nmb_freeze,
        nmb_feedback,

        nmb_reverb,
        nmb_mix,

        nmb_num_params,
    };
    static constexpr int numParams{nmb_num_params};
    static constexpr const char *streamingName{"nimbus"};
    static constexpr const char *displayName{"Nimbus"};

    Nimbus(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
           typename FXConfig::ValueStorage *p);
    ~Nimbus();

    void initialize();
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return -1; }
    size_t silentSamplesLength() const { return this->sampleRate() * 5; }
    void onSampleRateChanged() { initialize(); }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        auto np = (nmb_params)idx;
        using pmd = sst::basic_blocks::params::ParamMetaData;
        switch (np)
        {
        case nmb_mode:
            return pmd()
                .asInt()
#if EURORACK_CLOUDS_IS_SUPERPARASITES
                .withRange(0, 7)
#else
                .withRange(0, 3)
#endif
                .withName("Mode")
                .withDefault(0)
                .withUnorderedMapFormatting({{0, "Granularizer"},
                                             {1, "Pitch Shifter"},
                                             {2, "Looping Delay"},
                                             {3, "Spectral Madness"},
                                             {4, "Oliverb"},
                                             {5, "Reonestor"},
                                             {6, "Kammerl"},
                                             {7, "Spectral Cloud"}});
            // TODO: Make this also marked as param-invalidating and use conditions for names
        case nmb_quality:
            return pmd()
                .asInt()
                .withRange(0, 3)
                .withName("Quality")
                .withDefault(0)
                .withUnorderedMapFormatting({{0, "32k 16-bit Stereo"},
                                             {1, "32k 16-bit Mono"},
                                             {2, "16k 8-bit Stereo"},
                                             {3, "16k 8-bit Mono"}});
        case nmb_position:
            return pmd().asPercent().withName("Position").withDefault(0.f);
        case nmb_size:
            return pmd().asPercentBipolar().withName("Size").withDefault(0.f);
        case nmb_pitch:
            return pmd().asSemitoneRange(-48, 48).withDefault(0.f).withName("Pitch");
        case nmb_density:
            return pmd().asPercentBipolar().withName("Density").withDefault(0.f);
        case nmb_texture:
            return pmd().asPercentBipolar().withName("Texture").withDefault(0.f);
        case nmb_spread:
            return pmd().asPercent().withName("Spread").withDefault(0.f);
        case nmb_freeze:
            // TODO: On/Off FOrmatting around 0.5 here
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(0.f)
                .withName("Freeze")
                .withLinearScaleFormatting("");
        case nmb_feedback:
            return pmd().asPercent().withName("Feedback").withDefault(0.f);
        case nmb_reverb:
            return pmd().asPercent().withName("Reverb").withDefault(0.f);
        case nmb_mix:
            return pmd().asPercent().withName("Mix").withDefault(0.5f);
        case nmb_num_params:
            break;
        }
        return {};
    }

    // Only used by rack
    void setNimbusTrigger(bool b) { nimbusTrigger = b; }

  protected:
    float L alignas(16)[FXConfig::blockSize], R alignas(16)[FXConfig::blockSize];

    sdsp::lipol_sse<FXConfig::blockSize, false> mix;

    uint8_t *block_mem, *block_ccm;
    clouds::GranularProcessor *processor;
    static constexpr int processor_sr = 32000;
    static constexpr float processor_sr_inv = 1.f / 32000;
    int old_nmb_mode = 0;
    bool nimbusTrigger{false};

    using resamp_t = sst::basic_blocks::dsp::LanczosResampler<FXConfig::blockSize>;
    std::unique_ptr<resamp_t> surgeSR_to_euroSR, euroSR_to_surgeSR;

    static constexpr int raw_out_sz = FXConfig::blockSize << 6; // power of 2 pls
    float resampled_output[2][raw_out_sz];                      // at sr
    size_t resampReadPtr = 0, resampWritePtr = 1;               // see comment in init

    static constexpr int nimbusprocess_blocksize = 8;
    float stub_input[2][nimbusprocess_blocksize]; // This is the extra sample we have around
    size_t numStubs{0};
    int consumed = 0, created = 0;
    bool builtBuffer{false};

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const param)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};

#endif

} // namespace sst::effects::nimbus

#endif
