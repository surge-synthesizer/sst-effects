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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASEMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_PHASEMOD_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/FastMath.h"
#include "sst/filters/HalfRateFilter.h"

#include "../VoiceEffectCore.h"

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct PhaseMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"Phase Mod"};
    static constexpr const char *streamingName{"osc-phase-mod"};
    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpTranspose,
        fpLowpass,
        fpDepth,
    };

    PhaseMod() : core::VoiceEffectTemplateBase<VFXConfig>(), upFilter(6, true), downFilter(6, true)
    {
    }

    ~PhaseMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpTranspose:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-128, 128)
                    .withSemitoneFormatting()
                    .withDefault(-12)
                    .withName("Offset");
            }
            return pmd().asAudibleFrequency().withRange(-140, 70).withDefault(0).withName(
                "Frequency");
        case fpLowpass:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-128, 128)
                    .withSemitoneFormatting()
                    .withDefault(128)
                    .deactivatable()
                    .withName("Lowpass Offset");
            }
            return pmd().asAudibleFrequency().withDefault(70).deactivatable().withName(
                "Lowpass Frequency");
        case fpDepth:
            return pmd().asDecibel().withName("Depth").withDefault(0);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect() { lpf.init(); }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;
        namespace mech = sst::basic_blocks::mechanics;
        using mode = sst::filters::CytomicSVF::Mode;

        auto freq = this->getFloatParam(fpTranspose);
        auto lpfreq = this->getFloatParam(fpLowpass);
        if (keytrackOn)
        {
            freq += pitch;
            lpfreq += pitch;
        }

        float modulator alignas(16)[2][VFXConfig::blockSize];

        if (this->getIsDeactivated(fpLowpass))
        {
            mech::copy_from_to<VFXConfig::blockSize>(datainL, modulator[0]);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, modulator[1]);
        }
        else
        {
            lpfreq = 440 * this->note_to_pitch_ignoring_tuning(lpfreq);
            lpf.setCoeffForBlock<VFXConfig::blockSize>(mode::Lowpass, lpfreq, 0.f,
                                                       this->getSampleRateInv());
            lpf.processBlock<VFXConfig::blockSize>(datainL, datainR, modulator[0], modulator[1]);
        }

        omegaLerp.set_target(440 * this->note_to_pitch_ignoring_tuning(freq) * M_PI_2 *
                             this->getSampleRateInv());

        auto lvl = this->getFloatParam(fpDepth);
        auto lvlComp = -lvl * (1 - .75f * !std::signbit(lvl));

        preGainLerp.set_target(3.1415f * this->dbToLinear(lvl));
        postGainLerp.set_target(3.1415f * this->dbToLinear(lvlComp));

        constexpr int bs2 = VFXConfig::blockSize << 1;
        float OS alignas(16)[2][bs2];
        float omInterp alignas(16)[bs2];
        float phVals alignas(16)[bs2];

        preGainLerp.multiply_2_blocks_to(modulator[0], modulator[1], OS[0], OS[1]);
        upFilter.process_block_U2(OS[0], OS[1], OS[0], OS[1], bs2);

        omegaLerp.store_block(omInterp);
        phVals[0] = phase + omInterp[0];
        for (int k = 1; k < bs2; ++k)
        {
            phVals[k] = phVals[k - 1] + omInterp[k];
        }
        phase = phVals[bs2 - 1];
        if (phase > M_PI)
            phase -= 2 * M_PI;

        const auto half = SIMD_MM(set1_ps)(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = SIMD_MM(load_ps)(phVals + k);
            auto s0 = SIMD_MM(load_ps)(OS[0] + k);
            auto s1 = SIMD_MM(load_ps)(OS[1] + k);
            auto w0 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s0, ph));
            auto w1 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s1, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w0), sph));
            auto r1 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w1), sph));
            SIMD_MM(store_ps)(OS[0] + k, r0);
            SIMD_MM(store_ps)(OS[1] + k, r1);
        }

        downFilter.process_block_D2(OS[0], OS[1], bs2, dataoutL, dataoutR);
        postGainLerp.multiply_2_blocks(dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        namespace sdsp = sst::basic_blocks::dsp;
        namespace mech = sst::basic_blocks::mechanics;
        using mode = sst::filters::CytomicSVF::Mode;

        auto freq = this->getFloatParam(fpTranspose);
        auto lpfreq = this->getFloatParam(fpLowpass);
        if (keytrackOn)
        {
            freq += pitch;
            lpfreq += pitch;
        }

        float modulator alignas(16)[VFXConfig::blockSize];

        if (this->getIsDeactivated(fpLowpass))
        {
            mech::copy_from_to<VFXConfig::blockSize>(datain, modulator);
        }
        else
        {
            lpfreq = 440 * this->note_to_pitch_ignoring_tuning(lpfreq);
            lpf.setCoeffForBlock<VFXConfig::blockSize>(mode::Lowpass, lpfreq, 0.f,
                                                       this->getSampleRateInv());
            lpf.processBlock<VFXConfig::blockSize>(datain, modulator);
        }

        omegaLerp.set_target(440 * this->note_to_pitch_ignoring_tuning(freq) * M_PI_2 *
                             this->getSampleRateInv());

        auto lvl = this->getFloatParam(fpDepth);
        auto lvlComp = -lvl * (1 - .75f * !std::signbit(lvl));

        preGainLerp.set_target(3.1415f * this->dbToLinear(lvl));
        postGainLerp.set_target(3.1415f * this->dbToLinear(lvlComp));

        constexpr int bs2 = VFXConfig::blockSize << 1;
        float OS alignas(16)[2][bs2];
        float omInterp alignas(16)[bs2];
        float phVals alignas(16)[bs2];

        preGainLerp.multiply_block_to(modulator, OS[0]);
        upFilter.process_block_U2(OS[0], OS[0], OS[0], OS[1], bs2);

        omegaLerp.store_block(omInterp);
        phVals[0] = phase + omInterp[0];
        for (int k = 1; k < bs2; ++k)
        {
            phVals[k] = phVals[k - 1] + omInterp[k];
        }
        phase = phVals[bs2 - 1];
        if (phase > M_PI)
            phase -= 2 * M_PI;

        const auto half = SIMD_MM(set1_ps)(0.5f);
        for (int k = 0; k < bs2; k += 4)
        {
            auto ph = SIMD_MM(load_ps)(phVals + k);
            auto s0 = SIMD_MM(load_ps)(OS[0] + k);
            auto w0 = sdsp::clampToPiRangeSSE(SIMD_MM(add_ps)(s0, ph));
            ph = sdsp::clampToPiRangeSSE(ph);
            auto sph = sdsp::fastsinSSE(ph);

            auto r0 = SIMD_MM(mul_ps)(half, SIMD_MM(sub_ps)(sdsp::fastsinSSE(w0), sph));
            SIMD_MM(store_ps)(OS[0] + k, r0);
        }

        downFilter.process_block_D2(OS[0], OS[0], bs2, dataout, 0);
        postGainLerp.multiply_block(dataout);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }

  protected:
    bool keytrackOn{true};
    double phase{0.0};
    sst::filters::HalfRate::HalfRateFilter upFilter, downFilter;
    sst::filters::CytomicSVF lpf;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> preGainLerp, postGainLerp;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize << 1, true> omegaLerp;

  public:
    static constexpr int16_t streamingVersion{3};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam,
                                                   uint32_t *const streamingParams)
    {
        assert(streamedFrom <= streamingVersion);
        if (streamedFrom == 1)
        {
            // Between version 1 and 2 keytrack was all or nothing so just check param 0
            // turns out we didn't need this here anyway but keep it as an example
            // just in case
            [[maybe_unused]] auto waskt =
                streamingParams[0] & (uint32_t)voice_effects::core::StreamingFlags::IS_KEYTRACKED;
            // used to be 0 and 1 were ratio
            auto n = iparam[0];
            auto d = iparam[1];
            if (d == 0)
            {
                return;
            }

            fparam[0] += 12 * std::log2(1.f * n / d);
        }
        if (streamedFrom == 2)
        {
            fparam[0] -= 12.0f;
        }
    }
};
} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_PhaseMod_H
