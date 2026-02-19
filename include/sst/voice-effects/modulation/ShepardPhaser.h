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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARDPHASER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARDPHASER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/RNG.h"
#include "sst/basic-blocks/simd/setup.h"

#include <cmath>

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct ShepardPhaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"Shepard Phaser"};
    static constexpr const char *streamingName{"shepard"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{2};

    basic_blocks::dsp::RNG &rng;

    enum FloatParams
    {
        fpResonance,
        fpRate,
        fpStartFreq,
        fpEndFreq,
    };

    enum IntParams
    {
        ipStereo,
        ipPeaks
    };

    ShepardPhaser(basic_blocks::dsp::RNG &extrng)
        : core::VoiceEffectTemplateBase<VFXConfig>(), rng(extrng)
    {
    }

    ~ShepardPhaser() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpResonance:
            return pmd().asPercent().withDefault(1.0f).withName("Resonance");
        case fpRate:
            return pmd().asLfoRate(-7, 3).withDefault(0).withName("Rate");
        case fpStartFreq:
            return pmd().asAudibleFrequency().withName("Start Freq").withDefault(-30);
        case fpEndFreq:
            return pmd().asAudibleFrequency().withName("End Freq").withDefault(40);
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipPeaks:
            return pmd()
                .asInt()
                .withRange(4, 12)
                .withDefault(8)
                .withName("Peaks")
                .withLinearScaleFormatting("peaks");
            ;
        }
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect()
    {
        for (auto &f : filtersL)
        {
            f.init();
        }
        for (auto &f : filtersR)
        {
            f.init();
        }

        int peaks = this->getIntParam(ipPeaks);
        quadsToProcess = std::ceil((float)peaks / 4.f);
        halfway = 0.5 / static_cast<double>(peaks);
        logOfPeaks = std::log2f(static_cast<float>(peaks));
        gainScale = 1 / logOfPeaks;
        priorPeaks = peaks;

        phasorValue = rng.unif01();
        for (int i = 0; i < peaks; ++i)
        {
            auto offset = static_cast<double>(i) / static_cast<double>(peaks);
            auto pL = phasorValue + offset;
            phaseL[i] = pL - (int)pL;

            if (this->getIntParam(ipStereo))
            {
                auto pR = phasorValue + offset + halfway;
                phaseR[i] = pR - (int)pR;
            }
            else
            {
                phaseR[i] = phaseL[i];
            }

            // triangle for amplitude
            auto iTriL = triangle(phaseL[i]);
            auto iTriR = triangle(phaseR[i]);
            levelLerpL[i].set_target_instant(iTriL);
            levelLerpR[i].set_target_instant(iTriR);
        }
        for (int i = peaks; i < 12; ++i)
        {
            levelLerpL[i].set_target_instant(0.f);
            levelLerpR[i].set_target_instant(0.f);
            freqsL[i] = 0.00001f;
            freqsR[i] = 0.00001f;
        }
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    float triangle(const float p)
    {
        auto res = -(std::fabs(-2 * p + 1)) + 1;
        return res * res * res;
    }

#define MUL(a, b) SIMD_MM(mul_ps)(a, b)
#define ADD(a, b) SIMD_MM(add_ps)(a, b)
    SIMD_M128 triangle(const SIMD_M128 p)
    {
        namespace mech = sst::basic_blocks::mechanics;
        //  -(abs(-2 * p + 1)) + 1
        auto res = ADD(MUL(negOneSSE, mech::abs_ps(ADD(MUL(negTwoSSE, p), oneSSE))), oneSSE);
        // goes 0...1...0 as phase goes 0...1
        // then cube that
        return MUL(res, MUL(res, res));
    }
#undef MUL
#undef ADD

    template <bool stereo>
    void processStereoImpl(const float *const datainL, const float *const datainR, float *dataoutL,
                           float *dataoutR, float pitch)
    {
        auto range = std::clamp(this->getFloatParam(fpEndFreq), -60.f, 70.f) -
                     std::clamp(this->getFloatParam(fpStartFreq), -60.f, 70.f);
        auto res = std::clamp(this->getFloatParam(fpResonance) * .08f + .9f, 0.f, .98f);
        int peaks = this->getIntParam(ipPeaks);
        if (peaks != priorPeaks)
        {
            this->initVoiceEffect();
        }

        auto phaseInc =
            this->envelope_rate_linear_nowrap(-(this->getFloatParam(fpRate) - logOfPeaks));
        phasorValue += phaseInc;
        phasorValue -= (int)phasorValue;

        namespace mech = sst::basic_blocks::mechanics;
        mech::clear_block<VFXConfig::blockSize>(dataoutL);
        mech::clear_block<VFXConfig::blockSize>(dataoutR);

        for (int i = 0; i < peaks; ++i)
        {
            auto offset = static_cast<double>(i) / static_cast<double>(peaks);
            auto pL = phasorValue + offset;
            phaseL[i] = pL - (int)pL;
            freqsL[i] = 440.f * this->note_to_pitch_ignoring_tuning(
                                    this->getFloatParam(fpStartFreq) + (range * phaseL[i]));

            if constexpr (stereo)
            {
                auto pR = phasorValue + offset + halfway;
                phaseR[i] = pR - (int)pR;
                freqsR[i] = 440.f * this->note_to_pitch_ignoring_tuning(
                                        this->getFloatParam(fpStartFreq) + (range * phaseR[i]));
            }
            else
            {
                phaseR[i] = phaseL[i];
                freqsR[i] = freqsL[i];
            }
        }

        float outL alignas(16)[12][VFXConfig::blockSize];
        float outR alignas(16)[12][VFXConfig::blockSize];
        float triL alignas(16)[12];
        float triR alignas(16)[12];
        for (int i = 0; i < quadsToProcess; ++i)
        {
            int regidx = i * 4;

            auto tL = triangle(SIMD_MM(set_ps)(phaseL[3 + regidx], phaseL[2 + regidx],
                                               phaseL[1 + regidx], phaseL[0 + regidx]));
            auto tR = triangle(SIMD_MM(set_ps)(phaseR[3 + regidx], phaseR[2 + regidx],
                                               phaseR[1 + regidx], phaseR[0 + regidx]));
            SIMD_MM(store_ps)(&triL[regidx], tL);
            SIMD_MM(store_ps)(&triR[regidx], tR);

            filtersL[i].template setCoeffForBlockQuadBandpass<VFXConfig::blockSize>(
                &freqsL[regidx], res, this->getSampleRateInv());
            filtersR[i].template setCoeffForBlockQuadBandpass<VFXConfig::blockSize>(
                &freqsR[regidx], res, this->getSampleRateInv());

            filtersL[i].template processBlockQuad<VFXConfig::blockSize>(
                datainL, outL[0 + regidx], outL[1 + regidx], outL[2 + regidx], outL[3 + regidx]);
            filtersR[i].template processBlockQuad<VFXConfig::blockSize>(
                datainR, outR[0 + regidx], outR[1 + regidx], outR[2 + regidx], outR[3 + regidx]);
        }

        for (int i = 0; i < peaks; ++i)
        {
            levelLerpL[i].set_target(triL[i]);
            levelLerpR[i].set_target(triR[i]);
            levelLerpL[i].multiply_block(outL[i]);
            levelLerpR[i].multiply_block(outR[i]);

            mech::scale_accumulate_from_to<VFXConfig::blockSize>(outL[i], outR[i], gainScale,
                                                                 dataoutL, dataoutR);
        }
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        auto range = std::clamp(this->getFloatParam(fpEndFreq), -60.f, 70.f) -
                     std::clamp(this->getFloatParam(fpStartFreq), -60.f, 70.f);
        auto res = std::clamp(this->getFloatParam(fpResonance) * .08f + .9f, 0.f, .98f);
        int peaks = this->getIntParam(ipPeaks);
        if (peaks != priorPeaks)
        {
            this->initVoiceEffect();
        }

        auto phaseInc =
            this->envelope_rate_linear_nowrap(-(this->getFloatParam(fpRate) - logOfPeaks));
        phasorValue += phaseInc;
        phasorValue -= (int)phasorValue;

        namespace mech = sst::basic_blocks::mechanics;
        mech::clear_block<VFXConfig::blockSize>(dataout);

        for (int i = 0; i < peaks; ++i)
        {
            auto offset = static_cast<double>(i) / static_cast<double>(peaks);
            auto pL = phasorValue + offset;
            phaseL[i] = pL - (int)pL;
            freqsL[i] = 440.f * this->note_to_pitch_ignoring_tuning(
                                    this->getFloatParam(fpStartFreq) + (range * phaseL[i]));
        }

        float out alignas(16)[12][VFXConfig::blockSize];
        float tri alignas(16)[12];
        for (int i = 0; i < quadsToProcess; ++i)
        {
            int regidx = i * 4;

            auto t = triangle(SIMD_MM(set_ps)(phaseL[3 + regidx], phaseL[2 + regidx],
                                              phaseL[1 + regidx], phaseL[0 + regidx]));
            SIMD_MM(store_ps)(&tri[regidx], t);

            filtersL[i].template setCoeffForBlockQuadBandpass<VFXConfig::blockSize>(
                &freqsL[regidx], res, this->getSampleRateInv());

            filtersL[i].template processBlockQuad<VFXConfig::blockSize>(
                datain, out[0 + regidx], out[1 + regidx], out[2 + regidx], out[3 + regidx]);
        }

        for (int i = 0; i < peaks; ++i)
        {
            levelLerpL[i].set_target(tri[i]);
            levelLerpL[i].multiply_block(out[i]);

            mech::scale_accumulate_from_to<VFXConfig::blockSize>(out[i], gainScale, dataout);
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipStereo))
        {
            processStereoImpl<true>(datainL, datainL, dataoutL, dataoutR, pitch);
        }
        else
        {
            processStereoImpl<false>(datainL, datainL, dataoutL, dataoutR, pitch);
        }
    }

    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereoImpl<true>(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    size_t silentSamplesLength() const { return 10; }

  protected:
    std::array<sst::filters::CytomicSVF, 3> filtersL, filtersR;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> levelLerpL[12], levelLerpR[12];

    float phaseL[12], phaseR[12], freqsL[12], freqsR[12];

    int priorPeaks{0};
    double halfway{1.f};
    float logOfPeaks{0.f};
    float gainScale{1.f};
    double phasorValue{0.f};
    int quadsToProcess{1};

    const SIMD_M128 oneSSE = SIMD_MM(set1_ps(1.f));
    const SIMD_M128 negOneSSE = SIMD_MM(set1_ps(-1.f));
    const SIMD_M128 negTwoSSE = SIMD_MM(set1_ps(-2.f));

  public:
    static constexpr int16_t streamingVersion{1};
    static void remapParametersForStreamingVersion(int16_t streamedFrom, float *const fparam,
                                                   int *const iparam)
    {
        // base implementation - we have never updated streaming
        // input is parameters from stream version
        assert(streamedFrom == 1);
    }
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_SHEPARD_PHASER_H
