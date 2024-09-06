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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARD_PHASER_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_SHEPARD_PHASER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"
#include "sst/basic-blocks/dsp/RNG.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct ShepardPhaser : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Shepard Phaser"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{2};

    basic_blocks::dsp::RNG rng;

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

    ShepardPhaser() : core::VoiceEffectTemplateBase<VFXConfig>() {}

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
        using md = sst::filters::CytomicSVF::Mode;

        switch (idx)
        {
        case ipStereo:
            return pmd().asBool().withDefault(false).withName("Stereo");
        case ipPeaks:
            return pmd().asInt().withRange(4, 12).withDefault(8).withName("Peaks");
        }
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<ShepardPhaser, VFXConfig::blockSize>;
    lfo_t phasor{this};

    float triangle(const float &p)
    {
        auto res = 0.f;
        if (p < 0.5)
        {
            res = 4 * p - 1.f;
        }
        else
        {
            res = (1.f - 4 * (p - 0.5));
        }
        return res * .5 + .5;
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto stereo = this->getIntParam(ipStereo);
        auto range = std::clamp(this->getFloatParam(fpEndFreq), -60.f, 70.f) -
                     std::clamp(this->getFloatParam(fpStartFreq), -60.f, 70.f);
        auto res = std::clamp(this->getFloatParam(fpResonance) * .08f + .9f, 0.f, .98f);
        int peaks = this->getIntParam(ipPeaks);
        if (peaks != priorPeaks)
        {
            logOfPeaks = std::log2f(peaks);
            priorPeaks = peaks;
        }
        auto lfoRate = this->getFloatParam(fpRate) - logOfPeaks;

        if (isFirst)
        {
            phasor.applyPhaseOffset(rng.unif01());
            isFirst = false;
        }
        phasor.process_block(lfoRate, 0.f, lfo_t::RAMP);
        auto phasorValue = phasor.lastTarget * .5f + .5f;

        namespace mech = sst::basic_blocks::mechanics;
        mech::clear_block<VFXConfig::blockSize>(dataoutL);
        mech::clear_block<VFXConfig::blockSize>(dataoutR);

        for (int i = 0; i < peaks; ++i)
        {
            auto offset = static_cast<double>(i) / static_cast<double>(peaks);
            auto halfway = 0.5 / static_cast<double>(peaks);
            auto iPhaseL = std::fmod(phasorValue + offset, 1.0);
            auto iPhaseR = !stereo ? iPhaseL : std::fmod(phasorValue + offset + halfway, 1.0);

            auto iTriL = triangle(iPhaseL);
            auto iTriR = triangle(iPhaseR);

            auto freqL = 440.f * this->note_to_pitch_ignoring_tuning(
                                     this->getFloatParam(fpStartFreq) + (range * iPhaseL));
            auto freqR = 440.f * this->note_to_pitch_ignoring_tuning(
                                     this->getFloatParam(fpStartFreq) + (range * iPhaseR));

            filters[i].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::BP, freqL, freqR, res, res,
                this->getSampleRateInv(), 1.f, 1.f);

            float tmpL alignas(16)[VFXConfig::blockSize];
            float tmpR alignas(16)[VFXConfig::blockSize];
            mech::copy_from_to<VFXConfig::blockSize>(datainL, tmpL);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, tmpR);

            for (int k = 0; k < VFXConfig::blockSize; ++k)
            {
                filters[i].processBlockStep(tmpL[k], tmpR[k]);
            }

            if (!stereo)
            {
                lipolLevel[i].set_target(iTriL * iTriL * iTriL);
                lipolLevel[i].multiply_2_blocks(tmpL, tmpR);
            }
            else
            {
                lipolLevel[i].set_target(iTriL * iTriL * iTriL);
                lipolLevel[i].multiply_block(tmpL);
                lipolLevel[i + 12].set_target(iTriR * iTriR * iTriR);
                lipolLevel[i + 12].multiply_block(tmpR);
            }

            mech::scale_accumulate_from_to<VFXConfig::blockSize>(tmpL, tmpR, 0.5f, dataoutL,
                                                                 dataoutR);
        }
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        auto range = std::clamp(this->getFloatParam(fpEndFreq), -60.f, 70.f) -
                     std::clamp(this->getFloatParam(fpStartFreq), -60.f, 70.f);
        auto res = std::clamp(this->getFloatParam(fpResonance) * .08f + .9f, 0.f, .98f);
        int peaks = this->getIntParam(ipPeaks);
        if (peaks != priorPeaks)
        {
            logOfPeaks = std::log2f(peaks);
            priorPeaks = peaks;
        }
        auto lfoRate = this->getFloatParam(fpRate) - logOfPeaks;

        if (isFirst)
        {
            phasor.applyPhaseOffset(rng.unif01());
            isFirst = false;
        }
        phasor.process_block(lfoRate, 0.f, lfo_t::RAMP);
        auto phasorValue = phasor.lastTarget * .5f + .5f;

        namespace mech = sst::basic_blocks::mechanics;
        mech::clear_block<VFXConfig::blockSize>(dataoutL);

        for (int i = 0; i < peaks; ++i)
        {

            float iPhase = phasorValue + (i / static_cast<float>(peaks));
            if (iPhase > 1)
            {
                iPhase -= 1;
            }
            float iTri = triangle(iPhase);

            auto freqMod = this->getFloatParam(fpStartFreq) + (range * iPhase);
            auto freq = 440.f * this->note_to_pitch_ignoring_tuning(freqMod);
            filters[i].template setCoeffForBlock<VFXConfig::blockSize>(
                sst::filters::CytomicSVF::Mode::BP, freq, res, this->getSampleRateInv(), 1.f);

            float tmp alignas(16)[VFXConfig::blockSize];
            mech::copy_from_to<VFXConfig::blockSize>(datainL, tmp);

            for (int k = 0; k < VFXConfig::blockSize; ++k)
            {
                filters[i].processBlockStep(tmp[k]);
            }

            lipolLevel[i].set_target(iTri * iTri * iTri);
            lipolLevel[i].multiply_block(tmp);

            mech::scale_accumulate_from_to<VFXConfig::blockSize>(tmp, 0.5f, dataoutL);
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};

    std::array<sst::filters::CytomicSVF, 12> filters;

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolLevel[24];
    float logOfPeaks{0.f};
    int priorPeaks{0};
    bool isFirst{true};
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_SHEPARD_PHASER_H
