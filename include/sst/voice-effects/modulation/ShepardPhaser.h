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
#include <cmath>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/modulators/SimpleLFO.h"

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
            return pmd().asLfoRate(-7, 0).withDefault(-3).withName("Rate");
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
        auto lfoRate = this->getFloatParam(fpRate);
        auto range = this->getFloatParam(fpEndFreq) - this->getFloatParam(fpStartFreq);
        auto res = std::clamp(this->getFloatParam(fpResonance) * .2f + .8f, 0.f, 1.0f);
        int peaks = this->getIntParam(ipPeaks);

        namespace mech = sst::basic_blocks::mechanics;

        if (isFirst)
        {
            phasor.attack(lfo_t::RAMP);
            isFirst = false;
        }

        mech::clear_block<VFXConfig::blockSize>(dataoutL);
        mech::clear_block<VFXConfig::blockSize>(dataoutR);

        phasor.process_block(lfoRate, 0.f, lfo_t::RAMP);
        auto phasorValue = phasor.lastTarget * .5f + .5f;

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
                sst::filters::CytomicSVF::Mode::BP, freq, freq, res, res, this->getSampleRateInv(),
                1.f, 1.f);

            float tmpL alignas(16)[VFXConfig::blockSize];
            float tmpR alignas(16)[VFXConfig::blockSize];
            mech::copy_from_to<VFXConfig::blockSize>(datainL, tmpL);
            mech::copy_from_to<VFXConfig::blockSize>(datainR, tmpR);

            for (int k = 0; k < VFXConfig::blockSize; ++k)
            {
                filters[i].processBlockStep(tmpL[k], tmpR[k]);
                softClip(tmpL[k], tmpR[k]);
            }

            lipolLevel[i].set_target(iTri * iTri * iTri);
            lipolLevel[i].multiply_2_blocks(tmpL, tmpR);

            mech::scale_accumulate_from_to<VFXConfig::blockSize>(tmpL, tmpR, 0.3333f, dataoutL,
                                                                 dataoutR);
        }
    }

    void softClip(float &L, float &R)
    {
        L = std::clamp(L, -1.5f, 1.5f);
        L = L - 4.0 / 27.0 * L * L * L;

        R = std::clamp(R, -1.5f, 1.5f);
        R = R - 4.0 / 27.0 * R * R * R;
    }

    void softClip(float &C)
    {
        C = std::clamp(C, -1.5f, 1.5f);
        C = C - 4.0 / 27.0 * C * C * C;
    }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};

    std::array<sst::filters::CytomicSVF, 12> filters;

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolLevel[12];
    bool isFirst{true};
};
} // namespace sst::voice_effects::modulation
#endif // SCXT_SHEPARD_PHASER_H
