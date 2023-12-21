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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENSAW_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENSAW_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/DPWSawPulseOscillator.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenSaw : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Generate Sin"};

    enum struct GenSawFloatParams : uint32_t
    {
        offset,
        level,
        num_params
    };

    enum struct GenSawIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)GenSawFloatParams::num_params};
    static constexpr int numIntParams{(int)GenSawIntParams::num_params};

    GenSaw() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~GenSaw() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)GenSawFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((GenSawFloatParams)idx)
        {
        case GenSawFloatParams::offset:
            return pmd()
                .asFloat()
                .withRange(-96, 96)
                .withDefault(0)
                .withLinearScaleFormatting("semitones")
                .withName("Tune");
        case GenSawFloatParams::level:
            return pmd().asCubicDecibelAttenuation().withDefault(1.f).withName("Level");
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        processMonoToMono(datainL, dataoutL, pitch);
        sst::basic_blocks::mechanics::copy_from_to<VFXConfig::blockSize>(dataoutL, dataoutR);
    }
    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        mSawOsc.setFrequency(440.0 *
                                 this->note_to_pitch_ignoring_tuning(
                                     this->getFloatParam((int)GenSawFloatParams::offset) + pitch),
                             this->getSampleRateInv());
        auto levT = std::clamp(this->getFloatParam((int)GenSawFloatParams::level), 0.f, 1.f);
        levT = levT * levT * levT;
        mLevelLerp.set_target(levT);

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = mSawOsc.step();
        }
        mLevelLerp.multiply_block(dataoutL);
    }

  protected:
    sst::basic_blocks::dsp::DPWSawOscillator<
        sst::basic_blocks::dsp::BlockInterpSmoothingStrategy<VFXConfig::blockSize>>
        mSawOsc;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> mLevelLerp;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenSaw_H
