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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENSIN_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_GENSIN_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct GenSin : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Generate Sin"};

    enum struct GenSinFloatParams : uint32_t
    {
        offset,
        level,
        num_params
    };

    enum struct GenSinIntParams : uint32_t
    {
        num_params
    };

    static constexpr int numFloatParams{(int)GenSinFloatParams::num_params};
    static constexpr int numIntParams{(int)GenSinIntParams::num_params};

    GenSin() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~GenSin() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < (int)GenSinFloatParams::num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((GenSinFloatParams)idx)
        {
        case GenSinFloatParams::offset:
            return pmd()
                .asFloat()
                .withRange(-96, 96)
                .withDefault(0)
                .withLinearScaleFormatting("semitones")
                .withName("Tune");
        case GenSinFloatParams::level:
            return pmd().asCubicDecibelAttenuation().withDefault(0.5f).withName("Level");
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
        mQuadOsc.setRate(440.0 * 2 * M_PI *
                         this->note_to_pitch_ignoring_tuning(
                             this->getFloatParam((int)GenSinFloatParams::offset) + pitch) *
                         this->getSampleRateInv());
        auto levT = std::clamp(this->getFloatParam((int)GenSinFloatParams::level), 0.f, 1.f);
        levT = levT * levT * levT;
        mLevelLerp.set_target(levT);
        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            dataoutL[k] = mQuadOsc.v;
            mQuadOsc.step();
        }
        mLevelLerp.multiply_block(dataoutL);
    }

  protected:
    sst::basic_blocks::dsp::QuadratureOscillator<float> mQuadOsc;
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> mLevelLerp;
};
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GenSin_H
