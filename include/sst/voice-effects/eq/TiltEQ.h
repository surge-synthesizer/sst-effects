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

#ifndef INCLUDE_SST_VOICE_EFFECTS_EQ_TILTEQ_H
#define INCLUDE_SST_VOICE_EFFECTS_EQ_TILTEQ_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/filters/CytomicTilt.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <algorithm>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::eq
{
template <typename VFXConfig> struct TiltEQ : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Tilt EQ"};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpFreq,
        fpTilt,
    };

    TiltEQ() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~TiltEQ() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpFreq:
            return pmd().asAudibleFrequency().withName("Frequency");
        case fpTilt:
            return pmd()
                .asFloat()
                .withRange(-18, 18)
                .withDefault(0)
                .withLinearScaleFormatting("db")
                .withName("Tilt");
        }
        return pmd().asFloat().withName("Error");
    }

    void initVoiceEffect()
    {
        tilter.init();
        priorFreq = -9999.f;
        priorSlope = -9999.f;
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        setCoeffs();
        tilter.template processBlock<VFXConfig::blockSize>(datainL, datainR, dataoutL, dataoutR);
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        setCoeffs();
        tilter.template processBlock<VFXConfig::blockSize>(datainL, dataoutL);
    }

    void setCoeffs()
    {
        float freq = 440 * this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpFreq));
        // float slope = this->dbToLinear(std::clamp(this->getFloatParam(fpTilt), -18.f, 18.f) *
        // .5f);
        float slope = this->getFloatParam(fpTilt);

        if (slope != priorSlope || freq != priorFreq)
        {
            tilter.template setCoeffForBlock<VFXConfig::blockSize>(freq, .07f,
                                                                   this->getSampleRateInv(), slope);
            priorSlope = slope;
            priorFreq = freq;
        }
        else
        {
            tilter.retainCoeffForBlock();
        }
    }

    /*
    float getFrequencyGraph(float f)
    {

    }
    */

  protected:
    sst::filters::CytomicTilt tilter;
    float priorSlope = -123456.f;
    float priorFreq = -123456.f;

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

} // namespace sst::voice_effects::eq

#endif // SHORTCIRCUITXT_TiltEQ_H
