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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITIES_STEREOTOOL_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITIES_STEREOTOOL_H

#include "../VoiceEffectCore.h"

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/effects-shared/WidthProvider.h"
#include "sst/basic-blocks/dsp/PanLaws.h"

namespace sst::voice_effects::utilities
{
template <typename VFXConfig>
struct StereoTool : core::VoiceEffectTemplateBase<VFXConfig>,
                    effects_shared::WidthProvider<StereoTool<VFXConfig>, VFXConfig::blockSize, true>
{
    static constexpr const char *displayName{"Stereo Tool"};
    static constexpr const char *streamingName{"stereo-tool"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{0};
    static constexpr float halfPi{1.5707963};

    enum FloatParams
    {
        fpInputPan,
        fpRotation,
        fpWidth,
        fpOutputBalance
    };

    StereoTool() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~StereoTool() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpInputPan:
            return pmd().asPan().withName("Input Pan");
        case fpRotation:
            return pmd()
                .asFloat()
                .withRange(-halfPi, halfPi)
                .withDefault(0.f)
                .withName("Rotation")
                .withLinearScaleFormatting("°", 90 / halfPi);
        case fpWidth:
            return this->getWidthParam().withName("Width");
        case fpOutputBalance:
            return pmd().asPan().withName("Output Balance");
        }
        return pmd().asFloat().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        return pmd().asInt().withName("Error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace mech = basic_blocks::mechanics;
        namespace pan = basic_blocks::dsp::pan_laws;

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        pan::stereoEqualPower((this->getFloatParam(fpInputPan) + 1) / 2, preMatrix);
        preLerpL.set_target(preMatrix[0]);
        preLerpR.set_target(preMatrix[1]);
        preLerpL.multiply_block(dataoutL);
        preLerpR.multiply_block(dataoutR);

        sinLerp.set_target(sin(this->getFloatParam(fpRotation)));
        float s alignas(16)[VFXConfig::blockSize];
        sinLerp.store_block(s);

        cosLerp.set_target(cos(this->getFloatParam(fpRotation)));
        float c alignas(16)[VFXConfig::blockSize];
        cosLerp.store_block(c);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto inL = dataoutL[i];
            dataoutL[i] = inL * c[i] - dataoutR[i] * s[i];
            dataoutR[i] = inL * s[i] + dataoutR[i] * c[i];
        }

        this->setWidthTarget(widthLerpS, widthLerpM, fpWidth);
        this->applyWidth(dataoutL, dataoutR, widthLerpS, widthLerpM);

        pan::stereoEqualPower((this->getFloatParam(fpOutputBalance) + 1) / 2, postMatrix);
        postLerpL.set_target(postMatrix[0]);
        postLerpR.set_target(postMatrix[1]);
        postLerpL.multiply_block(dataoutL);
        postLerpR.multiply_block(dataoutR);
    }

  protected:
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> sinLerp, cosLerp, widthLerpS,
        widthLerpM, preLerpL, preLerpR, postLerpL, postLerpR;

    basic_blocks::dsp::pan_laws::panmatrix_t preMatrix{1, 1, 0, 0};
    basic_blocks::dsp::pan_laws::panmatrix_t postMatrix{1, 1, 0, 0};

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
} // namespace sst::voice_effects::utilities
#endif // SCXT_STEREOTOOL_H
