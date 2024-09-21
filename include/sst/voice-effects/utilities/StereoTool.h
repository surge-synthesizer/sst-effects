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

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/PanLaws.h"

namespace sst::voice_effects::utilities
{
template <typename VFXConfig> struct StereoTool : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Stereo Tool"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpRotation,
        fpWidth,
        fpMidSide,
        fpLeftRight
    };

    StereoTool() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~StereoTool() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpRotation:
            return pmd()
                .asFloat()
                .withRange(-90.f, 90.f)
                .withDefault(0.f)
                .withName("Rotation")
                .withLinearScaleFormatting("º");
        case fpWidth:
            return pmd().asPercent().withDefault(1.f).withRange(0.f, 2.f).withName("Width");
        case fpMidSide:
            return pmd().asPercent().withDefault(0.f).withRange(-1.f, 1.f).withName("Mid|Side");
        case fpLeftRight:
            return pmd().asPercent().withDefault(0.f).withRange(-1.f, 1.f).withName("Left|Right");
        }
        return pmd().asFloat().withName("Error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {

        rotLerp.set_target(this->getFloatParam(fpRotation));
        float rot alignas(16)[VFXConfig::blockSize];
        rotLerp.store_block(rot);

        widthLerp.set_target(this->getFloatParam(fpWidth));
        float w alignas(16)[VFXConfig::blockSize];
        widthLerp.store_block(w);

        msLerp.set_target(this->getFloatParam(fpMidSide));
        float ms alignas(16)[VFXConfig::blockSize];
        msLerp.store_block(ms);

        lrLerp.set_target(this->getFloatParam(fpLeftRight));
        float lr alignas(16)[VFXConfig::blockSize];
        lrLerp.store_block(lr);

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            auto rotRadians = rot[i] * 0.017453292; // degrees * PI/180
            auto width = w[i] / 2.f;
            auto side = std::min(ms[i] + 1, 1.f);
            auto center = 1 - ms[i];
            auto left = -std::min(lr[i], 0.f);
            auto left1 = -std::max(lr[i] - 1, -1.f);
            auto right = std::max(lr[i], 0.f);
            auto right1 = std::min(1 + lr[i], 1.f);

            auto sL = datainL[i];
            auto sR = datainR[i];

            // Rotation
            auto signL = sL >= 0 ? 1 : -1;
            auto signR = sR >= 0 ? 1 : -1;

            auto angle = atan(sL / sR);
            if ((signL == 1 && signR == -1) || (signL == -1 && signR == -1))
                angle += 3.141592654;
            if (signL == -1 && signR == 1)
                angle += 6.283185307;
            if (sR == 0)
            {
                if (sL > 0)
                    angle = 1.570796327;
                else
                    angle = 4.71238898;
            }
            if (sL == 0)
            {
                if (sR > 0)
                    angle = 0;
                else
                    angle = 3.141592654;
            }
            angle -= rotRadians;
            auto radius = sqrt(sL * sL + sR * sR);
            sL = sin(angle) * radius;
            sR = cos(angle) * radius;

            // 3 Way Balancer + Enhancer
            auto mono = (sL + sR) / 2 * center;
            auto stereo = (sL - sR) * side;
            sL = (mono + (stereo * left1 - stereo * right) * width) / std::max(width, 1.f);
            sR = (mono + (-stereo * right1 + stereo * left) * width) / std::max(width, 1.f);

            dataoutL[i] = sL;
            dataoutR[i] = sR;
        }
    }

  protected:
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> rotLerp, widthLerp, msLerp,
        lrLerp;
};
} // namespace sst::voice_effects::utilities
#endif // SCXT_STEREOTOOL_H
