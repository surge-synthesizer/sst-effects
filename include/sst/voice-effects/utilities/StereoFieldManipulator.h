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

#ifndef INCLUDE_SST_VOICE_EFFECTS_UTILITIES_STEREOFIELDMANIP_H
#define INCLUDE_SST_VOICE_EFFECTS_UTILITIES_STEREOFIELDMANIP_H

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/PanLaws.h"

namespace sst::voice_effects::utilities
{
template <typename VFXConfig>
struct StereoFieldManipulator : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Stereo Field Manipulator"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpRotation,
        fpWidth,
        fpCenter,
        fpLeftRight
    };

    StereoFieldManipulator() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~StereoFieldManipulator() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpRotation:
            return pmd().asFloat().withRange(-90.f, 90.f).withDefault(0.f).withName("Rotation");
        case fpWidth:
            return pmd().asPercent().withDefault(1.f).withRange(0.f, 2.f).withName("Width");
        }
        return pmd().asFloat().withName("Error");
    }

    void initVoiceEffect() {}

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        auto rotRadians = this->getFloatParam(fpRotation) * 0.017453292; // degrees * PI/180
        auto width = this->getFloatParam(fpWidth) / 2.f;
        std::cout << rotRadians << std::endl;

        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
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

            dataoutL[i] = sL;
            dataoutR[i] = sR;
        }
    }

    void processMonoToStereo(float *datainL, float *dataoutL, float *dataoutR, float pitch)
    {
        processStereo(datainL, datainL, dataoutL, dataoutR, pitch);
    }
};
} // namespace sst::voice_effects::utilities
#endif // SCXT_STEREOFIELDMANIP_H
