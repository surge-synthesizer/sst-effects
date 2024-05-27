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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DISTORTION_BITCRUSHER_H
#define INCLUDE_SST_VOICE_EFFECTS_DISTORTION_BITCRUSHER_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

namespace sst::voice_effects::distortion
{
template <typename VFXConfig> struct BitCrusher : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"BitCrusher"};

    enum bc_fparams
    {
        bc_samplerate,
        bc_bitdepth,
        bc_zeropoint,
        bc_cutoff,
        bc_resonance,
        bc_num_params
    };

    static constexpr int numFloatParams{bc_num_params};
    static constexpr int numIntParams{0};

    BitCrusher() : core::VoiceEffectTemplateBase<VFXConfig>(), lp(this) {}

    ~BitCrusher() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((bc_fparams)idx)
        {
        case bc_samplerate:
            return pmd()
                .asAudibleFrequency()
                .withRange(-12, 80)
                .withName("sampleRate")
                .withDefault(7.5);
        case bc_bitdepth:
            return pmd().asPercent().withName("bitdepth").withDefault(1.f);
        case bc_zeropoint:
            return pmd().asPercent().withName("zeropoint").withDefault(0.f);
        case bc_cutoff:
            return pmd().asAudibleFrequency().withName("cutoff");
        case bc_resonance:
            return pmd().asPercent().withName("resonance").withDefault(0.707f);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        float t = this->getSampleRateInv() * 440 *
                  this->equalNoteToPitch(this->getFloatParam(bc_samplerate));
        float bd = 16.f * std::min(1.f, std::max(0.f, this->getFloatParam(bc_bitdepth)));
        float b = powf(2, bd), b_inv = 1.f / b;

        int k;
        float dVal = this->getFloatParam(bc_zeropoint);
        for (k = 0; k < VFXConfig::blockSize; k++)
        {
            float inputL = datainL[k];
            float inputR = datainR[k];
            time[0] -= t;
            time[1] -= t;
            if (time[0] < 0.f)
            {
                float val = inputL * b;
                val += dVal;
                level[0] = (float)((int)val) * b_inv;
                time[0] += 1.0f;
                time[0] = std::max(time[0], 0.f);
            }
            if (time[1] < 0.f)
            {
                float val = inputR * b;
                val += dVal;
                level[1] = (float)((int)val) * b_inv;
                time[1] += 1.0f;
                time[1] = std::max(time[1], 0.f);
            }
            dataoutL[k] = level[0];
            dataoutR[k] = level[1];
        }

        lp.coeff_LP2B(lp.calc_omega(this->getFloatParam(bc_cutoff) / 12.f),
                      std::clamp(this->getFloatParam(bc_resonance), 0.001f, 0.999f));
        lp.process_block(dataoutL, dataoutR);
    }

  protected:
    float time[2]{0.f, 0.f}, level[2]{0.f, 0.f};
    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType lp;
};
} // namespace sst::voice_effects::distortion

#endif // SHORTCIRCUITXT_BITCRUSHER_H
