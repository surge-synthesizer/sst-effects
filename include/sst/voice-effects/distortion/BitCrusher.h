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

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{1};

    enum FloatParams
    {
        fpSamplerate,
        fpBitdepth,
        fpZeropoint,
        fpCutoff,
        fpResonance,
    };

    enum IntParams
    {
        ipFilterSwitch
    };

    BitCrusher() : core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~BitCrusher() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpSamplerate:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(0, 96)
                    .withName("Samplerate Ratio")
                    .withDefault(96)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withRange(-12, 80)
                .withName("Samplerate")
                .withDefault(80);
        case fpBitdepth:
            return pmd().asPercent().withName("bitdepth").withDefault(1.f);
        case fpZeropoint:
            return pmd().asPercent().withName("zeropoint").withDefault(0.f);
        case fpCutoff:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(0, 96)
                    .withName("Cutoff Offset")
                    .withDefault(96)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Cutoff");
        case fpResonance:
            return pmd().asPercent().withName("resonance").withDefault(0.707f);
        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipFilterSwitch:
            return pmd().asBool().withName("Filter Engage");
            break;
        }
        return pmd().withName("oops");
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        bool filterSwitch = this->getIntParam(ipFilterSwitch);
        float sRate = this->getFloatParam(fpSamplerate);
        if (keytrackOn)
        {
            sRate += pitch;
        }
        float t = this->getSampleRateInv() * 440 * this->equalNoteToPitch(sRate);
        float bd = 16.f * std::min(1.f, std::max(0.f, this->getFloatParam(fpBitdepth)));
        float b = powf(2, bd), b_inv = 1.f / b;

        auto reso = std::clamp(this->getFloatParam(fpResonance), 0.f, 1.f);
        auto cutoff = this->getFloatParam(fpCutoff);
        if (keytrackOn)
        {
            cutoff += pitch;
        }
        auto filterFreq = 440 * this->note_to_pitch_ignoring_tuning(cutoff);

        if (priorFreq != filterFreq && filterSwitch == true)
        {
            filter.template setCoeffForBlock<VFXConfig::blockSize>(
                filters::CytomicSVF::LP, filterFreq, reso, this->getSampleRateInv(), 0.f);
        }

        int k;
        float dVal = this->getFloatParam(fpZeropoint);
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
        if (filterSwitch == true)
        {
            filter.processBlock<VFXConfig::blockSize>(dataoutL, dataoutR, dataoutL, dataoutR);
        }
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }

  protected:
    float time[2]{0.f, 0.f}, level[2]{0.f, 0.f};
    bool keytrackOn{false};
    float priorFreq = -1.f;
    sst::filters::CytomicSVF filter;
};
} // namespace sst::voice_effects::distortion

#endif // SHORTCIRCUITXT_BITCRUSHER_H
