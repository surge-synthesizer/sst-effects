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

#ifndef INCLUDE_SST_VOICE_EFFECTS_MODULATION_RINGMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_MODULATION_RINGMOD_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct RingMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Ring Mod"};

    static constexpr int numFloatParams{1};
    static constexpr int numIntParams{0};

    enum FloatParams
    {
        fpCarrierFrequency,
    };

    RingMod() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~RingMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpCarrierFrequency:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withName("Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Frequency").withDefault(0);

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd().withName("Error");
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        auto pt = this->getFloatParam(fpCarrierFrequency) + (keytrackOn ? pitch : 0);

        qosc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pt) *
                     this->getSampleRateInv());
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            float modulator = qosc.v;
            dataoutL[i] = datainL[i] * modulator;
            dataoutR[i] = datainR[i] * modulator;

            qosc.step();
        }
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        auto pt = this->getFloatParam(fpCarrierFrequency) + (keytrackOn ? pitch : 0);

        qosc.setRate(440.0 * 2 * M_PI * this->note_to_pitch_ignoring_tuning(pt) *
                     this->getSampleRateInv());
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            float modulator = qosc.v;
            dataoutL[i] = datainL[i] * modulator;
            qosc.step();
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
    bool keytrackOn{false}, wasKeytrackOn{false};
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    sst::basic_blocks::dsp::QuadratureOscillator<float> qosc;
};

} // namespace sst::voice_effects::modulation
#endif // SHORTCIRCUITXT_CYTOMICSVF_H
