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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_SURGEBIQUADS_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_SURGEBIQUADS_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::filter
{
template <typename VFXConfig> struct SurgeBiquads : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Surge Biquads"};

    static constexpr int numFloatParams{2};
    static constexpr int numIntParams{1};

    SurgeBiquads() : core::VoiceEffectTemplateBase<VFXConfig>(), bq(this)
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -1);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
        bq.suspend();
    }

    ~SurgeBiquads() {}

    enum Modes
    {
        LP,
        LP2B,
        HP,
        BP,
        BP2A,
        PKA,
        NOTCH,
        ALL
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case 0:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName("Offset")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd().asAudibleFrequency().withName("Cutoff").withDefault(0);

        case 1:
            return pmd()
                .asPercent()
                .withDefault(0.7f)
                .withName("Resonance")
                .withLinearScaleFormatting("");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        using md = sst::filters::CytomicSVF::Mode;

        return pmd()
            .asInt()
            .withRange(0, 7)
            .withName("Mode")
            .withUnorderedMapFormatting({

                {Modes::LP, "Low Pass"},
                {Modes::LP2B, "Low Pass 2B"},
                {Modes::HP, "High Pass"},
                {Modes::BP, "Band Pass"},
                {Modes::BP2A, "Band Pass 2A"},
                {Modes::PKA, "PKA"},
                {Modes::NOTCH, "Notch"},
                {Modes::ALL, "All Pass"}})
            .withDefault(Modes::LP2B);
    }

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        calc_coeffs(pitch);
        bq.process_block_to(datainL, datainR, dataoutL, dataoutR);
    }

    void processMonoToMono(float *datainL, float *dataoutL, float pitch)
    {
        calc_coeffs(pitch);
        bq.process_block_to(datainL, dataoutL);
    }

    void calc_coeffs(float pitch = 0.f)
    {
        std::array<float, numFloatParams> param;
        std::array<int, numIntParams> iparam;
        bool diff{false}, idiff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            if (i == 0 && keytrackOn)
            {
                param[i] += pitch;
            }
            diff = diff || (mLastParam[i] != param[i]);
        }
        for (int i = 0; i < numIntParams; ++i)
        {
            iparam[i] = this->getIntParam(i);
            idiff = idiff || (mLastIParam[i] != iparam[i]);
        }
        idiff |= (wasKeytrackOn != keytrackOn);
        wasKeytrackOn = keytrackOn;

        if (diff || idiff)
        {
            if (idiff)
            {
                bq.suspend();
            }

            auto Q = std::clamp(param[1], 0.001f, 0.999f);
            auto omega = bq.calc_omega(param[0] / 12.f);
            switch ((Modes)iparam[0])
            {
            case LP:
                bq.coeff_LP(omega, Q);
                break;
            case LP2B:
                bq.coeff_LP2B(omega, Q);
                break;
            case HP:
                bq.coeff_HP(omega, Q);
                break;
            case BP:
                bq.coeff_BP(omega, Q);
                break;
            case BP2A:
                bq.coeff_BP2A(omega, Q);
                break;
            case PKA:
                bq.coeff_PKA(omega, Q);
                break;
            case NOTCH:
                bq.coeff_NOTCH(omega, Q);
                break;
            case ALL:
                bq.coeff_APF(omega, Q);
                break;
            }

            mLastParam = param;
            mLastIParam = iparam;
        }
    }

    float getFrequencyGraph(float f) { return bq.plot_magnitude(f); }

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

    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType bq;

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

} // namespace sst::voice_effects::filter
#endif // SHORTCIRCUITXT_CYTOMICSVF_H
