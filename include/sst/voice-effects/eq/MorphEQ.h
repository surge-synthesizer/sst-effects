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

#ifndef INCLUDE_SST_VOICE_EFFECTS_EQ_MORPHEQ_H
#define INCLUDE_SST_VOICE_EFFECTS_EQ_MORPHEQ_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

#include "../VoiceEffectCore.h"

#include <iostream>
#include <array>

namespace sst::voice_effects::eq
{
template <typename VFXConfig> struct MorphEQ : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr int numFilters{8};

    struct BandDesc
    {
        bool active{false};
        float freq{-10000.f};
        float gain{0.f};
        float BW{1.f};

        constexpr BandDesc() {}
        constexpr BandDesc(float f, float g, float b) : active(true), freq(f), gain(g), BW(b) {}
        ~BandDesc() = default;
    };
    struct Snapshot
    {
        char name[64]{};
        float gain{};
        std::array<BandDesc, 8> bands{};
        ~Snapshot() = default;
        constexpr Snapshot() {}
        constexpr Snapshot(const char *n, float g, std::array<BandDesc, 8> b)
        {
            for (int i = 0; i < 64; ++i)
            {
                name[i] = n[i];
                if (n[i] == 0)
                    break;
            }
            gain = g;
            for (int i = 0; i < 8; ++i)
            {
                bands[i] = b[i];
                if (bands[i].freq < -100)
                {
                    switch (i)
                    {
                    case 0:
                        bands[i].freq = -8;
                        break;
                    case 1:
                        bands[i].freq = -2.5;
                        break;
                    case 2:
                        bands[i].freq = -1;
                        break;
                    case 3:
                        bands[i].freq = -0;
                        break;
                    case 4:
                        bands[i].freq = 1;
                        break;
                    case 5:
                        bands[i].freq = 2.5;
                        break;
                    case 6:
                        bands[i].freq = 4.0;
                        break;
                    case 7:
                        bands[i].freq = 8.0;
                        break;
                    default:
                        bands[i].freq = 0;
                        break;
                    }
                }
            }
        }
    };

    static constexpr int numPresets{13};
    static constexpr std::array<Snapshot, numPresets> snapshots{
        // clang-format off
        Snapshot{"Flat", 0.f, {}},
        Snapshot{"Tone LP 1", -6.f,
                    {BandDesc(),
                     {-2.f, 24.f, .1f},
                     {-1.f, 24.f, .1f},
                     {0.f, 24.f, .1f},
                     {1.f, 24.f, .1f},
                     {2.f, 24.f, .1f},
                     {},
                     {0.f, 0.f, 1.f}}},

        Snapshot{"Tone LP 2", -6.f, {
               BandDesc(),
               {-0.2f, 24.f, .01f},
               {-0.2f, 24.f, .01f},
               {0.f, 24.f, .01f},
               {0.2f, 24.f, .01f},
               {0.2f, 24.f, .01f},
               {},
               {0.f, 0.f, 1.f},
           }},

        Snapshot{"Bell labs", -40.f, {
                   BandDesc{-6.f, 0.f, 1.f},
                   {-1.f, 50.f, 0.02f}, // 1
                   {1.f, 40.f, 0.04f}, // 2
                   {1.58f, 40.f, 0.06f}, // 3
                   {2.07f, 40.f, 0.07f}, // 4
                   {2.43f, 40.f, 0.08f}, // 5
                   {2.77f, 40.f, 0.1f}, // 6
                   {6.f, 0.f, 1.f}
           }},
        Snapshot{"Combs", 0.f, {
                   BandDesc(),
                   {0.f, -80.f, 0.01f}, // 1
                   {1.f, -80.f, 0.01f}, // 2
                   {1.58f, -80.f, 0.01f}, // 3
                   {2.f, -80.f, 0.01f}, // 4
                   {2.32f, -80.f, 0.01f}, // 5
                   {2.58f, -80.f, 0.01f}, // 6
                   {}
           }},
       Snapshot{"Random Dips", 0.f, {
                   BandDesc(),
                   {0.f, -80.f, 0.01f}, // 1
                   {0.7f, -80.f, 0.01f}, // 2
                   {1.6f, -80.f, 0.01f}, // 3
                   {2.3f, -80.f, 0.01f}, // 4
                   {3.1f, -80.f, 0.01f}, // 5
                   {4.54f, -80.f, 0.01f}, // 6
                   {}
           }},
        Snapshot{"Ups and Downs", 0.f, {
                   BandDesc(),
                   {0.f, -80.f, 0.01f}, // 1
                   {1.f, 12.f, 0.1f}, // 2
                   {1.58f, -80.f, 0.01f}, // 3
                   {2.f, 12.f, 0.1f}, // 4
                   {2.32f, -80.f, 0.01f}, // 5
                   {2.58f, 12.f, 0.1f}, // 6
                   {}
           }},
        Snapshot{"Formant A1", -40.f, {
                   BandDesc(),
                   {0.686f, 60.f, 1.6f}, // 1
                   {1.33f, 30.f, 0.13f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        Snapshot{"Formant A2", -40.f, {
                   BandDesc(),
                   {0.433f, 60.f, 1.8f}, // 1
                   {1.01f, 30.f, 0.15f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        Snapshot{"Formant E", -40.f, {
                   BandDesc(),
                   {0.35f, 60.f, 2.0f}, // 1
                   {2.05f, 30.f, 0.11f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        Snapshot{"Formant I", -40.f, {
                   BandDesc(),
                   {-0.65f, 60.f, 2.4f}, // 1
                   {2.357f, 30.f, 0.1f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        Snapshot{"Formant O", -40.f, {
                   BandDesc(),
                   {0.04f, 60.f, 2.0f}, // 1
                   {0.87f, 30.f, 0.3f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        Snapshot{"Formant U", -40.f, {
                   BandDesc(),
                   {-0.5f, 60.f, 2.3f}, // 1
                   {0.99f, 30.f, 0.4f}, // 2
                   {2.51f, 30.f, 0.07f}, // 3
                   {2.77f, 30.f, 0.05f}, // 4
           }},
        // clang-format on
    };
    static constexpr const char *effectName{"Morph EQ"};

    static constexpr int numFloatParams{5};
    static constexpr int numIntParams{2};

    MorphEQ() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mParametric.begin(), mParametric.end(), this);
        std::fill(mParametricC0.begin(), mParametricC0.end(), this);
        std::fill(mParametricC1.begin(), mParametricC1.end(), this);
    }

    ~MorphEQ() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case 0:
            return pmd().asPercent().withName("Morph").withDefault(0);
        case 1:
        case 2:
            return pmd()
                .asFloat()
                .withDefault(0.f)
                .withRange(-5, 5)
                .withName("Freq " + std::to_string(idx))
                .withLinearScaleFormatting("octaves")
                .withDefault(0);

        case 3:
            return pmd()
                .asFloat()
                .withName("Gain")
                .withDefault(0.f)
                .withRange(-24, 24)
                .withLinearScaleFormatting("dB");

        case 4:
            return pmd()
                .asFloat()
                .withRange(-1, 1)
                .withName("BW Offset")
                .withDefault(0.f)
                .withLinearScaleFormatting("octaves");

        default:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        std::unordered_map<int, std::string> names;
        for (int i = 0; i < snapshots.size(); ++i)
        {
            names[i] = snapshots[i].name;
        }

        return pmd()
            .asInt()
            .withRange(0, snapshots.size() - 1)
            .withName("Preset " + std::to_string(idx + 1))
            .withUnorderedMapFormatting(names)
            .withDefault(0);
    }

    void initVoiceEffect()
    {
        std::fill(mLastIParam.begin(), mLastIParam.end(), -13);
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        calc_coeffs();
        gain.multiply_2_blocks_to(datainL, datainR, dataoutL, dataoutR);

        float *inL = dataoutL;
        float *inR = dataoutR;
        for (int i = 0; i < numFilters; ++i)
        {
            if (mActive[i])
            {
                mParametric[i].process_block_to(inL, inR, dataoutL, dataoutR);
                inL = dataoutL;
                inR = dataoutR;
            }
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        calc_coeffs();

        gain.multiply_block_to(datainL, dataoutL);
        float *inL = dataoutL;

        for (int i = 0; i < numFilters; ++i)
        {
            if (mActive[i])
            {
                mParametric[i].process_block_to(inL, dataoutL);
                inL = dataoutL;
            }
        }
    }

    // includeInternal means the band graphic evaluators work but it takes some more CPU.
    // Use it in the UI, not the audio thread
    void calc_coeffs(bool includeInternal = false)
    {
        std::array<float, numFloatParams> param;
        std::array<int, numIntParams> iparam;
        bool diff{false}, idiff{false};
        for (int i = 0; i < numFloatParams; i++)
        {
            param[i] = this->getFloatParam(i);
            diff = diff || (mLastParam[i] != param[i]);
        }
        for (int i = 0; i < numIntParams; ++i)
        {
            iparam[i] = this->getIntParam(i);
            idiff = idiff || (mLastIParam[i] != iparam[i]);
        }

        if (diff || idiff || includeInternal)
        {
            auto &s0 = snapshots[std::clamp(iparam[0], 0, (int)snapshots.size() - 1)];
            auto &s1 = snapshots[std::clamp(iparam[1], 0, (int)snapshots.size() - 1)];

            if (idiff)
            {
                // snapshot changed so reset all the filters
                for (auto &filt : mParametric)
                {
                    filt.suspend();
                }
            }

            mAnyActive = false;
            for (int i = 0; i < numFilters; ++i)
            {
                mActive[i] = s0.bands[i].active || s1.bands[i].active;
                mAnyActive = mAnyActive || mActive[i];
            }

            float morph = std::clamp(param[0], 0.f, 1.f);
            float morph_m1 = 1 - morph;
            const float Q = 1 / sqrt(2.0);

            gainTarget = powf(10, 0.05 * (s0.gain * morph_m1 + s1.gain * morph));
            if (includeInternal)
            {
                gainC0 = powf(10, 0.05 * (s0.gain));
                gainC1 = powf(10, 0.05 * (s1.gain));
            }

            if (mActive[0])
            {
                mParametric[0].coeff_HP(
                    mParametric[0].calc_omega((s0.bands[0].freq + param[1]) * morph_m1 +
                                              (s1.bands[0].freq + param[2]) * morph),
                    Q);
                if (includeInternal)
                {
                    mParametricC0[0].coeff_HP(
                        mParametricC0[0].calc_omega(s0.bands[0].freq + param[1]), Q);
                    mParametricC0[0].coeff_instantize();
                    mParametricC1[0].coeff_HP(
                        mParametricC1[0].calc_omega(s1.bands[0].freq + param[2]), Q);
                    mParametricC1[0].coeff_instantize();
                }
            }
            if (mActive[7])
            {
                mParametric[7].coeff_LP2B(
                    mParametric[7].calc_omega((s0.bands[7].freq + param[1]) * morph_m1 +
                                              (s1.bands[7].freq + param[2]) * morph),
                    Q);
                if (includeInternal)
                {
                    mParametricC0[7].coeff_LP2B(
                        mParametricC0[7].calc_omega(s0.bands[7].freq + param[1]), Q);
                    mParametricC0[7].coeff_instantize();
                    mParametricC1[7].coeff_LP2B(
                        mParametricC1[7].calc_omega(s1.bands[7].freq + param[2]), Q);
                    mParametricC1[7].coeff_instantize();
                }
            }
            for (int i = 1; i < 7; i++)
            {
                if (mActive[i])
                {
                    mParametric[i].coeff_peakEQ(
                        mParametric[i].calc_omega((s0.bands[i].freq + param[1]) * morph_m1 +
                                                  (s1.bands[i].freq + param[2]) * morph),
                        std::max(s0.bands[i].BW * morph_m1 + s1.bands[i].BW * morph + param[4],
                                 0.001f),
                        s0.bands[i].gain * morph_m1 + s1.bands[i].gain * morph + param[3]);

                    if (includeInternal)
                    {
                        mParametricC0[i].coeff_peakEQ(
                            mParametricC0[i].calc_omega(s0.bands[i].freq + param[1]),
                            s0.bands[i].BW, s0.bands[i].gain);
                        mParametricC1[i].coeff_peakEQ(
                            mParametricC0[i].calc_omega(s1.bands[i].freq + param[2]),
                            s1.bands[i].BW, s1.bands[i].gain);
                        mParametricC0[i].coeff_instantize();
                        mParametricC1[i].coeff_instantize();
                    }
                }
            }

            mLastParam = param;
            mLastIParam = iparam;
        }

        gain.set_target(gainTarget);
    }

    float getFrequencyGraph(float f)
    {
        float y = 1;
        for (int k = 0; k < 8; k++)
            if (mActive[k])
                y *= mParametric[k].plot_magnitude(f);

        assert(gain.get_target() > 0);
        assert(y > 0);
        return y * gain.get_target();
    }

    // A bit of a hack - externalize this by calling calc_coeffs with isExternal to true first
    float getBandFrequencyGraph(int band, float f)
    {
        float y = 1;
        for (int k = 0; k < 8; k++)
        {
            if (mActive[k])
            {
                if (band == 0)
                {
                    y *= mParametricC0[k].plot_magnitude(f);
                }
                else
                {
                    y *= mParametricC1[k].plot_magnitude(f);
                }
                assert(y > 0);
            }
        }

        assert(gain.get_target() > 0);
        assert(gainC0 > 0);
        assert(gainC1 > 0);
        assert(y > 0);

        return y * (band == 0 ? gainC0 : gainC1);
    }
    size_t silentSamplesLength() const { return 10; }

  protected:
    std::array<float, numFloatParams> mLastParam{};
    std::array<int, numIntParams> mLastIParam{};
    std::array<typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType, numFilters>
        mParametric;

    std::array<typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType, numFilters>
        mParametricC0, mParametricC1;

    std::array<bool, numFilters> mActive{};
    bool mAnyActive{false};
    float gainTarget{1.f};
    float gainC0{1.f}, gainC1{1.f};
    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, false> gain;

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

#endif // SHORTCIRCUITXT_EqNBandParametric_H
