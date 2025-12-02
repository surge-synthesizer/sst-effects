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

#ifndef INCLUDE_SST_VOICE_EFFECTS_CATEGORY_THREEOPPHASEMOD_H
#define INCLUDE_SST_VOICE_EFFECTS_CATEGORY_THREEOPPHASEMOD_H

#include "sst/basic-blocks/tables/TwoToTheXProvider.h"
#include "sst/basic-blocks/tables/SixSinesWaveProvider.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "../VoiceEffectCore.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig> struct ThreeOpPhaseMod : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"3op Phase Mod"};

    static constexpr int numFloatParams{9};
    static constexpr int numIntParams{0};

    using ssSineTable = sst::basic_blocks::tables::SixSinesWaveProvider;
    ssSineTable &sPmSineTable;
    using twoToTheXTable = sst::basic_blocks::tables::TwoToTheXProvider;
    const twoToTheXTable &sTwoToTheXTable;

    enum FloatParams
    {
        fpRatio2,
        fpRatio3,
        fpFeedback2,
        fpFeedback3,
        fp1to2,
        fp1to3,
        fp2to3,
        fpLevel2,
        fpLevel3,
    };

    ThreeOpPhaseMod(const twoToTheXTable &tt, ssSineTable &pst)
        : core::VoiceEffectTemplateBase<VFXConfig>(), sPmSineTable(pst), sTwoToTheXTable(tt)
    {
    }

    ~ThreeOpPhaseMod() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpRatio2:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-5, 5)
                    .withATwoToTheBFormatting(1, 1, "x")
                    .withName("Op 2 Ratio")
                    .withDecimalPlaces(4)
                    .withDefault(0.0)
                    .withFeature(pmd::Features::BELOW_ONE_IS_INVERSE_FRACTION)
                    .withFeature(pmd::Features::ALLOW_FRACTIONAL_TYPEINS);
            }
            return pmd().asAudibleFrequency().withName("Op 2 Freq");
        case fpRatio3:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-5, 5)
                    .withATwoToTheBFormatting(1, 1, "x")
                    .withName("Op 3 Ratio")
                    .withDecimalPlaces(4)
                    .withDefault(0.0)
                    .withFeature(pmd::Features::BELOW_ONE_IS_INVERSE_FRACTION)
                    .withFeature(pmd::Features::ALLOW_FRACTIONAL_TYPEINS);
            }
            return pmd().asAudibleFrequency().withName("Op 3 Freq");
        case fpFeedback2:
            return pmd().asFloat().asPercentBipolar().withName("Op 2 Feedback");
        case fpFeedback3:
            return pmd().asFloat().asPercentBipolar().withName("Op 3 Feedback");
        case fp1to2:
            return pmd().asFloat().asPercent().withDefault(0.f).withName("1->2 Depth");
        case fp1to3:
            return pmd().asFloat().asPercent().withDefault(0.f).withName("1->3 Depth");
        case fp2to3:
            return pmd().asFloat().asPercent().withDefault(0.f).withName("2->3 Depth");
        case fpLevel2:
            return pmd().asFloat().asPercent().withDefault(0.f).withName("Op 2 Level");
        case fpLevel3:
            return pmd().asFloat().asPercent().withDefault(.75f).withName("Op 3 Level");
        };

        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        namespace mech = sst::basic_blocks::mechanics;

        mech::clear_block<VFXConfig::blockSize>(freq2);
        mech::clear_block<VFXConfig::blockSize>(freq3);
        mech::clear_block<VFXConfig::blockSize>(fbk2);
        mech::clear_block<VFXConfig::blockSize>(fbk3);
        mech::clear_block<VFXConfig::blockSize>(d12);
        mech::clear_block<VFXConfig::blockSize>(d13);
        mech::clear_block<VFXConfig::blockSize>(d23);
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        setupForBlock(pitch);

        int32_t pm2 alignas(16)[2][VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm2[0][i] = (int32_t)((1 << 27) * (datainL[i] * d12[i]));
            pm2[1][i] = (int32_t)((1 << 27) * (datainR[i] * d12[i]));
        }

        float out2 alignas(16)[2][VFXConfig::blockSize];
        for (int c = 0; c < 2; c++)
        {
            innerLoop(out2[c], fbVal2[c], fbk2, freq2, phase2[c], pm2[c]);
        }

        int32_t pm3 alignas(16)[2][VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm3[0][i] = (int32_t)((1 << 27) * (datainL[i] * d13[i]));
            pm3[0][i] += (int32_t)((1 << 27) * (out2[0][i] * d23[i]));

            pm3[1][i] = (int32_t)((1 << 27) * (datainR[i] * d13[i]));
            pm3[1][i] += (int32_t)((1 << 27) * (out2[1][i] * d23[i]));
        }

        float out3 alignas(16)[2][VFXConfig::blockSize];
        for (int c = 0; c < 2; c++)
        {
            innerLoop(out3[c], fbVal3[c], fbk3, freq3, phase3[c], pm3[c]);
        }

        level2Lerp.multiply_2_blocks(out2[0], out2[1]);
        level3Lerp.multiply_2_blocks(out3[0], out3[1]);

        mech::clear_block<VFXConfig::blockSize>(dataoutL);
        mech::clear_block<VFXConfig::blockSize>(dataoutR);
        mech::scale_accumulate_from_to<VFXConfig::blockSize>(out2[0], out2[1], .5f, dataoutL,
                                                             dataoutR);
        mech::scale_accumulate_from_to<VFXConfig::blockSize>(out3[0], out3[1], .5f, dataoutL,
                                                             dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        setupForBlock(pitch);

        int32_t pm2 alignas(16)[VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm2[i] = (int32_t)((1 << 27) * (datain[i] * d12[i]));
        }

        float out2 alignas(16)[VFXConfig::blockSize];
        innerLoop(out2, fbVal2[0], fbk2, freq2, phase2[0], pm2);

        int32_t pm3 alignas(16)[VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm3[i] = (int32_t)((1 << 27) * (datain[i] * d13[i]));
            pm3[i] += (int32_t)((1 << 27) * (out2[i] * d23[i]));
        }

        float out3 alignas(16)[VFXConfig::blockSize];
        innerLoop(out3, fbVal3[0], fbk3, freq3, phase3[0], pm3);

        level2Lerp.multiply_block(out2);
        level3Lerp.multiply_block(out3);

        mech::clear_block<VFXConfig::blockSize>(dataout);
        mech::scale_accumulate_from_to<VFXConfig::blockSize>(out2, .5f, dataout);
        mech::scale_accumulate_from_to<VFXConfig::blockSize>(out3, .5f, dataout);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }

    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{false}, wasKeytrackOn{false}, first{true};

    uint32_t phase2[2]{0, 0};
    float fbVal2[2][2]{{0.f, 0.f}, {0.f, 0.f}};
    uint32_t phase3[2]{0, 0};
    float fbVal3[2][2]{{0.f, 0.f}, {0.f, 0.f}};

    float freq2 alignas(16)[VFXConfig::blockSize];
    float freq3 alignas(16)[VFXConfig::blockSize];
    float fbk2 alignas(16)[VFXConfig::blockSize];
    float fbk3 alignas(16)[VFXConfig::blockSize];
    float d12 alignas(16)[VFXConfig::blockSize];
    float d13 alignas(16)[VFXConfig::blockSize];
    float d23 alignas(16)[VFXConfig::blockSize];

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> freq2Lerp, fbk2Lerp, freq3Lerp,
        fbk3Lerp, depth12Lerp, depth13Lerp, depth23Lerp, level2Lerp, level3Lerp;

    void setupForBlock(float pitch)
    {
        auto fp2{220.f}, fp3{220.f};

        if (keytrackOn)
        {
            fp2 *= this->note_to_pitch_ignoring_tuning(pitch) *
                   sTwoToTheXTable.twoToThe(this->getFloatParam(fpRatio2));
            fp3 *= this->note_to_pitch_ignoring_tuning(pitch) *
                   sTwoToTheXTable.twoToThe(this->getFloatParam(fpRatio3));
        }
        else
        {
            fp2 *= this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpRatio2));
            fp3 *= this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpRatio3));
        }

        freq2Lerp.set_target(fp2);
        freq3Lerp.set_target(fp3);
        fbk2Lerp.set_target(this->getFloatParam(fpFeedback2));
        fbk3Lerp.set_target(this->getFloatParam(fpFeedback3));
        depth12Lerp.set_target(this->getFloatParam(fp1to2));
        depth13Lerp.set_target(this->getFloatParam(fp1to3));
        depth23Lerp.set_target(this->getFloatParam(fp2to3));
        level2Lerp.set_target(this->getFloatParam(fpLevel2));
        level3Lerp.set_target(this->getFloatParam(fpLevel3));

        freq2Lerp.store_block(freq2);
        freq3Lerp.store_block(freq3);
        fbk2Lerp.store_block(fbk2);
        fbk3Lerp.store_block(fbk3);
        depth12Lerp.store_block(d12);
        depth13Lerp.store_block(d13);
        depth23Lerp.store_block(d23);
    }

    void innerLoop(float *output, float *fbv, float const *fbl, float const *freq, uint32_t &phs,
                   int32_t *pmv)
    {
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto dphs = sPmSineTable.dPhase(freq[i]);
            phs += dphs;

            auto fb = 0.5 * (fbv[0] + fbv[1]);
            auto sb = std::signbit(fbl[i]);
            fb = fb * (1 - sb * (1 - fb));

            auto ph = phs + pmv[i] + static_cast<int32_t>((1 << 24) * (fbl[i] * fb));
            auto out = sPmSineTable.at(ph);

            output[i] = out;
            fbv[1] = fbv[0];
            fbv[0] = out;
        }
    }

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

} // namespace sst::voice_effects::generator
#endif
