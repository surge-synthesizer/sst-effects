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

    using PmSineTable = sst::basic_blocks::tables::SixSinesWaveProvider;
    PmSineTable &sPmSineTable;
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

    ThreeOpPhaseMod(const twoToTheXTable &tt, PmSineTable &pst)
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
            return pmd()
                .asFloat()
                .withRange(-5, 5)
                .withATwoToTheBFormatting(1, 1, "x")
                .withName("Op 2 Ratio")
                .withDecimalPlaces(4)
                .withDefault(0.0)
                .withFeature(pmd::Features::BELOW_ONE_IS_INVERSE_FRACTION)
                .withFeature(pmd::Features::ALLOW_FRACTIONAL_TYPEINS);
        case fpRatio3:
            return pmd()
                .asFloat()
                .withRange(-5, 5)
                .withATwoToTheBFormatting(1, 1, "x")
                .withName("Op 3 Ratio")
                .withDecimalPlaces(4)
                .withDefault(0.0)
                .withFeature(pmd::Features::BELOW_ONE_IS_INVERSE_FRACTION)
                .withFeature(pmd::Features::ALLOW_FRACTIONAL_TYPEINS);
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

    void initVoiceEffect() {}
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        float sumIn alignas(16)[VFXConfig::blockSize];
        mech::add_block<VFXConfig::blockSize>(datainL, datainR, sumIn);

        float sumOut alignas(16)[VFXConfig::blockSize];
        processMonoToMono(sumIn, sumOut, pitch);

        mech::copy_from_to<VFXConfig::blockSize>(sumOut, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(sumOut, dataoutR);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;

        float freq2 alignas(16)[VFXConfig::blockSize];
        freq2Lerp.set_target(440 * this->note_to_pitch_ignoring_tuning(pitch) *
                             sTwoToTheXTable.twoToThe(this->getFloatParam(fpRatio2)));
        freq2Lerp.store_block(freq2);

        float freq3 alignas(16)[VFXConfig::blockSize];
        freq3Lerp.set_target(440 * this->note_to_pitch_ignoring_tuning(pitch) *
                             sTwoToTheXTable.twoToThe(this->getFloatParam(fpRatio3)));
        freq3Lerp.store_block(freq3);

        float fbk2 alignas(16)[VFXConfig::blockSize];
        fbk2Lerp.set_target(this->getFloatParam(fpFeedback2));
        fbk2Lerp.store_block(fbk2);

        float fbk3 alignas(16)[VFXConfig::blockSize];
        fbk3Lerp.set_target(this->getFloatParam(fpFeedback3));
        fbk3Lerp.store_block(fbk3);

        float d12 alignas(16)[VFXConfig::blockSize];
        depth12Lerp.set_target(this->getFloatParam(fp1to2));
        depth12Lerp.store_block(d12);

        float d13 alignas(16)[VFXConfig::blockSize];
        depth13Lerp.set_target(this->getFloatParam(fp1to3));
        depth13Lerp.store_block(d13);

        float d23 alignas(16)[VFXConfig::blockSize];
        depth23Lerp.set_target(this->getFloatParam(fp2to3));
        depth23Lerp.store_block(d23);

        int32_t pm2 alignas(16)[VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm2[i] = (int32_t)((1 << 27) * (datain[i] * d12[i]));
        }

        float out2 alignas(16)[VFXConfig::blockSize];
        innerLoop(out2, fbVal2, fbk2, freq2, pm2, phase2, dPhase2);

        int32_t pm3 alignas(16)[VFXConfig::blockSize];
        for (int i = 0; i < VFXConfig::blockSize; i++)
        {
            pm3[i] = (int32_t)((1 << 27) * (datain[i] * d13[i]));
            pm3[i] += (int32_t)((1 << 27) * (out2[i] * d23[i]));
        }

        float out3 alignas(16)[VFXConfig::blockSize];
        innerLoop(out3, fbVal3, fbk3, freq3, pm3, phase3, dPhase3);

        level2Lerp.set_target(this->getFloatParam(fpLevel2));
        level3Lerp.set_target(this->getFloatParam(fpLevel3));
        level2Lerp.multiply_block(out2);
        level3Lerp.multiply_block(out3);

        mech::add_block<VFXConfig::blockSize>(out3, out2);
        mech::copy_from_to<VFXConfig::blockSize>(out3, dataout);
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

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> freq2Lerp, fbk2Lerp, freq3Lerp,
        fbk3Lerp, depth12Lerp, depth13Lerp, depth23Lerp, level2Lerp, level3Lerp;

    uint32_t phase2;
    int32_t dPhase2;
    int32_t phaseInput2;
    float fbVal2[2]{0.f, 0.f};

    uint32_t phase3;
    int32_t dPhase3;
    int32_t phaseInput3;
    float fbVal3[2]{0.f, 0.f};

    void innerLoop(float *onto, float *fbv, float const *fbl, float const *freq, int32_t *phsIn,
                   uint32_t &phs, int32_t &dphs)
    {
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            dphs = sPmSineTable.dPhase(freq[i]);
            phs += dphs;

            auto fb = 0.5 * (fbv[0] + fbv[1]);
            auto sb = std::signbit(fbl[i]);
            fb = fb * (1 - sb * (1 - fb));

            auto ph = phs + phsIn[i] + static_cast<int32_t>((1 << 24) * (fbl[i] * fb));
            auto out = sPmSineTable.at(ph);

            onto[i] = out;
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
