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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTERSPLUSPLUS_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTERSPLUSPLUS_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/filters++.h"
#include "sst/basic-blocks/dsp/PanLaws.h"
#include "sst/basic-blocks/simd/setup.h"

#include "../VoiceEffectCore.h"

#include <vector>
#include <cmath>
#include <cassert>

namespace sst::voice_effects::filter
{
template <typename VFXConfig, filtersplusplus::FilterModel Model>
struct FiltersPlusPlus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Filters++"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{5};

    static constexpr int lineSize{4108}; // MAX_FB_COMB + FIRIPOL_N
    static constexpr int bufferSize = lineSize * 4 * sizeof(float);

    enum FloatParams
    {
        fpCutoffL,
        fpCutoffR,
        fpResonance,
        fpExtra
    };

    enum IntParams
    {
        ipStereo,
        ipPassband,
        ipSlope,
        ipDrive,
        ipSubmodel
    };

    using fmd = filtersplusplus::FilterModel;
    using fpb = filtersplusplus::Passband;
    using fsl = filtersplusplus::Slope;
    using fdr = filtersplusplus::DriveMode;
    using fsm = filtersplusplus::FilterSubModel;

    FiltersPlusPlus() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(priorFP.begin(), priorFP.end(), -1000.f);
        std::fill(priorIP.begin(), priorIP.end(), -1);

        filter.init();
        filter.setFilterModel(Model);
        filter.setActive(2, false);
        filter.setActive(3, false);

        // For some models we omit or reorder options, or set a custom param name
        if constexpr (Model == fmd::VemberClassic)
        {
            passbands.emplace_back(fpb::LP);
            passbands.emplace_back(fpb::HP);
            passbands.emplace_back(fpb::BP);
            passbands.emplace_back(fpb::Notch);
            drives.emplace_back(fdr::Standard);
            drives.emplace_back(fdr::Driven);
            drives.emplace_back(fdr::NotchMild);
        }
        if constexpr (Model == fmd::K35)
        {
            drives.emplace_back(fdr::K35_Continuous);
            extraName = "Drive";
        }
        if constexpr (Model == fmd::OBXD_4Pole)
        {
            slopes.emplace_back(fsl::Slope_Morph);
            submodels.emplace_back(fsm::UNSUPPORTED); // else we get legacy 24dB and explode
            extraName = "Slope";
        }
        if constexpr (Model == fmd::VintageLadder)
        {
            submodels.emplace_back(fsm::RungeKuttaCompensated);
            submodels.emplace_back(fsm::HuovCompensated);
            submodels.emplace_back(fsm::RungeKutta);
            submodels.emplace_back(fsm::Huov);
            subName = "Method";
        }
        if constexpr (Model == fmd::CutoffWarp || Model == fmd::ResonanceWarp)
        {
            subName = "Stages";
        }
        if constexpr (Model == fmd::TriPole)
        {
            subName = "Output";
        }
        if constexpr (Model == fmd::CytomicSVF)
        {
            extraName = "Gain";
        }
        if constexpr (Model == fmd::Comb)
        {
            filter.setQuad();
            slopes.emplace_back(fsl::Comb_Bipolar_ContinuousMix);
            extraName = "+/- blend";

            this->preReservePool(bufferSize);
            this->enableKeytrack(true);
        }

        // For most of them just use everything. ReturnUnsupported must be true or we explode
        if (passbands.empty())
        {
            passbands = filtersplusplus::potentialValuesFor<fpb>(Model, true);
        }
        if (slopes.empty())
        {
            slopes = filtersplusplus::potentialValuesFor<fsl>(Model, true);
        }
        if (drives.empty())
        {
            drives = filtersplusplus::potentialValuesFor<fdr>(Model, true);
        }
        if (submodels.empty())
        {
            submodels = filtersplusplus::potentialValuesFor<fsm>(Model, true);
        }
    }

    ~FiltersPlusPlus()
    {
        if constexpr (Model == fmd::Comb)
        {
            if (buffer[0])
            {
                VFXConfig::returnBlock(this, (uint8_t *)buffer[0], bufferSize);
                for (int i = 0; i < 4; i++)
                {
                    buffer[i] = nullptr;
                }
            }
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;
        auto defres = 0.f;

        switch (idx)
        {
        case fpCutoffL:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName(std::string("Offset") + (stereo ? " L" : ""))
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(std::string("Cutoff") + (stereo ? " L" : ""))
                .withDefault(0);
        case fpCutoffR:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 96)
                    .withName(!stereo ? std::string() : "Offset R")
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones");
            }
            return pmd()
                .asAudibleFrequency()
                .withName(!stereo ? std::string() : std::string("Cutoff R"))
                .withDefault(0);
        case fpResonance:
            if constexpr (Model == fmd::Comb)
                defres = .95f;
            return pmd().asPercent().withName("Resonance").withDefault(defres);
        case fpExtra:
            if constexpr (Model == fmd::CytomicSVF)
            {
                return pmd().asDecibelWithRange(-24, 24).withName(extraName).withDefault(0.f);
            }
            if constexpr (Model == fmd::Comb)
            {
                return pmd()
                    .asPercentBipolar()
                    .withCustomMinDisplay("-")
                    .withCustomMaxDisplay("+")
                    .withName(extraName)
                    .withDefault(1.f);
            }
            return pmd().asPercent().withName(extraName).withDefault(0.f);
        }

        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        namespace fpp = filtersplusplus;
        using pmd = basic_blocks::params::ParamMetaData;

        std::unordered_map<int, std::string> pbm{}, slm{}, drm{}, smm{};
        int drsi{0};
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipPassband:
            if (passbands.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < passbands.size(); ++i)
            {
                pbm.insert({i, fpp::toString(passbands[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, passbands.size() - 1)
                .withUnorderedMapFormatting(pbm)
                .withName("Passband");
        case ipSlope:
            if (slopes.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < slopes.size(); ++i)
            {
                slm.insert({i, fpp::toString(slopes[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, slopes.size() - 1)
                .withUnorderedMapFormatting(slm)
                .withName("Slope");
        case ipDrive:
            if (drives.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");
            if constexpr (Model == fmd::VemberClassic)
            {
                drm.insert({0, fpp::toString(drives[0])});
                drm.insert({1, this->getIntParam(ipPassband) < 3 ? fpp::toString(drives[1])
                                                                 : fpp::toString(drives[2])});
                drsi = 1;
            }
            else
            {
                for (int i = 0; i < drives.size(); ++i)
                {
                    drm.insert({i, fpp::toString(drives[i])});
                }
                drsi = drives.size() - 1;
            }

            return pmd().asInt().withRange(0, drsi).withUnorderedMapFormatting(drm).withName(
                "Drive");
        case ipSubmodel:
            if (submodels.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < submodels.size(); ++i)
            {
                smm.insert({i, fpp::toString(submodels[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, submodels.size() - 1)
                .withUnorderedMapFormatting(smm)
                .withName(subName);
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        std::fill(priorFP.begin(), priorFP.end(), -1000.f);
        std::fill(priorIP.begin(), priorIP.end(), -1);

        if constexpr (Model == fmd::Comb)
        {
            plusLerp.instantize();
            minusLerp.instantize();
            if (!buffer[0])
            {
                assert(filter.requiredDelayLinesSizes(Model, configFilter()) <= lineSize);
                auto block = VFXConfig::checkoutBlock(this, bufferSize);
                memset(block, 0, bufferSize);
                for (int i = 0; i < 4; ++i)
                {
                    buffer[i] = (float *)(block + i * lineSize * sizeof(float));
                    filter.provideDelayLine(i, buffer[i]);
                }
            }
        }

        filter.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
        setupFilter();
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void setupFilter()
    {
        filter.setModelConfiguration(configFilter());
        if (!filter.prepareInstance())
        {
            std::cout << "something's wrong" << std::endl;
            filter.setFilterModel(fmd::CytomicSVF);
            filter.setPassband(fpb::LP);
            std::cout << " Invalid filter config, defaulting to " << filter.displayName()
                      << std::endl;
        }
        if constexpr (Model == fmd::CytomicSVF)
        {
            extraBounds[0] = -24.f;
            extraBounds[1] = 24.f;
        }
        else if (filter.coefficientsExtraIsBipolar(Model, filter.getModelConfiguration(), 0))
        {
            extraBounds[0] = -1.f;
            extraBounds[1] = 1.f;
        }
        else
        {
            extraBounds[0] = 0.f;
            extraBounds[1] = 1.f;
        }
    }

    template <bool mono, bool monoCoeff> void setCoeffs(float pitch)
    {
        std::array<float, numFloatParams> fp;
        std::array<int, numIntParams> ip;
        bool fDiff{false}, iDiff{false};

        for (int i = numFloatParams - 1; i >= 0; --i)
        {
            fp[i] = this->getFloatParam(i);
            if (i < 2 && keytrackOn)
            {
                fp[i] += pitch;
            }
            fDiff = fDiff || fp[i] != priorFP[i];
            priorFP[i] = fp[i];
        }

        for (int i = 0; i < numIntParams; ++i)
        {
            ip[i] = this->getIntParam(i);
            iDiff = iDiff || ip[i] != priorIP[i];
            priorIP[i] = ip[i];
        }

        if (iDiff)
        {
            setupFilter();
        }

        if (fDiff || iDiff)
        {
            auto reso = std::clamp(this->getFloatParam(fpResonance), 0.f, 1.f);
            auto extra = std::clamp(this->getFloatParam(fpExtra), extraBounds[0], extraBounds[1]);

            if constexpr (Model == fmd::CytomicSVF)
            {
                // Andy assumes A = pow(10, dB/40), our converter uses dB/20, hence the * .5f
                extra = this->dbToLinear(extra * 0.5f);
            }

            auto freqL = this->getFloatParam(fpCutoffL);
            if (keytrackOn)
                freqL += pitch;

            if constexpr (Model == fmd::Comb)
            {
                reso = std::pow(reso, .25);
                if constexpr (mono)
                {
                    // in comb we use voice 0 and 1 in mono to get neg/pos feedback in parallel
                    filter.setStereo();
                    filter.makeCoefficients(0, freqL, reso, -1.f);
                    filter.makeCoefficients(1, freqL, reso, 1.f);
                    return;
                }
                // And in stereo that's the left and 2 & 3 do the same on the right
                filter.setQuad();
                filter.makeCoefficients(0, freqL, reso, -1.f);
                filter.makeCoefficients(1, freqL, reso, 1.f);
                if constexpr (monoCoeff)
                {
                    filter.copyCoefficientsFromVoiceToVoice(0, 2);
                    filter.copyCoefficientsFromVoiceToVoice(1, 3);
                    return;
                }

                auto freqR = this->getFloatParam(fpCutoffR);
                if (keytrackOn)
                    freqR += pitch;
                filter.makeCoefficients(2, freqR, reso, -1.f);
                filter.makeCoefficients(3, freqR, reso, 1.f);
            }
            else
            {
                // all the others we only use voice 0 and 1
                if constexpr (mono)
                {
                    filter.setMono();
                    filter.makeCoefficients(0, freqL, reso, extra);
                    return;
                }

                filter.setStereo();
                filter.makeCoefficients(0, freqL, reso, extra);
                if constexpr (monoCoeff)
                {
                    filter.copyCoefficientsFromVoiceToVoice(0, 1);
                    return;
                }

                auto freqR = this->getFloatParam(fpCutoffR);
                if (keytrackOn)
                    freqR += pitch;
                filter.makeCoefficients(1, freqR, reso, extra);
            }
        }
        else
        {
            if constexpr (Model == fmd::Comb)
            {
                filter.freezeCoefficientsFor(0);
                filter.freezeCoefficientsFor(1);
                if constexpr (!mono)
                {
                    filter.freezeCoefficientsFor(2);
                    filter.freezeCoefficientsFor(3);
                }
            }
            else
            {
                filter.freezeCoefficientsFor(0);
                if constexpr (!mono)
                    filter.freezeCoefficientsFor(1);
            }
        }
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        setCoeffs<true, true>(pitch);

        filter.prepareBlock();
        if constexpr (Model == fmd::Comb)
        {
            auto b = (this->getFloatParam(fpExtra) + 1) * .5f;
            epBlend(std::clamp(b, 0.f, 1.f), blendMatrix);
            minusLerp.set_target(blendMatrix[0]);
            plusLerp.set_target(blendMatrix[1]);
            float minus alignas(16)[VFXConfig::blockSize];
            float plus alignas(16)[VFXConfig::blockSize];
            minusLerp.store_block(minus);
            plusLerp.store_block(plus);

            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                auto p{0.f}, m{0.f};
                filter.processStereoSample(datain[i], datain[i], m, p);
                dataout[i] = m * minus[i] + p * plus[i];
            }
        }
        else
        {
            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                dataout[i] = filter.processMonoSample(datain[i]);
            }
        }
        filter.concludeBlock();
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipStereo) > 0)
            setCoeffs<false, false>(pitch);
        else
            setCoeffs<false, true>(pitch);

        filter.prepareBlock();
        if constexpr (Model == fmd::Comb)
        {
            auto b = (this->getFloatParam(fpExtra) + 1) * .5f;
            epBlend(std::clamp(b, 0.f, 1.f), blendMatrix);
            minusLerp.set_target(blendMatrix[0]);
            plusLerp.set_target(blendMatrix[1]);
            float minus alignas(16)[VFXConfig::blockSize];
            float plus alignas(16)[VFXConfig::blockSize];
            minusLerp.store_block(minus);
            plusLerp.store_block(plus);

            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                auto in = SIMD_MM(set_ps)(datainL[i], datainL[i], datainR[i], datainR[i]);
                auto proc = filter.processSample(in);
                auto blm = SIMD_MM(set_ps)(plus[i], minus[i], plus[i], minus[i]);
                proc = SIMD_MM(mul_ps(proc, blm));
                float res alignas(16)[4];
                SIMD_MM(store_ps)(res, proc);
                dataoutL[i] = res[0] + res[1];
                dataoutR[i] = res[2] + res[3];
            }
        }
        else
        {
            for (int i = 0; i < VFXConfig::blockSize; ++i)
            {
                filter.processStereoSample(datainL[i], datainR[i], dataoutL[i], dataoutR[i]);
            }
        }
        filter.concludeBlock();
    }

    void processMonoToStereo(const float *const datain, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datain, datain, dataoutL, dataoutR, pitch);
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool checkParameterConsistency() const { return true; }

  protected:
    bool keytrackOn{false};
    std::array<float, numFloatParams> priorFP;
    std::array<int, numIntParams> priorIP;
    float *buffer[4]{nullptr, nullptr, nullptr, nullptr};
    float extraBounds[2]{0.f, 1.f};

    basic_blocks::dsp::pan_laws::panmatrix_t blendMatrix;
    void epBlend(float blend, basic_blocks::dsp::pan_laws::panmatrix_t &res)
    {
        basic_blocks::dsp::pan_laws::stereoEqualPower(blend, res);
    }
    basic_blocks::dsp::lipol_sse<VFXConfig::blockSize> plusLerp, minusLerp;

    filtersplusplus::Filter filter = filtersplusplus::Filter();
    std::string extraName{""}, subName{""};

    std::vector<filtersplusplus::Passband> passbands;
    std::vector<filtersplusplus::Slope> slopes;
    std::vector<filtersplusplus::DriveMode> drives;
    std::vector<filtersplusplus::FilterSubModel> submodels;

    filtersplusplus::ModelConfig configFilter()
    {
        namespace fpp = filtersplusplus;
        fpp::ModelConfig cfg{};
        uint32_t pid{0}, sid{0}, did{0}, mid{0};
        int ps = passbands.size();
        int ss = slopes.size();
        int ds = drives.size();
        int ms = submodels.size();

        if (ps > 1)
        {
            pid = std::clamp(this->getIntParam(ipPassband), 0, ps - 1);
        }
        cfg.pt = passbands[pid];

        if (ss > 1)
        {
            sid = std::clamp(this->getIntParam(ipSlope), 0, ss - 1);
        }
        cfg.st = slopes[sid];

        if (ds > 1)
        {
            did = std::clamp(this->getIntParam(ipDrive), 0, ds - 1);
        }
        if constexpr (Model == fpp::FilterModel::VemberClassic)
        {
            // in vember classic notch the second drive mode is different
            if (pid == 3 && did == 1)
                did += 1;
        }
        cfg.dt = drives[did];

        if (ms > 1)
        {
            mid = std::clamp(this->getIntParam(ipSubmodel), 0, ms - 1);
        }
        cfg.mt = submodels[mid];

        return cfg;
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

} // namespace sst::voice_effects::filter
#endif // FILTERSPLUSPLUS_H
