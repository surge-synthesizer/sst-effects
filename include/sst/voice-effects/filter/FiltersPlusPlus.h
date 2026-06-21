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

#ifndef INCLUDE_SST_VOICE_EFFECTS_FILTER_FILTERSPLUSPLUS_H
#define INCLUDE_SST_VOICE_EFFECTS_FILTER_FILTERSPLUSPLUS_H

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
// The option lists a Filters++ model exposes (passbands, slopes, drives, submodels)
// are a pure function of the compile-time Model. Building them queries the filters++
// configuration API, which allocates - so we resolve them once into a shared static via
// warmUp() (called off the audio thread at processor registration) rather than in the
// FiltersPlusPlus constructor, which is spawned on the audio thread.
template <filtersplusplus::FilterModel Model> struct FilterModelConfiguration
{
    using fmd = filtersplusplus::FilterModel;
    using fpb = filtersplusplus::Passband;
    using fsl = filtersplusplus::Slope;
    using fdr = filtersplusplus::DriveMode;
    using fsm = filtersplusplus::FilterSubModel;

    using passbandType =
        std::conditional_t<Model == fmd::OBXD_Xpander, std::vector<filtersplusplus::ModelConfig>,
                           std::vector<filtersplusplus::Passband>>;

    passbandType passbands;
    std::vector<fsl> slopes;
    std::vector<fdr> drives;
    std::vector<fsm> submodels;
    const char *extraName{""};
    const char *subName{""};

    static const FilterModelConfiguration &get()
    {
        static const FilterModelConfiguration instance = build();
        return instance;
    }

    // Force the static to populate. Call off the audio thread (processor warmup).
    static void warmUp()
    {
        sst::filters::detail::BasicTuningProvider btp;
        [[maybe_unused]] auto one = btp.twoToThe(0);
        (void)get();
    }

  private:
    static FilterModelConfiguration build()
    {
        namespace fpp = filtersplusplus;
        FilterModelConfiguration c;

        // For some models we omit or reorder options, or set a custom param name
        if constexpr (Model == fmd::VemberClassic)
        {
            c.passbands.emplace_back(fpb::LP);
            c.passbands.emplace_back(fpb::HP);
            c.passbands.emplace_back(fpb::BP);
            c.passbands.emplace_back(fpb::Notch);
            c.drives.emplace_back(fdr::Standard);
            c.drives.emplace_back(fdr::Driven);
            c.drives.emplace_back(fdr::NotchMild);
        }
        if constexpr (Model == fmd::K35)
        {
            c.drives.emplace_back(fdr::K35_Continuous);
            c.extraName = "Drive";
        }
        if constexpr (Model == fmd::OBXD_4Pole)
        {
            c.slopes.emplace_back(fsl::Slope_Morph);
            c.submodels.emplace_back(fsm::UNSUPPORTED); // else we get legacy 24dB and explode
            c.extraName = "Slope";
        }
        if constexpr (Model == fmd::VintageLadder)
        {
            c.submodels.emplace_back(fsm::RungeKuttaCompensated);
            c.submodels.emplace_back(fsm::HuovCompensated);
            c.submodels.emplace_back(fsm::RungeKutta);
            c.submodels.emplace_back(fsm::Huov);
            c.subName = "Method";
        }
        if constexpr (Model == fmd::CutoffWarp || Model == fmd::ResonanceWarp)
        {
            c.subName = "Stages";
        }
        if constexpr (Model == fmd::TriPole)
        {
            c.subName = "Output";
        }
        if constexpr (Model == fmd::CytomicSVF)
        {
            c.extraName = "Gain";
        }
        if constexpr (Model == fmd::Comb)
        {
            c.slopes.emplace_back(fsl::Comb_Bipolar_ContinuousMix);
            c.extraName = "+/- blend";
        }

        // in this one case it's better UI/UX wise to have a single list
        if constexpr (Model == fmd::OBXD_Xpander)
        {
            c.passbands = fpp::Filter::availableModelConfigurations(Model, true);
        }

        // For most of them just use everything. ReturnUnsupported must be true or we explode
        if (c.passbands.empty())
        {
            if constexpr (Model != fmd::OBXD_Xpander)
                c.passbands = fpp::potentialValuesFor<fpb>(Model, true);
        }
        if (c.slopes.empty())
        {
            if constexpr (Model != fmd::OBXD_Xpander)
                c.slopes = fpp::potentialValuesFor<fsl>(Model, true);
        }
        if (c.drives.empty())
        {
            c.drives = fpp::potentialValuesFor<fdr>(Model, true);
        }
        if (c.submodels.empty())
        {
            c.submodels = fpp::potentialValuesFor<fsm>(Model, true);
        }

        return c;
    }
};

// to add a filter model from the filtersplusplus API:
// - add a streaming name for it in the switch
// - add it to the section in /tests/create-voice-effect
// - if desired, decide in FilterModelConfiguration<Model>::build() which configs to
// expose by emplace_back()'ing the options you want into the right vector
// - it should "just work"
template <typename VFXConfig, filtersplusplus::FilterModel Model>
struct FiltersPlusPlus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr auto nameFn()
    {
        constexpr size_t maxFN{40};
        constexpr auto modelStr = sst::filtersplusplus::toCharPtr(Model);
        constexpr size_t totalLen = 10 + maxFN + 1; // "Filters++ " + model + null

        std::array<char, totalLen> result{};
        std::fill(result.begin(), result.end(), 0);

        // Copy "Filters++ " (10 characters)
        result[0] = 'F';
        result[1] = 'i';
        result[2] = 'l';
        result[3] = 't';
        result[4] = 'e';
        result[5] = 'r';
        result[6] = 's';
        result[7] = '+';
        result[8] = '+';
        result[9] = ' ';

        // Copy model name
        for (size_t i = 0; (i < maxFN) && (modelStr[i] != 0); ++i)
            result[10 + i] = modelStr[i];

        result[totalLen - 1] = '\0';
        return result;
    }
    static constexpr auto ens = nameFn();
    static constexpr auto displayName = ens.data();

    static constexpr const char *streamingNameByModel()
    {
        switch (Model)
        {
        case filtersplusplus::FilterModel::CytomicSVF:
            return "filt-cytomic";
        case filtersplusplus::FilterModel::VemberClassic:
            return "vemberclassic";
        case filtersplusplus::FilterModel::DiodeLadder:
            return "diodeladder";
        case filtersplusplus::FilterModel::TriPole:
            return "tripole";
        case filtersplusplus::FilterModel::OBXD_4Pole:
            return "obxd-4pole";
        case filtersplusplus::FilterModel::OBXD_Xpander:
            return "obxd-Xpander";
        case filtersplusplus::FilterModel::CutoffWarp:
            return "cutoffwarp";
        case filtersplusplus::FilterModel::ResonanceWarp:
            return "reswarp";
        case filtersplusplus::FilterModel::SampleAndHold:
            return "SnH";
        case filtersplusplus::FilterModel::Comb:
            return "Comb";
        case filtersplusplus::FilterModel::K35:
            return "k35";
        case filtersplusplus::FilterModel::VintageLadder:
            return "vintageladder";
        default:
            break;
        }
        return "-error-";
    }
    // If you hit this assert you've created on a model not in above switch
    static_assert(streamingNameByModel()[0] != '-');
    static constexpr auto streamingName{streamingNameByModel()};

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

        // The option lists live in FilterModelConfiguration<Model> (warmed up off the
        // audio thread). Only instance-level setup belongs here since this constructor
        // is spawned on the audio thread.
        if constexpr (Model == fmd::Comb)
        {
            filter.setQuad();
            this->preReservePool(bufferSize);
            this->enableKeytrack(true);
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

    static const FilterModelConfiguration<Model> &config()
    {
        return FilterModelConfiguration<Model>::get();
    }

    // Build the shared per-model configuration cache. Called off the audio thread at
    // processor registration so the audio-thread constructor never builds it.
    static void warmUpCache() { FilterModelConfiguration<Model>::warmUp(); }

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
                    .withSemitoneFormatting();
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
                    .withSemitoneFormatting();
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
                return pmd()
                    .asDecibelWithRange(-24, 24)
                    .withName(config().extraName)
                    .withDefault(0.f);
            }
            if constexpr (Model == fmd::Comb)
            {
                return pmd()
                    .asPercentBipolar()
                    .withCustomMinDisplay("-")
                    .withCustomMaxDisplay("+")
                    .withName(config().extraName)
                    .withDefault(1.f);
            }
            return pmd().asPercent().withName(config().extraName).withDefault(0.f);
        }

        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        namespace fpp = filtersplusplus;
        using pmd = basic_blocks::params::ParamMetaData;

        std::unordered_map<int, std::string> pbm{}, slm{}, drm{}, smm{};
        int drsi{0};
        const auto &opts = config();
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipPassband:
            if (opts.passbands.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < opts.passbands.size(); ++i)
            {
                if constexpr (Model == fmd::OBXD_Xpander)
                {
                    auto p = fpp::toString(opts.passbands[i].pt);
                    auto s = fpp::toString(opts.passbands[i].st);
                    if (s != "UNSUPPORTED")
                        p = p + " " + s;
                    pbm.insert({i, p});
                }
                else
                {
                    pbm.insert({i, fpp::toString(opts.passbands[i])});
                }
            }
            return pmd()
                .asInt()
                .withRange(0, opts.passbands.size() - 1)
                .withUnorderedMapFormatting(pbm)
                .withName("Passband");
        case ipSlope:
            if (opts.slopes.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < opts.slopes.size(); ++i)
            {
                slm.insert({i, fpp::toString(opts.slopes[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, opts.slopes.size() - 1)
                .withUnorderedMapFormatting(slm)
                .withName("Slope");
        case ipDrive:
            if (opts.drives.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");
            if constexpr (Model == fmd::VemberClassic)
            {
                drm.insert({0, fpp::toString(opts.drives[0])});
                drm.insert({1, this->getIntParam(ipPassband) < 3 ? fpp::toString(opts.drives[1])
                                                                 : fpp::toString(opts.drives[2])});
                drsi = 1;
            }
            else
            {
                for (int i = 0; i < opts.drives.size(); ++i)
                {
                    drm.insert({i, fpp::toString(opts.drives[i])});
                }
                drsi = opts.drives.size() - 1;
            }

            return pmd().asInt().withRange(0, drsi).withUnorderedMapFormatting(drm).withName(
                "Drive");
        case ipSubmodel:
            if (opts.submodels.size() < 2)
                return pmd().withLinearScaleFormatting("").withName("");

            for (int i = 0; i < opts.submodels.size(); ++i)
            {
                smm.insert({i, fpp::toString(opts.submodels[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, opts.submodels.size() - 1)
                .withUnorderedMapFormatting(smm)
                .withName(opts.subName);
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
            auto freqL = this->getFloatParam(fpCutoffL) + keytrackOn * pitch;

            if constexpr (Model == fmd::CytomicSVF)
            {
                // Andy assumes A = pow(10, dB/40), our converter uses dB/20, hence the * .5f
                extra = this->dbToLinear(extra * 0.5f);
            }

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

                auto freqR = this->getFloatParam(fpCutoffR) + keytrackOn * pitch;
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
    bool getKeytrackDefault() const
    {
        return Model == filtersplusplus::FilterModel::Comb ? true : false;
    }

    bool checkParameterConsistency() const { return true; }
    size_t silentSamplesLength() const { return 10; }

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

    filtersplusplus::ModelConfig configFilter()
    {
        namespace fpp = filtersplusplus;
        const auto &opts = config();
        fpp::ModelConfig cfg{};
        if constexpr (Model == fmd::OBXD_Xpander)
        {
            int id = std::clamp(this->getIntParam(ipPassband), 0, (int)opts.passbands.size() - 1);
            return opts.passbands[id];
        }
        else
        {
            uint32_t pid{0}, sid{0}, did{0}, mid{0};
            int ps = opts.passbands.size();
            int ss = opts.slopes.size();
            int ds = opts.drives.size();
            int ms = opts.submodels.size();

            if (ps > 1)
            {
                pid = std::clamp(this->getIntParam(ipPassband), 0, ps - 1);
            }
            cfg.pt = opts.passbands[pid];

            if (ss > 1)
            {
                sid = std::clamp(this->getIntParam(ipSlope), 0, ss - 1);
            }
            cfg.st = opts.slopes[sid];

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
            cfg.dt = opts.drives[did];

            if (ms > 1)
            {
                mid = std::clamp(this->getIntParam(ipSubmodel), 0, ms - 1);
            }
            cfg.mt = opts.submodels[mid];

            return cfg;
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

} // namespace sst::voice_effects::filter
#endif // FILTERSPLUSPLUS_H
