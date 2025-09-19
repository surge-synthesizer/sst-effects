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

#include "../VoiceEffectCore.h"

#include <iostream>

namespace sst::voice_effects::filter
{
template <typename VFXConfig, filtersplusplus::FilterModel Model>
struct FiltersPlusPlus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Filters++"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{5};

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

    FiltersPlusPlus() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(priorFP.begin(), priorFP.end(), -1000.f);
        std::fill(priorIP.begin(), priorIP.end(), -1);

        filter.init();
        filter.setFilterModel(Model);

        using fmd = filtersplusplus::FilterModel;

        if constexpr (Model == fmd::VemberClassic)
        {
            num_passbands = 4;
            num_slopes = 2;
            num_drives = 3;

            passbands[0] = 0x04;
            passbands[1] = 0x08;
            passbands[2] = 0x0C;
            passbands[3] = 0x10;
            slopes[0] = 0x12;
            slopes[1] = 0x24;
            drives[0] = 0x10;
            drives[1] = 0x14;
            drives[2] = 0x18;
        }
        if constexpr (Model == fmd::K35)
        {
            num_passbands = 2;

            passbands[0] = 0x04;
            passbands[1] = 0x08;
            drives[0] = 0x39;

            extraName = "Drive";
        }
        if constexpr (Model == fmd::OBXD_4Pole)
        {
            passbands[0] = 0x04;
            slopes[0] = 0x40;

            extraName = "Slope";
        }
        if constexpr (Model == fmd::VintageLadder)
        {
            num_submodels = 2;

            passbands[0] = 0x04;
            submodels[0] = 0x22;
            submodels[1] = 0x26;

            subName = "Method";
        }
        if constexpr (Model == fmd::CutoffWarp)
        {
            num_passbands = 5;
            num_drives = 4;
            num_submodels = 3;

            passbands[0] = 0x04;
            passbands[1] = 0x08;
            passbands[2] = 0x0C;
            passbands[3] = 0x10;
            passbands[4] = 0x18;

            drives[0] = 0x60;
            drives[1] = 0x61;
            drives[2] = 0x62;
            drives[3] = 0x63;

            submodels[0] = 0x40;
            submodels[1] = 0x42;
            submodels[2] = 0x44;
        }
        if constexpr (Model == fmd::ResonanceWarp)
        {
            num_passbands = 5;
            num_drives = 4;
            num_submodels = 2;

            passbands[0] = 0x04;
            passbands[1] = 0x08;
            passbands[2] = 0x0C;
            passbands[3] = 0x10;
            passbands[4] = 0x18;

            drives[0] = 0x60;
            drives[1] = 0x61;
            drives[2] = 0x62;
            drives[3] = 0x63;

            submodels[0] = 0x40;
            submodels[1] = 0x42;

            extraName = "Waveshaper";
        }
        if constexpr (Model == fmd::TriPole)
        {
            num_passbands = 4;
            num_submodels = 3;

            passbands[0] = 0x30 + 0b000;
            passbands[1] = 0x30 + 0b010;
            passbands[2] = 0x30 + 0b101;
            passbands[3] = 0x30 + 0b111;

            submodels[0] = 0x31;
            submodels[1] = 0x32;
            submodels[2] = 0x33;

            subName = "Output";
        }
    }

    ~FiltersPlusPlus() {}

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

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
            return pmd().asPercent().withName("Resonance").withDefault(0.f);
        case fpExtra:
            return pmd().asPercent().withName(extraName).withDefault(0.f);
        }

        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        namespace fpp = filtersplusplus;
        using pmd = basic_blocks::params::ParamMetaData;

        std::unordered_map<int, std::string> pbm{}, slm{}, drm{}, smm{};
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(true);
        case ipPassband:
            if (num_passbands < 2)
                return pmd().withName("");

            for (int i = 0; i < num_passbands; ++i)
            {
                pbm.insert({i, fpp::toString((fpp::Passband)passbands[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, num_passbands - 1)
                .withUnorderedMapFormatting(pbm)
                .withName("Passband");
        case ipSlope:
            if (num_slopes < 2)
                return pmd().withName("");

            for (int i = 0; i < num_slopes; ++i)
            {
                slm.insert({i, fpp::toString((fpp::Slope)slopes[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, num_slopes - 1)
                .withUnorderedMapFormatting(slm)
                .withName("Slope");
        case ipDrive:
            if (num_drives < 2)
                return pmd().withName("");

            for (int i = 0; i < num_drives; ++i)
            {
                drm.insert({i, fpp::toString((fpp::DriveMode)drives[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, num_drives - 1)
                .withUnorderedMapFormatting(drm)
                .withName("Drive");
        case ipSubmodel:
            if (num_submodels < 2)
                return pmd().withName("");

            for (int i = 0; i < num_submodels; ++i)
            {
                smm.insert({i, fpp::toString((fpp::FilterSubModel)submodels[i])});
            }
            return pmd()
                .asInt()
                .withRange(0, num_submodels - 1)
                .withUnorderedMapFormatting(smm)
                .withName(subName);
        }

        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        filter.setSampleRateAndBlockSize(this->getSampleRate(), VFXConfig::blockSize);
        setupFilter(true);
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    void setupFilter(bool first)
    {
        if (first)
            filter.reset();
        filter.setModelConfiguration(configFilter());
        if (!filter.prepareInstance())
        {
            filter.setFilterModel(filtersplusplus::FilterModel::CytomicSVF);
            filter.setPassband(filtersplusplus::Passband::LP);
            std::cout << " Invalid filter config, defaulting to " << filter.displayName()
                      << std::endl;
        }
    }

    template <bool stereo, bool stereoCoeff> void setCoeffs(float pitch)
    {
        /* std::array<float, numFloatParams> fp;
        std::array<int, numIntParams> ip;
        bool fDiff{false}, iDiff{false}

        for (int i = 0; i < numFloatParams; ++i)
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
        */
        // wrapping this next stuff in if (fDiff || iDiff)
        // sounds horrible. I'm not sure why

        auto reso = std::clamp(this->getFloatParam(fpResonance), 0.f, 1.f);
        auto morph = std::clamp(this->getFloatParam(fpExtra), 0.f, 1.f);

        auto freqL = this->getFloatParam(fpCutoffL);
        if (keytrackOn)
            freqL += pitch;

        if constexpr (!stereo)
        {
            filter.setMono();
            filter.makeCoefficients(0, freqL, reso, morph);
            return;
        }

        filter.setStereo();
        filter.makeCoefficients(0, freqL, reso, morph);
        if constexpr (!stereoCoeff)
        {
            filter.copyCoefficientsFromVoiceToVoice(0, 1);
            return;
        }

        auto freqR = this->getFloatParam(fpCutoffR);
        if (keytrackOn)
            freqR += pitch;
        filter.makeCoefficients(1, freqR, reso, morph);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        setCoeffs<false, false>(pitch);

        filter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            dataout[i] = filter.processMonoSample(datain[i]);
        }
        filter.concludeBlock();
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        if (this->getIntParam(ipStereo) > 0)
            setCoeffs<true, true>(pitch);
        else
            setCoeffs<true, false>(pitch);

        filter.prepareBlock();
        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            filter.processStereoSample(datainL[i], datainR[i], dataoutL[i], dataoutR[i]);
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

    filtersplusplus::Filter filter = filtersplusplus::Filter();
    int num_slopes{1}, num_passbands{1}, num_drives{1}, num_submodels{1};
    std::string driveName{"Drive"}, extraName{""}, subName{""};

    uint32_t passbands[5] = {0, 0, 0, 0, 0};
    uint32_t slopes[4] = {0, 0, 0, 0};
    uint32_t drives[4] = {0, 0, 0, 0};
    uint32_t submodels[3] = {0, 0, 0};

    filtersplusplus::ModelConfig configFilter()
    {
        namespace fpp = filtersplusplus;
        fpp::ModelConfig cfg{};
        uint32_t pid{0}, sid{0}, did{0}, mid{0};

        if (num_passbands > 1)
        {
            pid = std::clamp(this->getIntParam(ipPassband), 0, num_passbands - 1);
        }
        cfg.pt = (fpp::Passband)passbands[pid];

        if (num_slopes > 1)
        {
            sid = std::clamp(this->getIntParam(ipSlope), 0, num_slopes - 1);
        }
        cfg.st = (fpp::Slope)slopes[sid];

        if (num_drives > 1)
        {
            did = std::clamp(this->getIntParam(ipDrive), 0, num_drives - 1);
        }
        if constexpr (Model == filtersplusplus::FilterModel::VemberClassic)
        {
            // in vember classic notch the second drive mode is different
            if (pid == 3 && did == 1)
                did += 1;
        }
        cfg.dt = (fpp::DriveMode)drives[did];

        if (num_submodels > 1)
        {
            mid = std::clamp(this->getIntParam(ipSubmodel), 0, num_submodels - 1);
        }
        cfg.mt = (fpp::FilterSubModel)submodels[mid];

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
