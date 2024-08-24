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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_SHORTDELAY_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_SHORTDELAY_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/SSESincDelayLine.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"
#include "DelaySupport.h"

namespace sst::voice_effects::delay
{
template <typename VFXConfig> struct ShortDelay : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Short Delay"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{1};

    static constexpr float maxMiliseconds{250.f};

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    static constexpr int shortLineSize{15}; // enough for 250ms at 96k
    static constexpr int longLineSize{17};  // enough for 250ms at 96k

    enum FloatParams
    {
        fpTimeL,
        fpTimeR,
        fpFeedback,
        fpCrossFeed,
        fpLowCut,
        fpHighCut
    };

    enum IntParams
    {
        ipStereo
    };

    ShortDelay(const SincTable &st)
        : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>(), lp(this), hp(this)
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~ShortDelay()
    {
        if (isShort)
        {
            lineSupport[0].template returnLines<shortLineSize>(this);
            lineSupport[1].template returnLines<shortLineSize>(this);
        }
        else
        {
            lineSupport[0].template returnLines<longLineSize>(this);
            lineSupport[1].template returnLines<longLineSize>(this);
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        bool stereo = this->getIntParam(ipStereo) > 0;

        switch (idx)
        {
        case fpTimeL:
            return pmd()
                .asFloat()
                .withRange(0, maxMiliseconds)
                .withDefault(50)
                .withLinearScaleFormatting("ms")
                .withName(std::string("Time") + (stereo ? " L" : ""));

        case fpTimeR:
            return pmd()
                .asFloat()
                .withRange(0, maxMiliseconds)
                .withDefault(50)
                .withLinearScaleFormatting("ms")
                .withName(!stereo ? std::string() : "Time R");

        case fpFeedback:
            return pmd().asPercent().withDefault(0.f).withName("Feedback");
        case fpCrossFeed:
            return pmd().asPercent().withDefault(0.f).withName(!stereo ? std::string()
                                                                       : "CrossFeed");

        case fpLowCut:
            return pmd().asAudibleFrequency().withDefault(-60).withName("LowCut").deactivatable();
        case fpHighCut:
            return pmd().asAudibleFrequency().withDefault(70).withName("HighCut").deactivatable();
        }
        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        return pmd().asBool().withDefault(true).withName("Stereo");
    }

    void initVoiceEffect()
    {
        if (this->getSampleRate() * 0.1 > (1 << 14))
        {
            isShort = false;
            for (int i = 0; i < 2; ++i)
            {
                if (lineSupport[i].template hasLinePointer<shortLineSize>())
                {
                    // swapped short to long. Return
                    lineSupport[i].template returnLines<shortLineSize>(this);
                }
                if (!lineSupport[i].template hasLinePointer<longLineSize>())
                {
                    lineSupport[i].template preReserveLines<longLineSize>(this);
                    lineSupport[i].template prepareLine<longLineSize>(this, sSincTable);
                }
                else
                {
                    lineSupport[i].template clearLine<longLineSize>();
                }
            }
        }
        else
        {
            isShort = true;
            for (int i = 0; i < 2; ++i)
            {
                if (lineSupport[i].template hasLinePointer<longLineSize>())
                {
                    // swapped long to short. Return
                    lineSupport[i].template returnLines<longLineSize>(this);
                }
                if (!lineSupport[i].template hasLinePointer<shortLineSize>())
                {
                    lineSupport[i].template preReserveLines<shortLineSize>(this);
                    lineSupport[i].template prepareLine<shortLineSize>(this, sSincTable);
                }
                else
                {
                    lineSupport[i].template clearLine<shortLineSize>();
                }
            }
        }

        lipolDelay[0].set_target_instant(
            std::clamp(this->getFloatParam(fpTimeL), 0.f, maxMiliseconds) * this->getSampleRate() /
            1000.f);
        lipolDelay[1].set_target_instant(
            std::clamp(this->getFloatParam(fpTimeR), 0.f, maxMiliseconds) * this->getSampleRate() /
            1000.f);

        lipolFb.set_target_instant(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));
        lipolCross.set_target_instant(std::clamp(this->getFloatParam(fpCrossFeed), 0.f, 1.f));

        lp.suspend();
        hp.suspend();
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <typename T>
    void stereoImpl(const std::array<T *, 2> &lines, float *datainL, float *datainR,
                    float *dataoutL, float *dataoutR)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;
        auto stereoSwitch = this->getIntParam(ipStereo);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        float FIRipol = static_cast<float>(SincTable::FIRipol_N);

        auto timeL = this->getFloatParam(fpTimeL);
        auto timeR = (stereoSwitch) ? this->getFloatParam(fpTimeR) : timeL;

        lipolDelay[0].set_target(std::max(
            (std::clamp(timeL, 0.f, maxMiliseconds) * this->getSampleRate() / 1000.f), FIRipol));
        lipolDelay[1].set_target(std::max(
            (std::clamp(timeR, 0.f, maxMiliseconds) * this->getSampleRate() / 1000.f), FIRipol));

        float ld alignas(16)[2][VFXConfig::blockSize];
        lipolDelay[0].store_block(ld[0]);
        lipolDelay[1].store_block(ld[1]);

        lipolFb.set_target(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));
        float feedback alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(feedback);

        lipolCross.set_target(std::clamp(this->getFloatParam(fpCrossFeed), 0.f, 1.f));
        float crossfeed alignas(16)[VFXConfig::blockSize];
        lipolCross.store_block(crossfeed);

        auto lpen = !this->getIsDeactivated(fpHighCut);
        auto hpen = !this->getIsDeactivated(fpLowCut);

        if (lpen)
            lp.coeff_LP2B(lp.calc_omega(this->getFloatParam(fpHighCut) / 12.0), 0.707);
        if (hpen)
            hp.coeff_HP(hp.calc_omega(this->getFloatParam(fpLowCut) / 12.0), 0.707);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto out0 = lines[0]->read(ld[0][i]);
            auto out1 = lines[1]->read(ld[1][i]);

            if (lpen)
                lp.process_sample(out0, out1, out0, out1);
            if (hpen)
                hp.process_sample(out0, out1, out0, out1);

            dataoutL[i] = out0;
            dataoutR[i] = out1;

            auto fbc0 = feedback[i] * dataoutL[i];
            auto fbc1 = feedback[i] * dataoutR[i];

            if (stereoSwitch)
            {
                fbc0 += crossfeed[i] * dataoutR[i];
                fbc1 += crossfeed[i] * dataoutL[i];
            }

            // soft clip for now
            fbc0 = std::clamp(fbc0, -1.5f, 1.5f);
            fbc0 = fbc0 - 4.0 / 27.0 * fbc0 * fbc0 * fbc0;

            fbc1 = std::clamp(fbc1, -1.5f, 1.5f);
            fbc1 = fbc1 - 4.0 / 27.0 * fbc1 * fbc1 * fbc1;

            lines[0]->write(datainL[i] + fbc0);
            lines[1]->write(datainR[i] + fbc1);
        }
    }

    template <typename T> void monoImpl(T *line, float *datainL, float *dataoutL)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;
        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        float FIRipol = static_cast<float>(SincTable::FIRipol_N);

        lipolDelay[0].set_target(
            std::max((std::clamp(this->getFloatParam(fpTimeL), 0.f, maxMiliseconds) *
                      this->getSampleRate() / 1000.f),
                     FIRipol));

        lipolFb.set_target(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));

        float ld alignas(16)[VFXConfig::blockSize];
        lipolDelay[0].store_block(ld);

        float fb alignas(16)[VFXConfig::blockSize];
        lipolFb.store_block(fb);

        auto lpen = !this->getIsDeactivated(fpHighCut);
        auto hpen = !this->getIsDeactivated(fpLowCut);
        if (lpen)
            lp.coeff_LP2B(lp.calc_omega(this->getFloatParam(fpHighCut) / 12.0), 0.707);
        if (hpen)
            hp.coeff_HP(hp.calc_omega(this->getFloatParam(fpLowCut) / 12.0), 0.707);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto output = line->read(ld[i]);

            auto nope = 0.f;
            if (lpen)
                lp.process_sample(output, nope, output, nope);
            if (hpen)
                hp.process_sample(output, nope, output, nope);

            dataoutL[i] = output;

            auto feedback = fb[i] * dataoutL[i];

            // soft clip for now
            feedback = std::clamp(feedback, -1.5f, 1.5f);
            feedback = feedback - 4.0 / 27.0 * feedback * feedback * feedback;

            line->write(datainL[i] + feedback);
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        if (isShort)
        {
            stereoImpl(std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                                  lineSupport[1].template getLinePointer<shortLineSize>()},
                       datainL, datainR, dataoutL, dataoutR);
        }
        else
        {
            stereoImpl(std::array{lineSupport[0].template getLinePointer<longLineSize>(),
                                  lineSupport[1].template getLinePointer<longLineSize>()},
                       datainL, datainR, dataoutL, dataoutR);
        }
    }

    void processMonoToStereo(float *datain, float *dataoutL, float *dataoutR, float pitch)
    {
        if (isShort)
        {
            stereoImpl(std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                                  lineSupport[1].template getLinePointer<shortLineSize>()},
                       datain, datain, dataoutL, dataoutR);
        }
        else
        {
            stereoImpl(std::array{lineSupport[0].template getLinePointer<longLineSize>(),
                                  lineSupport[1].template getLinePointer<longLineSize>()},
                       datain, datain, dataoutL, dataoutR);
        }
    }

    void processMonoToMono(float *datain, float *dataout, float pitch)
    {
        if (isShort)
        {
            monoImpl(lineSupport[0].template getLinePointer<shortLineSize>(), datain, dataout);
        }
        else
        {
            monoImpl(lineSupport[0].template getLinePointer<shortLineSize>(), datain, dataout);
        }
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }
    bool checkParameterConsistency() const { return true; }
    size_t tailLength() const
    {
        auto fbsum = this->getFloatParam(fpFeedback) + this->getFloatParam(fpCrossFeed);
        if (fbsum > 1)
        {
            return -1;
        }
        auto tm = lipolDelay[0].target;
        if (this->getIntParam(ipStereo))
            tm = std::max(tm, lipolDelay[1].target);

        // so fbsum ^ N = 1e-4
        // n log fbsum = log 1e-4
        // n = log 1e-4 / log fbsum
        // so if you use log10 its -4 / log fbsum
        //
        // You get at least one hit so you add a time to N
        // so (N+1) tm is the ringout.
        // But that log_fbsum is expensive. So lets just store
        // some static values for values 0.05,.... up to 1
        static constexpr float fbVals[]{3.074487147360963,  4.0,
                                        4.854906619166852,  5.7227062322935724,
                                        6.643856189774724,  7.649957157572793,
                                        8.773238967862678,  10.05176637892824,
                                        11.534441578462017, 13.287712379549449,
                                        15.406106280365744, 18.03030220777539,
                                        21.380468045300727, 25.82278494325153,
                                        32.01569111860437,  41.275404634064685,
                                        56.672415948335924, 87.41738130713135,
                                        179.5622699214194,  250};
        auto ct = std::clamp((int)floor(fbsum * 20), 0, 19);
        return tm * (fbVals[ct] + 1);
    }

    std::array<details::DelayLineSupport, 2> lineSupport;
    bool isShort{true};

    std::array<float, numFloatParams> mLastParam{};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb, lipolCross,
        lipolDelay[2];

    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType lp, hp;
};

} // namespace sst::voice_effects::delay

#endif // SHORTCIRCUITXT_EqNBandParametric_H
