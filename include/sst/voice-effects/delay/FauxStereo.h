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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_FAUXSTEREO_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_FAUXSTEREO_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"
#include "DelaySupport.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/SSESincDelayLine.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/MidSide.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"

namespace sst::voice_effects::delay
{
template <typename VFXConfig> struct FauxStereo : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Faux Stereo"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{0};

    static constexpr int shortLineSize{14}, longLineSize{16};

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;

    const SincTable &sSincTable;

    FauxStereo(const SincTable &st) : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>()
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~FauxStereo()
    {
        if (isShort)
            lineSupport.returnLines<shortLineSize>(this);
        else
            lineSupport.returnLines<longLineSize>(this);
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case 0:
            return pmd()
                .asFloat()
                .withRange(-24, 24)
                .withDefault(6)
                .withLinearScaleFormatting("db")
                .withName("Amplitude");
        case 1:
            return pmd()
                .asFloat()
                .withRange(0, 100)
                .withDefault(30)
                .withLinearScaleFormatting("ms")
                .withName("Delay");
        case 2:
            return pmd()
                .asPercent()
                .withDefault(0.5)
                .withName("Source (M/S)")
                .withCustomMaxDisplay("Side")
                .withCustomMinDisplay("Mid");
        }
        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        if (this->getSampleRate() * 0.1 > (1 << shortLineSize))
        {
            isShort = false;
            lineSupport.preReserveLines<longLineSize>(this);
            lineSupport.prepareLine<longLineSize>(this, sSincTable);
        }
        else
        {
            isShort = true;
            lineSupport.preReserveLines<shortLineSize>(this);
            lineSupport.prepareLine<shortLineSize>(this, sSincTable);
        }

        auto amp = this->getFloatParam(0);
        auto dly = this->getFloatParam(1);
        auto msp = this->getFloatParam(2);
        lipolAmp.set_target_instant(this->dbToLinear(amp));
        lipolSource.set_target_instant(std::clamp(msp, 0.f, 1.f));
        lipolDelay.set_target_instant(std::clamp(dly / 1000.0f, 0.f, 0.1f));
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <typename Line>
    void processOntoLine(Line *line, float *datainL, float *datainR, float *dataoutL,
                         float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;
        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        auto amp = this->getFloatParam(0);
        auto dly = this->getFloatParam(1);
        auto msp = this->getFloatParam(2);
        lipolAmp.set_target(this->dbToLinear(amp));
        lipolSource.set_target(std::clamp(msp, 0.f, 1.f));
        lipolDelay.set_target(std::clamp(dly / 1000.0f, 0.f, 0.1f));

        float mid alignas(16)[VFXConfig::blockSize], side alignas(16)[VFXConfig::blockSize],
            sideDly alignas(16)[VFXConfig::blockSize];

        sdsp::encodeMS<VFXConfig::blockSize>(datainL, datainR, mid, side);

        lipolSource.fade_block_to(mid, side, sideDly);

        // do delay here
        float dlyTime alignas(16)[VFXConfig::blockSize];
        lipolDelay.store_block(dlyTime);

        for (size_t i = 0; i < VFXConfig::blockSize; ++i)
        {
            line->write(sideDly[i]);
            sideDly[i] = line->read(dlyTime[i] * this->getSampleRate());
        }

        lipolAmp.MAC_block_to(sideDly, side);
        sdsp::decodeMS<VFXConfig::blockSize>(mid, side, dataoutL, dataoutR);
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        if (isShort)
        {
            processOntoLine(lineSupport.getLinePointer<shortLineSize>(), datainL, datainR, dataoutL,
                            dataoutR, pitch);
        }
        else
        {
            processOntoLine(lineSupport.getLinePointer<longLineSize>(), datainL, datainR, dataoutL,
                            dataoutR, pitch);
        }
    }

  protected:
    details::DelayLineSupport lineSupport;

    bool isShort{true};
    std::array<float, numFloatParams> mLastParam{};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolAmp, lipolSource, lipolDelay;
};

} // namespace sst::voice_effects::delay

#endif // SHORTCIRCUITXT_EqNBandParametric_H
