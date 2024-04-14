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

namespace sst::voice_effects::delay
{
template <typename VFXConfig> struct ShortDelay : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Short Delay"};

    static constexpr int numFloatParams{6};
    static constexpr int numIntParams{0};

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    enum FloatParams
    {
        fpTimeL,
        fpTimeR,
        fpFeedback,
        fpCrossFeed,
        fpLowCut,
        fpHighCut
    };

    ShortDelay(const SincTable &st)
        : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>(), lp(this), hp(this)
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
        this->preReservePool(sizeof(short_line));
        this->preReservePool(sizeof(long_line));
    }

    ~ShortDelay()
    {
        for (int i = 0; i < 2; ++i)
        {
            // Return to memory pool
            if (isShort && shortDelays[i])
            {
                shortDelays[i]->~short_line();
                this->returnBlock(shortLineBuffers[i], sizeof(short_line));
            }
            if (!isShort && longDelays[i])
            {
                longDelays[i]->~long_line();
                this->returnBlock(longLineBuffers[i], sizeof(long_line));
            }
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case fpTimeL:
            return pmd()
                .asFloat()
                .withRange(0, 100)
                .withDefault(50)
                .withLinearScaleFormatting("ms")
                .withName("Time L");

        case fpTimeR:
            return pmd()
                .asFloat()
                .withRange(0, 100)
                .withDefault(50)
                .withLinearScaleFormatting("ms")
                .withName("Time R");

        case fpFeedback:
            return pmd().asPercent().withDefault(0.f).withName("Feedback");
        case fpCrossFeed:
            return pmd().asPercent().withDefault(0.f).withName("CrossFeed");

        case fpLowCut:
            return pmd().asAudibleFrequency().withDefault(-60).withName("LowCut");
        case fpHighCut:
            return pmd().asAudibleFrequency().withDefault(70).withName("HighCut");
        }
        return pmd().withName("Error");
    }

    void initVoiceEffect()
    {
        if (this->getSampleRate() * 0.1 > (1 << 14))
        {
            isShort = false;
            for (int i = 0; i < 2; ++i)
            {
                longLineBuffers[i] = this->checkoutBlock(sizeof(long_line));
                longDelays[i] = new (longLineBuffers[i]) long_line(sSincTable);
            }
        }
        else
        {
            isShort = true;
            for (int i = 0; i < 2; ++i)
            {
                shortLineBuffers[i] = this->checkoutBlock(sizeof(short_line));
                shortDelays[i] = new (shortLineBuffers[i]) short_line(sSincTable);
            }
        }

        lipolDelay[0].set_target_instant(
            std::clamp(this->getFloatParam(fpTimeL) / 1000.0f, 0.f, 0.1f));
        lipolDelay[1].set_target_instant(
            std::clamp(this->getFloatParam(fpTimeR) / 1000.0f, 0.f, 0.1f));

        lipolFb.set_target_instant(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));
        lipolCross.set_target_instant(std::clamp(this->getFloatParam(fpCrossFeed), 0.f, 1.f));

        lp.suspend();
        hp.suspend();
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <typename T>
    void processImpl(const std::array<T *, 2> &lines, float *datainL, float *datainR,
                     float *dataoutL, float *dataoutR, float pitch)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;
        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);

        lipolDelay[0].set_target(std::clamp(this->getFloatParam(fpTimeL) / 1000.0f, 0.f, 0.1f));
        lipolDelay[1].set_target(std::clamp(this->getFloatParam(fpTimeR) / 1000.0f, 0.f, 0.1f));

        lipolFb.set_target(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));
        lipolCross.set_target(std::clamp(this->getFloatParam(fpCrossFeed), 0.f, 1.f));

        float ld alignas(16)[2][VFXConfig::blockSize];
        lipolDelay[0].store_block(ld[0]);
        lipolDelay[1].store_block(ld[1]);

        float fb alignas(16)[2][VFXConfig::blockSize];
        lipolFb.store_block(fb[0]);
        lipolCross.store_block(fb[1]);

        lp.coeff_LP2B(lp.calc_omega(this->getFloatParam(fpHighCut) / 12.0), 0.707);
        hp.coeff_HP(hp.calc_omega(this->getFloatParam(fpLowCut) / 12.0), 0.707);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            dataoutL[i] = lines[0]->read(ld[0][i] * this->getSampleRate());
            dataoutR[i] = lines[1]->read(ld[1][i] * this->getSampleRate());

            auto fbc0 = fb[0][i] * dataoutL[i] + fb[1][i] * dataoutR[i];
            auto fbc1 = fb[0][i] * dataoutR[i] + fb[1][i] * dataoutL[i];

            // soft clip for now
            fbc0 = std::clamp(fbc0, -1.5f, 1.5f);
            fbc0 = fbc0 - 4.0 / 27.0 * fbc0 * fbc0 * fbc0;

            fbc1 = std::clamp(fbc1, -1.5f, 1.5f);
            fbc1 = fbc1 - 4.0 / 27.0 * fbc1 * fbc1 * fbc1;

            lp.process_sample(fbc0, fbc1, fbc0, fbc1);
            hp.process_sample(fbc0, fbc1, fbc0, fbc1);

            lines[0]->write(datainL[i] + fbc0);
            lines[1]->write(datainR[i] + fbc1);
        }
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        if (isShort)
            processImpl(shortDelays, datainL, datainR, dataoutL, dataoutR, pitch);
        else
            processImpl(longDelays, datainL, datainR, dataoutL, dataoutR, pitch);
    }

  protected:
    using short_line = sst::basic_blocks::dsp::SSESincDelayLine<1 << 14>;
    using long_line = sst::basic_blocks::dsp::SSESincDelayLine<1 << 16>;
    std::array<uint8_t *, 2> shortLineBuffers{nullptr, nullptr};
    std::array<uint8_t *, 2> longLineBuffers{nullptr, nullptr};
    std::array<short_line *, 2> shortDelays{nullptr, nullptr};
    std::array<long_line *, 2> longDelays{nullptr, nullptr};
    bool isShort{true};
    std::array<float, numFloatParams> mLastParam{};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolFb, lipolCross,
        lipolDelay[2];

    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType lp, hp;
};

} // namespace sst::voice_effects::delay

#endif // SHORTCIRCUITXT_EqNBandParametric_H
