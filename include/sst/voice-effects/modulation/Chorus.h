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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_CHORUS_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_CHORUS_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"

#include "../VoiceEffectCore.h"

#include <iostream>

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/SSESincDelayLine.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/RNG.h"
#include "sst/basic-blocks/tables/SincTableProvider.h"
#include "../delay/DelaySupport.h"

#include "sst/basic-blocks/modulators/SimpleLFO.h"

namespace sst::voice_effects::modulation
{
template <typename VFXConfig> struct Chorus : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Chorus"};

    static constexpr int numFloatParams{4};
    static constexpr int numIntParams{2};

    static constexpr float maxMiliseconds{50.f};

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    basic_blocks::dsp::RNG rng;

    enum FloatParams
    {
        fpTime,
        fpFeedback,
        fpRate,
        fpDepth,
    };

    enum IntParams
    {
        ipShape,
        ipStereo,
    };

    Chorus(const SincTable &st) : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~Chorus()
    {
        lineSupport[0].returnAll(this);
        lineSupport[1].returnAll(this);
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case fpTime:
            return pmd()
                .asFloat()
                .withRange(0.f, maxMiliseconds * .5f)
                .withDefault(7.5f)
                .withLinearScaleFormatting("ms")
                .withName("Base Delay");
        case fpFeedback:
            return pmd().asPercent().withDefault(0.f).withName("Feedback");
        case fpRate:
            return pmd().asLfoRate(-3, 4).withName("Rate");
        case fpDepth:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(.25f)
                .withLinearScaleFormatting("%", 100.f)
                .withName("Depth");
        }
        return pmd().withName("Error");
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch (idx)
        {
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(false);
        case ipShape:
            return pmd()
                .asInt()
                .withRange(0, 6)
                .withUnorderedMapFormatting({
                    {0, "Sine"},
                    {1, "Triangle"},
                    {2, "Ramp Up"},
                    {3, "Ramp Down"},
                    {4, "Square"},
                    {5, "Noise"},
                    {6, "S&H"},
                })
                .withName("LFO shape");
        }
        return pmd().asInt().withName("Error");
    }

    size_t lineSize() const
    {
        int sz{1};

        while (this->getSampleRate() * maxMiliseconds * 0.001 > 1 << sz)
        {
            sz++;
        }

        return static_cast<size_t>(std::clamp(sz, 12, 20));
    }

    void initVoiceEffect()
    {
        for (int i = 0; i < 2; ++i)
        {
            lineSupport[i].returnAllExcept(lineSize(), this);
            lineSupport[i].reservePrepareAndClear(lineSize(), this, sSincTable);
        }

        timeLerp[0].set_target_instant(
            std::clamp(this->getFloatParam(fpTime), 0.f, maxMiliseconds) * this->getSampleRate() /
            1000.f);
        timeLerp[1].set_target_instant(
            std::clamp(this->getFloatParam(fpTime), 0.f, maxMiliseconds) * this->getSampleRate() /
            1000.f);

        feedbackLerp.set_target_instant(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    using lfo_t = sst::basic_blocks::modulators::SimpleLFO<Chorus, VFXConfig::blockSize>;
    lfo_t LFO{this};
    typename lfo_t::Shape lfoShape = lfo_t::Shape::SINE;

    void shapeCheck()
    {
        switch (this->getIntParam(ipShape))
        {
        case 0:
            lfoShape = lfo_t::SINE;
            break;
        case 1:
            lfoShape = lfo_t::TRI;
            break;
        case 2:
            lfoShape = lfo_t::RAMP;
            break;
        case 3:
            lfoShape = lfo_t::DOWN_RAMP;
            break;
        case 4:
            lfoShape = lfo_t::PULSE;
            break;
        case 5:
            lfoShape = lfo_t::SMOOTH_NOISE;
            break;
        case 6:
            lfoShape = lfo_t::SH_NOISE;
            break;
        }
    }

    bool phaseSet = false;

    template <typename T>
    void stereoImpl(const std::array<T *, 2> &lines, const float *const datainL,
                    const float *const datainR, float *dataoutL, float *dataoutR)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        bool stereo = this->getIntParam(ipStereo);
        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        shapeCheck();

        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                auto phase = rng.unif01();
                LFO.applyPhaseOffset(phase);
            }
            else if (lfoShape == lfo_t::PULSE)
            {

                LFO.applyPhaseOffset(rng.boolean() ? 0.f : .5f);
            }
            else
            {

                LFO.applyPhaseOffset(0.f);
            }
            phaseSet = true;
        }
        LFO.process_block(lfoRate, 0.f, lfoShape);

        auto baseTime = this->getFloatParam(fpTime);

        float lfoValueL = LFO.lastTarget * baseTime * lfoDepth;
        float lfoValueR = LFO.lastTarget * baseTime * lfoDepth * (stereo ? -1 : 1);

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        mech::copy_from_to<VFXConfig::blockSize>(datainR, dataoutR);
        float FIRipol = static_cast<float>(SincTable::FIRipol_N);

        timeLerp[0].set_target(std::max((std::clamp(baseTime + (lfoValueL), 0.f, maxMiliseconds) *
                                         this->getSampleRate() / 1000.f),
                                        FIRipol));
        timeLerp[1].set_target(std::max((std::clamp(baseTime + (lfoValueR), 0.f, maxMiliseconds) *
                                         this->getSampleRate() / 1000.f),
                                        FIRipol));

        feedbackLerp.set_target(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));

        float ld alignas(16)[2][VFXConfig::blockSize];
        timeLerp[0].store_block(ld[0]);
        timeLerp[1].store_block(ld[1]);

        float fb alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fb);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto out0 = lines[0]->read(ld[0][i]);
            auto out1 = lines[1]->read(ld[1][i]);

            dataoutL[i] = out0;
            dataoutR[i] = out1;

            auto fbL = fb[i] * dataoutL[i];
            auto fbR = fb[i] * dataoutR[i];

            // soft clip for now
            fbL = std::clamp(fbL, -1.5f, 1.5f);
            fbL = fbL - 4.0 / 27.0 * fbL * fbL * fbL;

            fbR = std::clamp(fbR, -1.5f, 1.5f);
            fbR = fbR - 4.0 / 27.0 * fbR * fbR * fbR;

            lines[0]->write(datainL[i] + fbL);
            lines[1]->write(datainR[i] + fbR);
        }
    }

    template <typename T> void monoImpl(T *line, const float *const datainL, float *dataoutL)
    {
        namespace mech = sst::basic_blocks::mechanics;
        namespace sdsp = sst::basic_blocks::dsp;

        auto lfoRate = this->getFloatParam(fpRate);
        auto lfoDepth = this->getFloatParam(fpDepth);
        shapeCheck();

        if (!phaseSet)
        {
            if (lfoShape == lfo_t::SINE || lfoShape == lfo_t::TRI)
            {
                auto phase = rng.unif01();
                LFO.applyPhaseOffset(phase);
            }
            else if (lfoShape == lfo_t::PULSE)
            {

                LFO.applyPhaseOffset(rng.boolean() ? 0.f : .5f);
            }
            else
            {

                LFO.applyPhaseOffset(0.f);
            }
            phaseSet = true;
        }
        LFO.process_block(lfoRate, 0.f, lfoShape);

        auto baseTime = this->getFloatParam(fpTime);

        float lfoValue = LFO.lastTarget * baseTime * lfoDepth;

        mech::copy_from_to<VFXConfig::blockSize>(datainL, dataoutL);
        float FIRipol = static_cast<float>(SincTable::FIRipol_N);

        timeLerp[0].set_target(std::max((std::clamp(baseTime + (lfoValue), 0.f, maxMiliseconds) *
                                         this->getSampleRate() / 1000.f),
                                        FIRipol));

        feedbackLerp.set_target(std::clamp(this->getFloatParam(fpFeedback), 0.f, 1.f));

        float ld alignas(16)[VFXConfig::blockSize];
        timeLerp[0].store_block(ld);

        float fb alignas(16)[VFXConfig::blockSize];
        feedbackLerp.store_block(fb);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto output = line->read(ld[i]);

            dataoutL[i] = output;

            auto feed = fb[i] * dataoutL[i];

            // soft clip for now
            feed = std::clamp(feed, -1.5f, 1.5f);
            feed = feed - 4.0 / 27.0 * feed * feed * feed;

            line->write(datainL[i] + feed);
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        // a very rare case where [&] is appropriate, binding the entire argument set for one call
        lineSupport[0].dispatch(lineSize(), [&](auto N) {
            auto *line0 = lineSupport[0].template getLinePointer<N>();
            auto *line1 = lineSupport[1].template getLinePointer<N>();
            std::array<decltype(line0), 2> lines = {line0, line1};
            stereoImpl(lines, datainL, datainR, dataoutL, dataoutR);
        });
    }

    void processMonoToStereo(const float *const datain, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processStereo(datain, datain, dataoutL, dataoutR, pitch);
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        lineSupport[0].dispatch(lineSize(), [&](auto N) {
            auto *line = lineSupport[0].template getLinePointer<N>();
            monoImpl(line, datain, dataout);
        });
    }

    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

    size_t silentSamplesLength() const { return this->getSampleRate() * maxMiliseconds * .001f; }

  protected:
    std::array<delay::details::DelayLineSupport, 2> lineSupport;
    bool isShort{true};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> feedbackLerp, timeLerp[2];

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

} // namespace sst::voice_effects::modulation

#endif // SHORTCIRCUITXT_CHORUS_H
