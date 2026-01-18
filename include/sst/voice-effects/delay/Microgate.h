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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_MICROGATE_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_MICROGATE_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "../VoiceEffectCore.h"
#include "DelaySupport.h"

#include <cmath>

namespace sst::voice_effects::delay
{
template <typename VFXConfig> struct MicroGate : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *displayName{"MicroGate"};
    static constexpr const char *streamingName{"micro-gate-fx"};

    static constexpr uint16_t numFloatParams{4};
    static constexpr uint16_t numIntParams{0};

    static constexpr int maxTime{-2}; // 2^maxTime = max in seconds

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    enum FloatParams
    {
        fpRepeats,
        fpLoopLength,
        fpThreshold,
        fpThruLevel
    };

    MicroGate(SincTable &st) : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>() {}

    ~MicroGate()
    {
        lineSupport[0].returnAll(this);
        lineSupport[1].returnAll(this);
    }

    basic_blocks::params::ParamMetaData paramAt(uint16_t idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case fpRepeats:
            return pmd()
                .asFloat()
                .withLinearScaleFormatting("repeats")
                .withDecimalPlaces(0)
                .withName("Repeats")
                .withRange(2, 65)
                .withCustomMaxDisplay("inf")
                .withDefault(4);
        case fpLoopLength:
            return pmd()
                .asLog2SecondsRange(-8.f, maxTime, -6.f)
                .withMilisecondsBelowOneSecond()
                .withName("Repeat Length");
        case fpThreshold:
            return pmd()
                .asLinearDecibel()
                .withRange(-60, 12)
                .withName("Threshold")
                .withDefault(0.f)
                .withMultiplicativeModulationOffByDefault();
        case fpThruLevel:
            return pmd().asLinearDecibel().withName("Thru Level").withDefault(0.f);
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    size_t lineSize() const
    {
        int sz{1};
        auto ctt = this->getSampleRate() * std::pow(2, maxTime);
        while (ctt > 1 << sz)
        {
            sz++;
        }
        return static_cast<size_t>(std::clamp(sz, 12, 20));
    }

    void initVoiceEffect()
    {
        srComp = this->getSampleRate() / 48000.f;
        thruLevLerp.instantize();

        recSize = lineSize();
        for (int i = 0; i < 2; ++i)
        {
            zcf[i].setSRComp(srComp);

            lineSupport[i].returnAllExcept(recSize, this);
            lineSupport[i].reservePrepareAndClear(recSize, this, sSincTable);
        }
    }

    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <typename T>
    void stereoImpl(const std::array<T *, 2> &lines, const float *const datainL,
                    const float *const datainR, float *dataoutL, float *dataoutR)
    {

        namespace mech = sst::basic_blocks::mechanics;

        float thruLev alignas(16)[VFXConfig::blockSize];
        float tl = this->dbToLinear(this->getFloatParam(fpThruLevel));
        thruLevLerp.set_target(tl);
        thruLevLerp.store_block(thruLev);

        float threshold = this->dbToLinear(this->getFloatParam(fpThreshold));
        auto envL = mech::blockAbsAvg<VFXConfig::blockSize>(datainL);
        auto envR = mech::blockAbsAvg<VFXConfig::blockSize>(datainR);

        if (!ARM && (envL > threshold || envR > threshold))
        {
            ARM = true;
            repeated[0] = 0;
            repeated[1] = 0;
            zcf[0].newRun();
            zcf[1].newRun();
            lines[0]->clear();
            lines[1]->clear();
            recorded[0] = 0;
            recorded[1] = 0;
        }

        int repeats = static_cast<int>(std::clamp(this->getFloatParam(fpRepeats), 2.f, 65.f));
        if (repeats == 65)
            repeats = INT_MAX;

        float loopLength = std::max(
            FIRipol, this->note_to_pitch_ignoring_tuning(12 * this->getFloatParam(fpLoopLength)) *
                         this->getSampleRate());

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            float current[2] = {datainL[k], datainR[k]};

            for (int c = 0; c < 2; c++)
            {

                if (REC[c])
                {
                    lines[c]->write(current[c]);
                    REC[c] = recorded[c]++ <= 1 << recSize;
                }

                if (LOOP[c])
                {
                    if (repeated[c] > 0)
                    {
                        current[c] = lines[c]->readNaivelyAt(bufpos[c]);
                    }

                    if (bufpos[c]++ > loopLength)
                    {
                        if (zcf[c].checkSample(current[c]))
                        {
                            bufpos[c] = 0;
                            if (++repeated[c] >= repeats)
                            {
                                LOOP[c] = false;
                                REC[c] = false;
                                ARM = LOOP[0] || LOOP[1];
                            }
                        }
                    }
                    else
                    {
                        zcf[c].logSample(current[c]);
                    }
                }
                else
                {
                    if (ARM)
                    {
                        if (zcf[c].checkSampleAndSetDirection(current[c]))
                        {
                            bufpos[c] = 0;
                            REC[c] = true;
                            LOOP[c] = true;
                        }
                    }
                    current[c] *= thruLev[k];
                }
            }
            dataoutL[k] = current[0];
            dataoutR[k] = current[1];
        }
    }

    template <typename T> void monoImpl(T *line, const float *const datain, float *dataout)
    {
        namespace mech = sst::basic_blocks::mechanics;

        float thruLev alignas(16)[VFXConfig::blockSize];
        float tl = this->dbToLinear(this->getFloatParam(fpThruLevel));
        thruLevLerp.set_target(tl);
        thruLevLerp.store_block(thruLev);

        float threshold = this->dbToLinear(this->getFloatParam(fpThreshold));
        auto env = mech::blockAbsAvg<VFXConfig::blockSize>(datain);
        if (!ARM && env > threshold)
        {
            ARM = true;
            repeated[0] = 0;
            zcf[0].newRun();
            line->clear();
            recorded[0] = 0;
        }

        int repeats = static_cast<int>(std::clamp(this->getFloatParam(fpRepeats), 2.f, 65.f));
        if (repeats == 65)
            repeats = INT_MAX;

        float loopLength = std::max(
            FIRipol, this->note_to_pitch_ignoring_tuning(12 * this->getFloatParam(fpLoopLength)) *
                         this->getSampleRate());

        for (int k = 0; k < VFXConfig::blockSize; k++)
        {
            float current = datain[k];

            if (REC[0])
            {
                line->write(current);
                REC[0] = recorded[0]++ <= 1 << recSize;
            }

            if (LOOP[0])
            {
                if (repeated[0] > 0)
                {
                    current = line->readNaivelyAt(bufpos[0]);
                }

                if (bufpos[0]++ > loopLength)
                {
                    if (zcf[0].checkSample(current))
                    {
                        bufpos[0] = 0;
                        if (++repeated[0] >= repeats)
                        {
                            LOOP[0] = false;
                            REC[0] = false;
                            ARM = false;
                        }
                    }
                }
                else
                {
                    zcf[0].logSample(current);
                }
            }
            else
            {
                if (ARM)
                {
                    if (zcf[0].checkSampleAndSetDirection(current))
                    {
                        bufpos[0] = 0;
                        REC[0] = true;
                        LOOP[0] = true;
                    }
                }
                current *= thruLev[k];
            }
            dataout[k] = current;
        }
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        lineSupport[0].dispatch(recSize, [&](auto N) {
            auto *line0 = lineSupport[0].template getLinePointer<N>();
            auto *line1 = lineSupport[1].template getLinePointer<N>();
            std::array<decltype(line0), 2> lines = {line0, line1};
            stereoImpl(lines, datainL, datainR, dataoutL, dataoutR);
        });
    }

    void processMonoToMono(const float *const datain, float *dataout, float pitch)
    {
        lineSupport[0].dispatch(recSize, [&](auto N) {
            auto *line = lineSupport[1].template getLinePointer<N>();
            monoImpl(line, datain, dataout);
        });
    }

    size_t silentSamplesLength() const { return 10; }

  protected:
    float srComp{1.f};
    uint16_t repeated[2]{1, 1};
    int bufpos[2]{0, 0}, buflength[2]{0, 0};
    bool ARM{false};
    bool REC[2]{false, false};
    bool LOOP[2]{false, false};
    int recorded[2]{0, 0};
    int recSize{0};
    int BLOCK{0};

    std::array<details::DelayLineSupport<sst::basic_blocks::dsp::SSESincDelayLine>, 2> lineSupport;

    float FIRipol = static_cast<float>(SincTable::FIRipol_N);

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> thruLevLerp;

    struct zeroCrossingFinder
    {
        void setSRComp(float s) { srCo = (int)s; }
        void newRun() { safety = 5 * srCo; }

        bool checkSampleAndSetDirection(const float sample)
        {
            if (safety-- > 0)
            {
                prior = sample;
                return false;
            }

            if (std::signbit(sample * prior))
            {
                dir = sample > prior;
                prior = sample;
                return true;
            }
            prior = sample;
            return false;
        }

        bool checkSample(const float sample)
        {
            bool res = std::signbit(sample * prior) && sample > prior == dir;
            prior = sample;
            return res;
        }

        void logSample(const float sample) { prior = sample; }

      private:
        int srCo{5};
        int safety{5};
        float prior{0.f};
        bool dir{false};
    };
    std::array<zeroCrossingFinder, 2> zcf;

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
} // namespace sst::voice_effects::delay

#endif