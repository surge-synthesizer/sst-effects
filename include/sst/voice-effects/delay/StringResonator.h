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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DELAY_STRINGRESONATOR_H
#define INCLUDE_SST_VOICE_EFFECTS_DELAY_STRINGRESONATOR_H

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
template <typename VFXConfig> struct StringResonator : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"String Exciter"};

    static constexpr int numFloatParams{3};
    static constexpr int numIntParams{0};

    static constexpr float maxMiliseconds{100.f}; // 10 hz floor

    using SincTable = sst::basic_blocks::tables::SurgeSincTableProvider;
    const SincTable &sSincTable;

    static constexpr int shortLineSize{14};
    static constexpr int longLineSize{16};

    enum FloatParams
    {
        fpOffset,
        fpDecay,
        fpStiffness,
    };

    StringResonator(const SincTable &st)
        : sSincTable(st), core::VoiceEffectTemplateBase<VFXConfig>(), lp(this), hp(this)
    {
        std::fill(mLastParam.begin(), mLastParam.end(), -188888.f);
    }

    ~StringResonator()
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
        switch (idx)
        {
        case fpOffset:
            return pmd()
                .asFloat()
                .withRange(-48, 48)
                .withDefault(0)
                .withLinearScaleFormatting("semitones")
                .withName("Offset");

        case fpDecay:
            return pmd()
                .asFloat()
                .withRange(0.f, 1.f)
                .withDefault(0.8f)
                .withLinearScaleFormatting("")
                .withName("Decay");
        case fpStiffness:
            return pmd().asPercentBipolar().withDefault(0.f).withName("Stiffness");
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
                lineSupport[i].template preReserveLines<longLineSize>(this);
                lineSupport[i].template prepareLine<longLineSize>(this, sSincTable);
            }
        }
        else
        {
            isShort = true;
            for (int i = 0; i < 2; ++i)
            {
                lineSupport[i].template preReserveLines<shortLineSize>(this);
                lineSupport[i].template prepareLine<shortLineSize>(this, sSincTable);
            }
        }
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

        auto pt = pitch + this->getFloatParam(fpOffset);
        pt += pitchAdjustmentForStiffness();
        setupFilters(pt);

        lipolPitch.set_target(this->getSampleRate() /
                              (440 * this->note_to_pitch_ignoring_tuning(pt)));
        auto dcv = std::clamp(this->getFloatParam(fpDecay), 0.f, 1.f) * 0.12 + 0.88;
        dcv = std::min(sqrt(dcv), 0.99999);
        lipolDecay.set_target(dcv);
        if (firstPitch)
        {
            lipolPitch.instantize();
            lipolDecay.instantize();
            firstPitch = false;
        }
        float dt alignas(16)[VFXConfig::blockSize];
        float dc alignas(16)[VFXConfig::blockSize];
        lipolPitch.store_block(dt);
        lipolDecay.store_block(dc);

        float tone = this->getFloatParam(fpStiffness);

        for (int i = 0; i < VFXConfig::blockSize; ++i)
        {
            auto dl = lines[0]->read(dt[i]);
            auto dr = lines[1]->read(dt[i]);

            auto wl = datainL[i] + dc[i] * dl;
            auto wr = datainR[i] + dc[i] * dr;

            if (tone < 0)
            {
                lp.process_sample(wl, wr, wl, wr);
            }
            else if (tone > 0)
            {
                hp.process_sample(wl, wr, wl, wr);
            }

            lines[0]->write(wl);
            lines[1]->write(wr);

            dataoutL[i] = dl;
            dataoutR[i] = dr;
        }
    }

    void setupFilters(float pitch)
    {
        // This is the surge corrected tuning tone control
        pitch += 69;
        float clo = 10, cmidhi = 60, cmid = 100, chi = -70;
        float hpCutoff = chi;
        float lpCutoff = cmid;
        // If you change this you also need to recallibrate the tuning corrections!
        auto tone = this->getFloatParam(fpStiffness);
        if (tone > 0)
        {
            // We want a smaller range but chi stays the same so cmidhi - chi = 60 + 70 = 130
            // and we want about 60% to knock us out so
            cmidhi = 10, chi = -70;
            // OK so cool scale the HP cutoff
            auto tv = tone;
            hpCutoff = tv * (cmidhi - chi) + chi + pitch - 60;
        }
        else
        {
            auto tv = -tone;
            lpCutoff = pitch - 40 * tv;
        }
        // Inefficient - copy coefficients later
        lp.coeff_LP(lp.calc_omega((lpCutoff / 12.0)), 0.707);
        hp.coeff_HP(hp.calc_omega((hpCutoff / 12.0)), 0.707);
    }

    float pitchAdjustmentForStiffness()
    {

        auto tv = this->getFloatParam(fpStiffness);
        if (tv == 0)
            return 0.f;

        if (tv < 0)
        {
            // I just whacked at it in a tuner at levels and came up with this. These are pitch
            // shifts so basically i ran A/69/440 into a tuner with the burst chirp and saw how far
            // we were off in frequency at 0, 25, 50 etc... then converted to notes using 12TET
            static constexpr float retunes[] = {-0.0591202, -0.122405, -0.225738, -0.406056,
                                                -0.7590243};
            float fidx = std::clamp(-4 * tv, 0.f, 4.f);
            int idx = std::clamp((int)fidx, 0, 3);
            float frac = fidx - idx;

            auto res = retunes[idx] * (1 - frac) + retunes[idx + 1] * frac;

            return -res;
        }
        else
        {
            // I just whacked at it in a tuner at levels and came up with this
            static constexpr float retunes[] = {0.02752047, 0.09026062, 0.31, 0.615, 0.87};
            float fidx = std::clamp(4 * tv, 0.f, 4.f);
            int idx = std::clamp((int)fidx, 0, 3);
            float frac = fidx - idx;

            auto res = retunes[idx] * (1 - frac) + retunes[idx + 1] * frac;

            return -res;
        }
        return 0;
    }
    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        if (isShort)
        {
            processImpl(std::array{lineSupport[0].template getLinePointer<shortLineSize>(),
                                   lineSupport[1].template getLinePointer<shortLineSize>()},
                        datainL, datainR, dataoutL, dataoutR, pitch);
        }
        else
        {
            processImpl(std::array{lineSupport[0].template getLinePointer<longLineSize>(),
                                   lineSupport[1].template getLinePointer<longLineSize>()},
                        datainL, datainR, dataoutL, dataoutR, pitch);
        }
    }

  protected:
    std::array<details::DelayLineSupport, 2> lineSupport;
    bool isShort{true};
    bool firstPitch{false};

    std::array<float, numFloatParams> mLastParam{};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> lipolPitch, lipolDecay;

    typename core::VoiceEffectTemplateBase<VFXConfig>::BiquadFilterType lp, hp;
};

} // namespace sst::voice_effects::delay

#endif // SHORTCIRCUITXT_EqNBandParametric_H
