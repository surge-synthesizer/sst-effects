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

#ifndef INCLUDE_SST_EFFECTS_SHARED_TREEMONSTERCORE_H
#define INCLUDE_SST_EFFECTS_SHARED_TREEMONSTERCORE_H

#include <utility>
#include <cassert>

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/filters/BiquadFilter.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"

namespace sst::effects_shared
{
template <typename T> struct FXConfigAdapter
{
    using config_t = typename T::FXConfig_t;
};

namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename BaseClass, typename BiquadType, bool addHPLP = true>
struct TreemonsterCore : public BaseClass
{
    using FXConfig = typename FXConfigAdapter<BaseClass>::config_t;

    enum tm_params
    {
        tm_threshold = 0,
        tm_speed,
        tm_hp,
        tm_lp,

        tm_pitch,
        tm_ring_mix,

        tm_width,
        tm_mix,

        tm_num_ctrls,
    };

    template <typename... Args>
    TreemonsterCore(Args &&...args) : BaseClass(std::forward<Args>(args)...)
    {
        rm.set_blocksize(FXConfig::blockSize);
        width.set_blocksize(FXConfig::blockSize);
        mix.set_blocksize(FXConfig::blockSize);
    }

    void processWithoutMixOrWith(float *dataL, float *dataR, float *wetL, float *wetR);

    void processWithMixAndWidth(float *dataL, float *dataR)
    {
        this->setWidthTarget(width, tm_width);
        mix.set_target_smoothed(this->floatValue(tm_mix));

        float wetL alignas(16)[FXConfig::blockSize], wetR alignas(16)[FXConfig::blockSize];

        processWithoutMixOrWith(dataL, dataR, wetL, wetR);

        this->applyWidth(wetL, wetR, width);
        mix.fade_2_blocks_inplace(dataL, wetL, dataR, wetR);
    }

    float L alignas(16)[FXConfig::blockSize], R alignas(16)[FXConfig::blockSize];

    // These are outputs which you can optionally grab from outside
    // the main processing loop. The Rack module does this.
    float smoothedPitch[2][FXConfig::blockSize], envelopeOut[2][FXConfig::blockSize];

    sdsp::lipol_sse<FXConfig::blockSize, false> rm alignas(16), width alignas(16), mix alignas(16);

    using quadr_osc = sst::basic_blocks::dsp::SurgeQuadrOsc<float>;
    quadr_osc oscL alignas(16), oscR alignas(16);

    void initialize()
    {
        if constexpr (addHPLP)
        {
            assert(hp.storage);
            assert(lp.storage);
        }
        setvars(true);
        bi = 0;
    }
    void setvars(bool init);

  protected:
    int bi; // block increment (to keep track of events not occurring every n blocks)
    float length[2], lastval[2], length_target[2], length_smooth[2];
    bool first_thresh[2];
    BiquadType lp, hp;

    double envA, envR;
    float envV[2];
};

template <typename BaseClass, typename BiquadType, bool addHPLP>
inline void TreemonsterCore<BaseClass, BiquadType, addHPLP>::setvars(bool init)
{
    if (init)
    {
        if constexpr (addHPLP)
        {
            lp.suspend();
            hp.suspend();

            auto hpv = this->floatValue(tm_hp);
            hp.coeff_HP(hp.calc_omega(hpv / 12.0), 0.707);
            hp.coeff_instantize();

            auto lpv = this->floatValue(tm_lp);
            lp.coeff_LP2B(lp.calc_omega(lpv / 12.0), 0.707);
            lp.coeff_instantize();
        }

        oscL.set_rate(0.f);
        oscR.set_rate(0.f);

        rm.set_target(1.f);
        width.set_target(0.f);
        mix.set_target(1.f);

        rm.instantize();
        width.instantize();
        mix.instantize();

        // envelope follower times: 5 ms attack, 500 ms release
        envA = pow(0.01, 1.0 / (5 * this->sampleRate() * 0.001));
        envR = pow(0.01, 1.0 / (500 * this->sampleRate() * 0.001));
        envV[0] = 0.f;
        envV[1] = 0.f;

        length[0] = 100;
        length[1] = 100;
        length_target[0] = 100;
        length_target[1] = 100;
        length_smooth[0] = 100;
        length_smooth[1] = 100;
        first_thresh[0] = true;
        first_thresh[1] = true;
        oscL.set_phase(0);
        oscR.set_phase(M_PI / 2.0);
    }
}
template <typename BaseClass, typename BiquadType, bool addHPLP>
void TreemonsterCore<BaseClass, BiquadType, addHPLP>::processWithoutMixOrWith(float *dataL,
                                                                              float *dataR,
                                                                              float *L, float *R)
{
    static constexpr double MIDI_0_FREQ = 8.17579891564371; // or 440.0 * pow( 2.0, - (69.0/12.0 ) )

    float tbuf alignas(16)[2][FXConfig::blockSize];
    float envscaledSineWave alignas(16)[2][FXConfig::blockSize];

    auto thres = this->dbToLinear(std::clamp(this->floatValue(tm_threshold), -96.f, 0.f));

    // copy dry signal (dataL, dataR) to wet signal (L, R)
    mech::copy_from_to<FXConfig::blockSize>(dataL, L);
    mech::copy_from_to<FXConfig::blockSize>(dataR, R);

    // copy it to pitch detection buffer (tbuf) as well
    // in case filters are not activated
    mech::copy_from_to<FXConfig::blockSize>(dataL, tbuf[0]);
    mech::copy_from_to<FXConfig::blockSize>(dataR, tbuf[1]);

    if constexpr (addHPLP)
    {
        // apply filters to the pitch detection buffer
        if (!this->isDeactivated(tm_hp))
        {
            hp.coeff_HP(hp.calc_omega(this->floatValue(tm_hp) / 12.0), 0.707);
            hp.process_block(tbuf[0], tbuf[1]);
        }

        if (!this->isDeactivated(tm_lp))
        {
            lp.coeff_LP2B(lp.calc_omega(this->floatValue(tm_lp) / 12.0), 0.707);
            lp.process_block(tbuf[0], tbuf[1]);
        }
    }

    /*
     * We assume wavelengths below this are just noisy detection errors. This is used to
     * clamp when we have a pitch detect basically.
     */
    constexpr float smallest_wavelength = 16.0;

    float qs = std::clamp(this->floatValue(tm_speed), 0.f, 1.f);
    qs *= qs * qs * qs;
    float speed = 0.9999 - qs * 0.0999 / 128;
    float numberOfSteps = FXConfig::blockSize * 48000 / this->sampleRate();

    float lsCache[2];
    lsCache[0] = length_smooth[0];
    lsCache[1] = length_smooth[1];

    for (int i = 0; i < numberOfSteps; ++i)
    {
        length_smooth[0] = speed * length_smooth[0] + (1 - speed) * length_target[0];
        length_smooth[1] = speed * length_smooth[1] + (1 - speed) * length_target[1];
    }

    for (int c = 0; c < 2; ++c)
    {
        auto l2c = log2(this->sampleRate() / std::max(lsCache[c], 2.f) / MIDI_0_FREQ);
        auto l2s = log2(this->sampleRate() / std::max(length_smooth[c], 2.f) / MIDI_0_FREQ);
        auto dl = (l2s - l2c) / FXConfig::blockSize;

        for (auto k = 0; k < FXConfig::blockSize; ++k)
        {
            smoothedPitch[c][k] = l2c + dl * k;
        }
    }

    auto twoToPitch = powf(2.0, this->floatValue(tm_pitch) * (1 / 12.f));
    oscL.set_rate((2.0 * M_PI / std::max(2.f, length_smooth[0])) * twoToPitch);
    oscR.set_rate((2.0 * M_PI / std::max(2.f, length_smooth[1])) * twoToPitch);

    for (int k = 0; k < FXConfig::blockSize; k++)
    {
        // envelope detection
        for (int c = 0; c < 2; ++c)
        {
            auto v = (c == 0 ? dataL[k] : dataR[k]);
            auto e = envV[c];

            if (v > e)
            {
                e = envA * (e - v) + v;
            }
            else
            {
                e = envR * (e - v) + v;
            }

            envV[c] = e;
            envelopeOut[c][k] = e;
        }

        // pitch detection
        if ((lastval[0] < 0.f) && (tbuf[0][k] >= 0.f))
        {
            if (tbuf[0][k] > thres && length[0] > smallest_wavelength)
            {
                length_target[0] =
                    (length[0] > length_smooth[0] * 10 ? length_smooth[0] : length[0]);
                if (first_thresh[0])
                    length_smooth[0] = length[0];
                first_thresh[0] = false;
            }

            length[0] = 0.0; // (0.0-lastval[0]) / ( tbuf[0][k] - lastval[0]);
        }

        if ((lastval[1] < 0.f) && (tbuf[1][k] >= 0.f))
        {
            if (tbuf[1][k] > thres && length[1] > smallest_wavelength)
            {
                length_target[1] =
                    (length[1] > length_smooth[1] * 10 ? length_smooth[1] : length[1]);
                if (first_thresh[1])
                    length_smooth[1] = length[1];
                first_thresh[1] = false;
            }

            length[1] = 0.0; // (0.0-lastval[1]) / ( tbuf[1][k] - lastval[1]);
        }

        oscL.process();
        oscR.process();

        // do not apply followed envelope to sine oscillator - we need full freight sine for RM
        L[k] = oscL.r;
        R[k] = oscR.r;

        // but we need to store the scaled for mix
        envscaledSineWave[0][k] = oscL.r * envV[0];
        envscaledSineWave[1][k] = oscR.r * envV[0];

        // track positive zero crossings
        length[0] += 1.0f;
        length[1] += 1.0f;

        lastval[0] = tbuf[0][k];
        lastval[1] = tbuf[1][k];
    }

    // do dry signal * pitch tracked signal ringmod
    // store to pitch detection buffer
    mech::mul_block<FXConfig::blockSize>(L, dataL, tbuf[0]);
    mech::mul_block<FXConfig::blockSize>(R, dataR, tbuf[1]);

    // mix pure pitch tracked sine with ring modulated signal
    rm.set_target_smoothed(std::clamp(this->floatValue(tm_ring_mix), 0.f, 1.f));
    rm.fade_2_blocks_to(envscaledSineWave[0], tbuf[0], envscaledSineWave[1], tbuf[1], L, R);
}

} // namespace sst::effects_shared
#endif // SURGE_TREEMONSTERCORE_H
