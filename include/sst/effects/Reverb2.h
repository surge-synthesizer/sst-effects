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
#ifndef INCLUDE_SST_EFFECTS_REVERB2_H
#define INCLUDE_SST_EFFECTS_REVERB2_H

#include <cstring>
#include "EffectCore.h"
#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/Lag.h"
#include "sst/basic-blocks/dsp/BlockInterpolators.h"
#include "sst/basic-blocks/dsp/QuadratureOscillators.h"
#include "sst/filters/BiquadFilter.h"
#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/mechanics/simd-ops.h"

namespace sst::effects::reverb2
{
namespace sdsp = sst::basic_blocks::dsp;
namespace mech = sst::basic_blocks::mechanics;

template <typename FXConfig> struct Reverb2 : core::EffectTemplateBase<FXConfig>
{
    enum rev2_params
    {
        rev2_predelay = 0,

        rev2_room_size,
        rev2_decay_time,
        rev2_diffusion,
        rev2_buildup,
        rev2_modulation,

        rev2_lf_damping,
        rev2_hf_damping,

        rev2_width,
        rev2_mix,

        rev2_num_params,
    };

    static constexpr int numParams{rev2_mix + 1};
    static constexpr const char *effectName{"reverb2"};

    static constexpr int NUM_BLOCKS = 4;
    static constexpr int NUM_INPUT_ALLPASSES = 4;
    static constexpr int NUM_ALLPASSES_PER_BLOCK = 2;
    static constexpr int MAX_ALLPASS_LEN = 16384 * 8;
    static constexpr int MAX_DELAY_LEN = 16384 * 8;
    static constexpr int DELAY_LEN_MASK = MAX_DELAY_LEN - 1;
    static constexpr int DELAY_SUBSAMPLE_BITS = 8;
    static constexpr int DELAY_SUBSAMPLE_RANGE = (1 << DELAY_SUBSAMPLE_BITS);
    static constexpr int PREDELAY_BUFFER_SIZE =
        48000 * 8 * 4; // max sample rate is 48000 * 8 probably
    static constexpr int PREDELAY_BUFFER_SIZE_LIMIT =
        48000 * 8 * 3; // allow for one second of diffusion

    class allpass
    {
      public:
        allpass();
        float process(float x, float coeff);
        void setLen(int len);

      private:
        int _len;
        int _k;
        float _data[MAX_ALLPASS_LEN];
    };

    class delay
    {
      public:
        delay();
        float process(float x, int tap1, float &tap_out1, int tap2, float &tap_out2,
                      int modulation);
        void setLen(int len);

      private:
        int _len;
        int _k;
        float _data[MAX_DELAY_LEN];
    };

    class predelay
    {
      public:
        predelay() { memset(_data, 0, PREDELAY_BUFFER_SIZE * sizeof(float)); }
        float process(float in, int tap)
        {
            k = (k + 1);
            if (k == PREDELAY_BUFFER_SIZE)
                k = 0;
            auto p = k - tap;
            while (p < 0)
                p += PREDELAY_BUFFER_SIZE;
            auto res = _data[p];
            _data[k] = in;
            return res;
        }

      private:
        int k = 0;
        float _data[PREDELAY_BUFFER_SIZE];
    };

    class onepole_filter
    {
      public:
        onepole_filter();
        float process_lowpass(float x, float c0);
        float process_highpass(float x, float c0);

      private:
        float a0;
    };

    Reverb2(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
            typename FXConfig::ValueStorage *p);

    void initialize() { setvars(true); }
    void processBlock(float *__restrict L, float *__restrict R);

    void suspendProcessing() { initialize(); }
    int getRingoutDecay() const { return ringout_time; }
    void onSampleRateChanged() { initialize(); }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;
        switch ((rev2_params)idx)
        {
        case rev2_predelay:
            return pmd()
                .asFloat()
                .withName("Pre-Delay")
                .withRange(-8.f, 1.f)
                .withDefault(-4.f)
                .withATwoToTheBFormatting(1.f, 1.f, "s")
                .withCustomMinDisplay("0s")
                .withMilisecondsBelowOneSecond()
                .temposyncable()
                .withTemposyncMultiplier(1.f);

        case rev2_decay_time:
            return pmd()
                .asFloat()
                .withName("Decay Time")
                .withRange(-4.f, 6.f)
                .withDefault(0.75f)
                .withATwoToTheBFormatting(1.f, 1.f, "s")
                .withCustomMinDisplay("0s")
                .withMilisecondsBelowOneSecond();

        case rev2_room_size:
            return pmd().asPercentBipolar().withName("Room Size").withDefault(0.f);

        case rev2_diffusion:
            return pmd().asPercent().withName("Diffusion").withDefault(1.f);

        case rev2_buildup:
            return pmd().asPercent().withName("Buildup").withDefault(1.f);

        case rev2_modulation:
            return pmd().asPercent().withName("Modulation").withDefault(0.5f);

        case rev2_hf_damping:
            return pmd().asPercent().withName("HF Damping").withDefault(0.2f);

        case rev2_lf_damping:
            return pmd().asPercent().withName("LF Damping").withDefault(0.2f);

        case rev2_mix:
            return pmd().asPercent().withName("Mix").withDefault(0.33f);

        case rev2_width:
            return pmd().withName("Width").asDecibelNarrow().withDefault(0.f);

        default:
            return pmd().asPercentBipolar().withName("ERROR " + std::to_string(idx));
        }
    }

  protected:
    void update_rtime();
    void setvars(bool);
    void calc_size(float);

    int ringout_time;
    allpass _input_allpass[NUM_INPUT_ALLPASSES];
    allpass _allpass[NUM_BLOCKS][NUM_ALLPASSES_PER_BLOCK];
    onepole_filter _hf_damper[NUM_BLOCKS];
    onepole_filter _lf_damper[NUM_BLOCKS];
    delay _delay[NUM_BLOCKS];
    predelay _predelay;
    int _tap_timeL[NUM_BLOCKS];
    int _tap_timeR[NUM_BLOCKS];
    float _tap_gainL[NUM_BLOCKS];
    float _tap_gainR[NUM_BLOCKS];
    float _state;
    sdsp::lipol<float, FXConfig::blockSize, true> _decay_multiply;
    sdsp::lipol<float, FXConfig::blockSize, true> _diffusion;
    sdsp::lipol<float, FXConfig::blockSize, true> _buildup;
    sdsp::lipol<float, FXConfig::blockSize, true> _hf_damp_coefficent;
    sdsp::lipol<float, FXConfig::blockSize, true> _lf_damp_coefficent;
    sdsp::lipol<float, FXConfig::blockSize, true> _modulation;

    using quadr_osc = sst::basic_blocks::dsp::SurgeQuadrOsc<float>;
    quadr_osc _lfo;
    float last_decay_time = -1.0;

    sdsp::lipol_sse<FXConfig::blockSize, false> width, mix;

    inline int msToSamples(float ms, float scale, float samplerate)
    {
        float a = samplerate * ms * 0.001f;
        float b = a * scale;
        // these are clamped out above
        // assert( b < Reverb2Effect::MAX_ALLPASS_LEN);
        // assert( b < Reverb2Effect::MAX_DELAY_LEN);
        return (int)(b);
    }

    static constexpr double db60{0.001}; // powf(10.f, 0.05f * -60.f);
};

template <typename FXConfig>
Reverb2<FXConfig>::Reverb2(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                           typename FXConfig::ValueStorage *p)
    : core::EffectTemplateBase<FXConfig>(s, e, p)
{
    _state = 0.f;
}

template <typename FXConfig> Reverb2<FXConfig>::allpass::allpass()
{
    _k = 0;
    _len = 1;
    memset(_data, 0, MAX_ALLPASS_LEN * sizeof(float));
}

template <typename FXConfig> void Reverb2<FXConfig>::allpass::setLen(int len)
{
    _len = std::clamp(len, 0, MAX_ALLPASS_LEN - 1);
}

template <typename FXConfig> float Reverb2<FXConfig>::allpass::process(float in, float coeff)
{
    _k++;
    if (_k >= _len)
        _k = 0;
    float delay_in = in - coeff * _data[_k];
    float result = _data[_k] + coeff * delay_in;
    _data[_k] = delay_in;
    return result;
}

template <typename FXConfig> Reverb2<FXConfig>::delay::delay()
{
    _k = 0;
    _len = 1;
    memset(_data, 0, MAX_DELAY_LEN * sizeof(float));
}

template <typename FXConfig> void Reverb2<FXConfig>::delay::setLen(int len)
{
    _len = std::clamp(len, 0, MAX_DELAY_LEN - 1);
}

template <typename FXConfig>
float Reverb2<FXConfig>::delay::process(float in, int tap1, float &tap_out1, int tap2,
                                        float &tap_out2, int modulation)
{
    _k = (_k + 1) & DELAY_LEN_MASK;

    tap_out1 = _data[(_k - tap1) & DELAY_LEN_MASK];
    tap_out2 = _data[(_k - tap2) & DELAY_LEN_MASK];

    int modulation_int = modulation >> DELAY_SUBSAMPLE_BITS;
    int modulation_frac1 = modulation & (DELAY_SUBSAMPLE_RANGE - 1);
    int modulation_frac2 = DELAY_SUBSAMPLE_RANGE - modulation_frac1;

    float d1 = _data[(_k - _len + modulation_int + 1) & DELAY_LEN_MASK];
    float d2 = _data[(_k - _len + modulation_int) & DELAY_LEN_MASK];
    const float multiplier = 1.f / (float)(DELAY_SUBSAMPLE_RANGE);

    float result = (d1 * (float)modulation_frac1 + d2 * (float)modulation_frac2) * multiplier;
    _data[_k] = in;

    return result;
}

template <typename FXConfig> Reverb2<FXConfig>::onepole_filter::onepole_filter() { a0 = 0.f; }

template <typename FXConfig>
float Reverb2<FXConfig>::onepole_filter::process_lowpass(float x, float c0)
{
    a0 = a0 * c0 + x * (1.f - c0);
    return a0;
}

template <typename FXConfig>
float Reverb2<FXConfig>::onepole_filter::process_highpass(float x, float c0)
{
    a0 = a0 * (1.f - c0) + x * c0;
    return x - a0;
}

template <typename FXConfig> void Reverb2<FXConfig>::update_rtime()
{
    auto ts = this->temposyncRatio(rev2_predelay);
    // * 2.f is to get the dB120 time
    auto pdlyt = std::max(0.1f, powf(2.f, this->floatValue(rev2_predelay)) * ts) * 2.f;
    auto dcyt = std::max(1.0f, powf(2.f, this->floatValue(rev2_decay_time))) * 2.f;
    float t =  (this->sampleRate() * (dcyt + pdlyt)) / FXConfig::blockSize;

    ringout_time = (int)t;
}

template <typename FXConfig> void Reverb2<FXConfig>::calc_size(float scale)
{
    float m = scale;

    auto sr = this->sampleRate();
    _tap_timeL[0] = msToSamples(80.3, m, sr);
    _tap_timeL[1] = msToSamples(59.3, m, sr);
    _tap_timeL[2] = msToSamples(97.7, m, sr);
    _tap_timeL[3] = msToSamples(122.6, m, sr);
    _tap_timeR[0] = msToSamples(35.5, m, sr);
    _tap_timeR[1] = msToSamples(101.6, m, sr);
    _tap_timeR[2] = msToSamples(73.9, m, sr);
    _tap_timeR[3] = msToSamples(80.3, m, sr);

    _input_allpass[0].setLen(msToSamples(4.76, m, sr));
    _input_allpass[1].setLen(msToSamples(6.81, m, sr));
    _input_allpass[2].setLen(msToSamples(10.13, m, sr));
    _input_allpass[3].setLen(msToSamples(16.72, m, sr));

    _allpass[0][0].setLen(msToSamples(38.2, m, sr));
    _allpass[0][1].setLen(msToSamples(53.4, m, sr));
    _delay[0].setLen(msToSamples(178.8, m, sr));

    _allpass[1][0].setLen(msToSamples(44.0, m, sr));
    _allpass[1][1].setLen(msToSamples(41, m, sr));
    _delay[1].setLen(msToSamples(126.5, m, sr));

    _allpass[2][0].setLen(msToSamples(48.3, m, sr));
    _allpass[2][1].setLen(msToSamples(60.5, m, sr));
    _delay[2].setLen(msToSamples(106.1, m, sr));

    _allpass[3][0].setLen(msToSamples(38.9, m, sr));
    _allpass[3][1].setLen(msToSamples(42.2, m, sr));
    _delay[3].setLen(msToSamples(139.4, m, sr));
}

template <typename FXConfig> void Reverb2<FXConfig>::setvars(bool init)
{
    // TODO: balance the gains from the calculated decay coefficient?
    _tap_gainL[0] = 1.5f / 4.f;
    _tap_gainL[1] = 1.2f / 4.f;
    _tap_gainL[2] = 1.0f / 4.f;
    _tap_gainL[3] = 0.8f / 4.f;
    _tap_gainR[0] = 1.5f / 4.f;
    _tap_gainR[1] = 1.2f / 4.f;
    _tap_gainR[2] = 1.0f / 4.f;
    _tap_gainR[3] = 0.8f / 4.f;

    calc_size(1.f);
}

template <typename FXConfig> void Reverb2<FXConfig>::processBlock(float *dataL, float *dataR)
{
    float scale = powf(2.f, 1.f * this->floatValue(rev2_room_size));
    calc_size(scale);

    if (fabs(this->floatValue(rev2_decay_time) - last_decay_time) > 0.001f)
        update_rtime();

    last_decay_time = this->floatValue(rev2_decay_time);

    float wetL alignas(16)[FXConfig::blockSize], wetR alignas(16)[FXConfig::blockSize];

    float loop_time_s = 0.5508 * scale;
    float decay = powf(db60, loop_time_s / (4.f * (powf(2.f, this->floatValue(rev2_decay_time)))));

    _decay_multiply.newValue(decay);
    _diffusion.newValue(0.7f * this->floatValue(rev2_diffusion));
    _buildup.newValue(0.7f * this->floatValue(rev2_buildup));
    _hf_damp_coefficent.newValue(0.8 * this->floatValue(rev2_hf_damping));
    _lf_damp_coefficent.newValue(0.2 * this->floatValue(rev2_lf_damping));
    _modulation.newValue(this->floatValue(rev2_modulation) * this->sampleRate() * 0.001f * 5.f);

    width.set_target_smoothed(this->dbToLinear(this->floatValue(rev2_width)));
    mix.set_target_smoothed(this->floatValue(rev2_mix));

    _lfo.set_rate(2.0 * M_PI * powf(2, -2.f) / this->sampleRate());

    int pdt = std::clamp((int)(this->sampleRate() * pow(2.f, this->floatValue(rev2_predelay)) *
                                this->temposyncRatioInv(rev2_predelay)),
                          1, PREDELAY_BUFFER_SIZE_LIMIT - 1);

    for (int k = 0; k < FXConfig::blockSize; k++)
    {
        float in = (dataL[k] + dataR[k]) * 0.5f;

        in = _predelay.process(in, pdt);

        in = _input_allpass[0].process(in, _diffusion.v);
        in = _input_allpass[1].process(in, _diffusion.v);
        in = _input_allpass[2].process(in, _diffusion.v);
        in = _input_allpass[3].process(in, _diffusion.v);
        float x = _state;

        float outL = 0.f;
        float outR = 0.f;

        float lfos[NUM_BLOCKS];
        lfos[0] = _lfo.r;
        lfos[1] = _lfo.i;
        lfos[2] = -_lfo.r;
        lfos[3] = -_lfo.i;

        auto hdc = std::clamp(_hf_damp_coefficent.v, 0.01f, 0.99f);
        auto ldc = std::clamp(_lf_damp_coefficent.v, 0.01f, 0.99f);
        for (int b = 0; b < NUM_BLOCKS; b++)
        {
            x = x + in;
            for (int c = 0; c < NUM_ALLPASSES_PER_BLOCK; c++)
            {
                x = _allpass[b][c].process(x, _buildup.v);
            }

            x = _hf_damper[b].process_lowpass(x, hdc);
            x = _lf_damper[b].process_highpass(x, ldc);

            int modulation = (int)(_modulation.v * lfos[b] * (float)DELAY_SUBSAMPLE_RANGE);
            float tap_outL = 0.f;
            float tap_outR = 0.f;
            x = _delay[b].process(x, _tap_timeL[b], tap_outL, _tap_timeR[b], tap_outR, modulation);
            outL += tap_outL * _tap_gainL[b];
            outR += tap_outR * _tap_gainR[b];

            x *= _decay_multiply.v;
        }

        wetL[k] = outL;
        wetR[k] = outR;
        _state = x;
        _decay_multiply.process();
        _diffusion.process();
        _buildup.process();
        _hf_damp_coefficent.process();
        _lfo.process();
        _modulation.process();
    }

    // scale width
    this->applyWidth(wetL, wetR, width);

    mix.fade_2_blocks_inplace(dataL, wetL, dataR, wetR);
}

} // namespace sst::effects::reverb2

#endif
