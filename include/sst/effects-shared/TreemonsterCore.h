//
// Created by Paul Walker on 7/9/24.
//

#ifndef SURGE_TREEMONSTERCORE_H
#define SURGE_TREEMONSTERCORE_H

#include <utility>

namespace sst::effects_shared
{
template<typename BaseClass>
struct TreemonsterCore : BaseClass
{
    using FXConfig = typename BaseClass::FXConfig_t;

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

    template <typename... Args> TreemonsterCore(Args &&...args)
    : BaseClass(std::forward<Args>(args)...)
          // TODO: , lp(s), hp(s)
    {}

    // These are outputs which you can optionally grab from outside
    // the main processing loop. The Rack module does this.
    float smoothedPitch[2][FXConfig::blockSize], envelopeOut[2][FXConfig::blockSize];

  private:
    int bi; // block increment (to keep track of events not occurring every n blocks)
    float length[2], lastval[2], length_target[2], length_smooth[2];
    bool first_thresh[2];
    typename FXConfig::BiquadFilter lp, hp;

    double envA, envR;
    float envV[2];
};
}
#endif // SURGE_TREEMONSTERCORE_H
