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

#ifndef INCLUDE_SST_VOICE_EFFECTS_DISTORTION_MICROGATE_H
#define INCLUDE_SST_VOICE_EFFECTS_DISTORTION_MICROGATE_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "../VoiceEffectCore.h"

#include "sst/basic-blocks/mechanics/block-ops.h"
namespace mech = sst::basic_blocks::mechanics;

namespace sst::voice_effects::distortion
{
template <typename VFXConfig> struct MicroGate : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"BitCrusher"};

    static constexpr int microgateBufferSize{4096};
    static constexpr int microgateBlockSize{microgateBufferSize * sizeof(float) * 2};

    enum mg_fparams
    {
        mg_hold,
        mg_loop,
        mg_threshold,
        mg_reduction,

        mg_num_params
    };

    static constexpr int numFloatParams{mg_num_params};
    static constexpr int numIntParams{0};

    MicroGate() : core::VoiceEffectTemplateBase<VFXConfig>()
    {
        this->preReservePool(microgateBlockSize);
    }

    ~MicroGate()
    {
        if (loopbuffer[0])
        {
            VFXConfig::returnBlock(this, (uint8_t *)loopbuffer[0], microgateBlockSize);
        }
    }

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        assert(idx >= 0 && idx < mg_num_params);
        using pmd = basic_blocks::params::ParamMetaData;

        switch ((mg_fparams)idx)
        {
        case mg_hold:
            return pmd().asLog2SecondsRange(-8, 5, -3).withName("hold");
        case mg_loop:
            return pmd().asPercent().withName("loop");
        case mg_threshold:
            return pmd().asLinearDecibel().withName("threshold");
        case mg_reduction:
            return pmd().asLinearDecibel().withName("reduction");
        case mg_num_params:
            break;
        }

        return pmd().withName("Unknown " + std::to_string(idx));
    }

    void initVoiceEffect()
    {
        assert(!loopbuffer[0]); // in theory we should never double init the same instance
        if (!loopbuffer[0])
        {
            auto data = VFXConfig::checkoutBlock(this, microgateBlockSize);
            memset(data, 0, microgateBlockSize);
            loopbuffer[0] = (float *)data;
            loopbuffer[1] = (float *)(data + microgateBufferSize * sizeof(float));
        }
    }

    void initVoiceEffectParams()
    {
        this->setFloatParam(0, -3.f);
        this->setFloatParam(1, 0.5f);
        this->setFloatParam(2, -12.f);
        this->setFloatParam(3, -96.f);
    }

    void processStereo(float *datainL, float *datainR, float *dataoutL, float *dataoutR,
                       float pitch)
    {
        constexpr int blockSize{VFXConfig::blockSize};
        mech::copy_from_to<blockSize>(datainL, dataoutL);
        mech::copy_from_to<blockSize>(datainR, dataoutR);

        // TODO fixme
        float threshold = this->dbToLinear(this->getFloatParam(2));
        float reduction = this->dbToLinear(this->getFloatParam(3));
        int ihtime = (int)(float)(this->getSampleRate() *
                                  this->equalNoteToPitch(12 * this->getFloatParam(0)));

        for (int k = 0; k < blockSize; k++)
        {
            float input = std::max(fabs(datainL[k]), fabs(datainR[k]));

            if ((input > threshold) && !gate_state)
            {
                holdtime = ihtime;
                gate_state = true;
                is_recording[0] = true;
                bufpos[0] = 0;
                is_recording[1] = true;
                bufpos[1] = 0;
            }

            if (holdtime < 0)
                gate_state = false;

            if ((!(onesampledelay[0] * datainL[k] > 0)) && (datainL[k] > 0))
            {
                gate_zc_sync[0] = gate_state;
                int looplimit =
                    (int)(float)(4 + 3900 * this->getFloatParam(1) * this->getFloatParam(1));
                if (bufpos[0] > looplimit)
                {
                    is_recording[0] = false;
                    buflength[0] = bufpos[0];
                }
            }
            if ((!(onesampledelay[1] * datainR[k] > 0)) && (datainR[k] > 0))
            {
                gate_zc_sync[1] = gate_state;
                int looplimit =
                    (int)(float)(4 + 3900 * this->getFloatParam(1) * this->getFloatParam(1));
                if (bufpos[1] > looplimit)
                {
                    is_recording[1] = false;
                    buflength[1] = bufpos[1];
                }
            }
            onesampledelay[0] = datainL[k];
            onesampledelay[1] = datainR[k];

            if (gate_zc_sync[0])
            {
                if (is_recording[0])
                {
                    loopbuffer[0][bufpos[0]++ & (microgateBufferSize - 1)] = datainL[k];
                }
                else
                {
                    dataoutL[k] = loopbuffer[0][bufpos[0] & (microgateBufferSize - 1)];
                    bufpos[0]++;
                    if (bufpos[0] >= buflength[0])
                        bufpos[0] = 0;
                }
            }
            else
                dataoutL[k] *= reduction;

            if (gate_zc_sync[1])
            {
                if (is_recording[1])
                {
                    loopbuffer[1][bufpos[1]++ & (microgateBufferSize - 1)] = datainR[k];
                }
                else
                {
                    dataoutR[k] = loopbuffer[1][bufpos[1] & (microgateBufferSize - 1)];
                    bufpos[1]++;
                    if (bufpos[1] >= buflength[1])
                        bufpos[1] = 0;
                }
            }
            else
                dataoutR[k] *= reduction;

            holdtime--;
        }
    }

  protected:
    int holdtime{0};
    bool gate_state{false}, gate_zc_sync[2]{false, false};
    float onesampledelay[2]{-1, -1};
    float *loopbuffer[2]{nullptr, nullptr};
    int bufpos[2]{0, 0}, buflength[2]{0, 0};
    bool is_recording[2]{false, false};
};
} // namespace sst::voice_effects::distortion

#endif