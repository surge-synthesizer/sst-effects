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

#ifndef INCLUDE_SST_EFFECTS_NIMBUSIMPL_H
#define INCLUDE_SST_EFFECTS_NIMBUSIMPL_H

#include "Nimbus.h"

#if SST_EFFECTS_EURORACK
#ifdef _MSC_VER
#define __attribute__(x)
#endif

#define TEST // remember this is how you tell the eurorack code to use dsp not hardware
#if EURORACK_CLOUDS_IS_SUPERPARASITES
#include "supercell/dsp/granular_processor.h"
#else
#include "clouds/dsp/granular_processor.h"
#endif

#undef TEST
#ifdef _MSC_VER
#undef __attribute__
#endif

namespace sst::effects::nimbus
{
template <typename FXConfig>
Nimbus<FXConfig>::Nimbus(typename FXConfig::GlobalStorage *s, typename FXConfig::EffectStorage *e,
                         typename FXConfig::ValueStorage *p)
    : core::EffectTemplateBase<FXConfig>(s, e, p)
{
    const int memLen = 118784;
    const int ccmLen = 65536 - 128;
    block_mem = new uint8_t[memLen]();
    block_ccm = new uint8_t[ccmLen]();
    processor = new clouds::GranularProcessor();
#if EURORACK_CLOUDS_IS_SUPERPARASITES
#else
    memset(processor, 0, sizeof(*processor));
#endif

    processor->Init(block_mem, memLen, block_ccm, ccmLen);
    mix.set_blocksize(FXConfig::blockSize);
}

template <typename FXConfig> Nimbus<FXConfig>::~Nimbus()
{
    delete[] block_mem;
    delete[] block_ccm;
    delete processor;
}

template <typename FXConfig> void Nimbus<FXConfig>::initialize()
{
    mix.set_target(1.f);
    mix.instantize();

    surgeSR_to_euroSR = std::make_unique<resamp_t>(this->sampleRate(), processor_sr);
    euroSR_to_surgeSR = std::make_unique<resamp_t>(processor_sr, this->sampleRate());

    memset(resampled_output, 0, raw_out_sz * 2 * sizeof(float));

    consumed = 0;
    created = 0;
    builtBuffer = false;
    resampReadPtr = 0;
    resampWritePtr = 1; // why 1? well while we are stalling we want to output 0 so write 1 ahead
}

template <typename FXConfig>
void Nimbus<FXConfig>::processBlock(float *__restrict dataL, float *__restrict dataR)
{
    if (!surgeSR_to_euroSR || !euroSR_to_surgeSR)
        return;

    /* Resample Temp Buffers */
    float resample_this[2][FXConfig::blockSize << 3];
    float resample_into[2][FXConfig::blockSize << 3];

    for (int i = 0; i < FXConfig::blockSize; ++i)
    {
        surgeSR_to_euroSR->push(dataL[i], dataR[i]);
    }

    float srgToEur[2][FXConfig::blockSize << 3];
    auto outputFramesGen = surgeSR_to_euroSR->populateNext(resample_into[0], resample_into[1],
                                                           FXConfig::blockSize << 3);

    if (outputFramesGen)
    {
        clouds::ShortFrame input[FXConfig::blockSize << 3];
        clouds::ShortFrame output[FXConfig::blockSize << 3];

        int frames_to_go = outputFramesGen;
        int outpos = 0;

        auto modeInt = this->intValue(nmb_mode);
        // Just make sure we are safe if we swap between superparasites and not
#if EURORACK_CLOUDS_IS_SUPERPARASITES
        modeInt = std::clamp(modeInt, 0, 7);
#else
        modeInt = std::clamp(modeInt, 0, 3);
#endif
        processor->set_playback_mode(
            (clouds::PlaybackMode)((int)clouds::PLAYBACK_MODE_GRANULAR + modeInt));
        processor->set_quality(this->intValue(nmb_quality));

        int consume_ptr = 0;

        while (frames_to_go + numStubs >= nimbusprocess_blocksize)
        {
            int sp = 0;
            while (numStubs > 0)
            {
                input[sp].l = (short)(std::clamp(stub_input[0][sp], -1.f, 1.f) * 32767.0f);
                input[sp].r = (short)(std::clamp(stub_input[1][sp], -1.f, 1.f) * 32767.0f);
                sp++;
                numStubs--;
            }

            for (int i = sp; i < nimbusprocess_blocksize; ++i)
            {
                input[i].l =
                    (short)(std::clamp(resample_into[0][consume_ptr], -1.f, 1.f) * 32767.0f);
                input[i].r =
                    (short)(std::clamp(resample_into[1][consume_ptr], -1.f, 1.f) * 32767.0f);
                consume_ptr++;
            }

            int inputSz = nimbusprocess_blocksize; // sdata.output_frames_gen + sp;

            auto parm = processor->mutable_parameters();

            float den_val, tex_val;

            den_val = (this->floatValue(nmb_density) + 1.f) * 0.5;
            tex_val = (this->floatValue(nmb_texture) + 1.f) * 0.5;

            parm->position = std::clamp(this->floatValue(nmb_position), 0.f, 1.f);
            parm->size = std::clamp(this->floatValue(nmb_size), 0.f, 1.f);
            parm->density = std::clamp(den_val, 0.f, 1.f);
            parm->texture = std::clamp(tex_val, 0.f, 1.f);
            parm->pitch = std::clamp(this->floatValue(nmb_pitch), -48.f, 48.f);
            parm->stereo_spread = std::clamp(this->floatValue(nmb_spread), 0.f, 1.f);
            parm->feedback = std::clamp(this->floatValue(nmb_feedback), 0.f, 1.f);
            parm->freeze = this->floatValue(nmb_freeze) > 0.5;
            parm->reverb = std::clamp(this->floatValue(nmb_reverb), 0.f, 1.f);
            parm->dry_wet = 1.f;

#if EURORACK_CLOUDS_IS_SUPERPARASITES
            parm->capture = nimbusTrigger;
#else
            parm->trigger = nimbusTrigger; // this is an external granulating source. Skip it
#endif
            parm->gate = parm->freeze; // This is the CV for the freeze button

            processor->Prepare();
            processor->Process(input, output, inputSz);

            for (int i = 0; i < inputSz; ++i)
            {
                resample_this[0][outpos + i] = output[i].l / 32767.0f;
                resample_this[1][outpos + i] = output[i].r / 32767.0f;
            }
            outpos += inputSz;
            frames_to_go -= (nimbusprocess_blocksize - sp);
        }

        if (frames_to_go > 0)
        {
            int startSub = numStubs;
            int addStub = frames_to_go;
            numStubs += frames_to_go;

            for (int i = 0; i < addStub; ++i)
            {
                stub_input[0][i + startSub] = resample_into[0][consume_ptr];
                stub_input[1][i + startSub] = resample_into[1][consume_ptr];
                consume_ptr++;
            }
        }

        if (outpos > 0)
        {
            for (int i = 0; i < outpos; ++i)
            {
                euroSR_to_surgeSR->push(resample_this[0][i], resample_this[1][i]);
            }

            auto dsoutputFramesGen = euroSR_to_surgeSR->populateNext(
                resample_into[0], resample_into[1], FXConfig::blockSize << 3);
            created += dsoutputFramesGen;

            size_t w = resampWritePtr;
            for (int i = 0; i < dsoutputFramesGen; ++i)
            {
                resampled_output[0][w] = resample_into[0][i];
                resampled_output[1][w] = resample_into[1][i];

                w = (w + 1U) & (raw_out_sz - 1U);
            }
            resampWritePtr = w;
        }
    }

    // If you hit this you need to adjust this gapping ratio probably.
    static_assert(FXConfig::blockSize >= nimbusprocess_blocksize);
    int ratio = std::max((int)std::ceil(processor_sr_inv * this->sampleRate()) - 2, 0);
    bool rpi = (created) > (FXConfig::blockSize * (1 + ratio) + 8); // leave some buffer
    if (rpi)
        builtBuffer = true;

    size_t rp = resampReadPtr;
    for (int i = 0; i < FXConfig::blockSize; ++i)
    {
        L[i] = resampled_output[0][rp];
        R[i] = resampled_output[1][rp];
        rp = (rp + rpi) & (raw_out_sz - 1);
    }
    resampReadPtr = rp;

    mix.set_target_smoothed(std::clamp(this->floatValue(nmb_mix), 0.f, 1.f));
    mix.fade_2_blocks_inplace(dataL, L, dataR, R);
}

} // namespace sst::effects::nimbus
#endif

#endif // SURGE_NIMBUSIMPL_H
