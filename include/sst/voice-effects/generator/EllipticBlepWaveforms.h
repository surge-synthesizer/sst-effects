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

#ifndef INCLUDE_SST_VOICE_EFFECTS_GENERATOR_ELLIPTICBLEPWAVEFORMS_H
#define INCLUDE_SST_VOICE_EFFECTS_GENERATOR_ELLIPTICBLEPWAVEFORMS_H

#include "sst/basic-blocks/params/ParamMetadata.h"
#include "sst/basic-blocks/dsp/EllipticBlepOscillators.h"

#include "../VoiceEffectCore.h"

#include "sst/basic-blocks/mechanics/block-ops.h"
#include "sst/basic-blocks/dsp/OscillatorDriftUnisonCharacter.h"
#include "sst/basic-blocks/dsp/PanLaws.h"

namespace sst::voice_effects::generator
{
template <typename VFXConfig>
struct EllipticBlepWaveforms : core::VoiceEffectTemplateBase<VFXConfig>
{
    static constexpr const char *effectName{"Waveform Oscillator"};

    static constexpr int numFloatParams{7};
    static constexpr int numIntParams{5};
    static constexpr int maxUnison{7};

    enum FloatParams
    {
        fpOffset,
        fpSync,
        fpPulseWidth,
        fpUniDetune,
        fpUniWidth,
        fpDrift,
        fpLevel
    };

    enum IntParams
    {
        ipWaveform,
        ipUnisonVoices,
        ipUnisonExtend,
        ipStereo,
        ipRandomPhaseAtOutset
    };

    enum Wave
    {
        SAW = 0,
        SEMISIN = 1,
        PULSE = 2,
        TRI = 3,
        NEARSIN = 4
    };

    basic_blocks::params::ParamMetaData paramAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        auto ot = this->getIntParam(0);
        switch (idx)
        {
        case fpOffset:
            if (keytrackOn)
            {
                return pmd()
                    .asFloat()
                    .withRange(-48, 48)
                    .withDefault(0)
                    .withLinearScaleFormatting("semitones")
                    .withName("Tune");
            }
            return pmd().asAudibleFrequency().withName("Frequency");
        case fpSync:
            return pmd()
                .asFloat()
                .withRange(0, 48)
                .withDefault(0)
                .withLinearScaleFormatting("semitones")
                .withName("Sync");
        case fpPulseWidth:
            return pmd().asPercent().withDefault(0.5).withName("Width");
        case fpUniDetune:
            return pmd()
                .asFloat()
                .withRange(0, 1)
                .withDefault(.1)
                .withLinearScaleFormatting("cents", this->getIntParam(ipUnisonExtend) ? 1200 : 100)
                .withName("Detune");
        case fpUniWidth:
            return pmd().asPercent().withDefault(1.0).withName("Uni Width");
        case fpDrift:
            return pmd().asPercent().withDefault(0.f).withName("Drift");
        case fpLevel:
            return pmd().asCubicDecibelAttenuation().withDefault(1.f).withName("Level");
        }
        return pmd().withName("Unknown " + std::to_string(idx)).asPercent();
    }

    basic_blocks::params::ParamMetaData intParamAt(int idx) const
    {
        using pmd = basic_blocks::params::ParamMetaData;

        switch (idx)
        {
        case ipWaveform:
            return pmd()
                .asInt()
                .withRange(0, 4)
                .withUnorderedMapFormatting({
                    {Wave::SAW, "Saw"},
                    {Wave::SEMISIN, "Semisine"},
                    {Wave::PULSE, "Pulse"},
                    {Wave::TRI, "Triangle"},
                    {Wave::NEARSIN, "Sine"},
                })
                .withName("Wave");
        case ipUnisonVoices:
            return pmd()
                .asInt()
                .withRange(1, 7)
                .withDefault(1)
                .withLinearScaleFormatting("")
                .withName("Unison Count");
        case ipUnisonExtend:
            return pmd().asOnOffBool().withDefault(false).withName("Extend Unison");
        case ipStereo:
            return pmd().asStereoSwitch().withDefault(true);
        case ipRandomPhaseAtOutset:
            return pmd().asOnOffBool().withDefault(false).withName("Randomize Phase");
        }
        return pmd().withName("error");
    }

    EllipticBlepWaveforms() : core::VoiceEffectTemplateBase<VFXConfig>(), uni(1) {}
    ~EllipticBlepWaveforms() = default;

    void initVoiceEffect()
    {
        for (int i = 0; i < maxUnison; ++i)
        {
            sawOscs[i].setSampleRate(this->getSampleRate());
            semisinOscs[i].setSampleRate(this->getSampleRate());
            sinOscs[i].setSampleRate(this->getSampleRate());
            triOscs[i].setSampleRate(this->getSampleRate());
            pulseOscs[i].setSampleRate(this->getSampleRate());
        }
        if (this->getIntParam(ipRandomPhaseAtOutset))
        {
            auto sr = this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpSync));

            for (int i = 0; i < maxUnison; ++i)
            {
                sawOscs[i].setInitialPhase(rng.unif01(), sr);
                semisinOscs[i].setInitialPhase(rng.unif01(), sr);
                sinOscs[i].setInitialPhase(rng.unif01(), sr);
                triOscs[i].setInitialPhase(rng.unif01(), sr);
                pulseOscs[i].setInitialPhase(rng.unif01(), sr);
            }
        }
        for (int i = 0; i < maxUnison; ++i)
        {
            driftLFOs[i].init(true);
        }
    }
    void initVoiceEffectParams() { this->initToParamMetadataDefault(this); }

    template <bool toStereo, typename T>
    void genericProcess(std::array<T, maxUnison> &t, float *dataoutL, float *dataoutR, float pitch)
    {
        auto uc = std::max(this->getIntParam(ipUnisonVoices), 1);
        auto ue = this->getIntParam(ipUnisonExtend);
        auto upw = this->getFloatParam(fpUniWidth);

        if (uc != lastUnison)
        {
            uni = sst::basic_blocks::dsp::UnisonSetup<float>(uc);
            lastUnison = uc;
            uniPanValid = false;
        }
        if (upw != lastUniPanWidth)
        {
            uniPanValid = false;
            lastUniPanWidth = upw;
        }
        float tune = this->getFloatParam(fpOffset);

        auto sr = this->note_to_pitch_ignoring_tuning(this->getFloatParam(fpSync));

        // Drift I just normalized by ear here

        // dt is 0->1 mapping to 0->100 cents
        auto dt = this->getFloatParam(fpUniDetune) * (ue ? 12 : 1);

        if constexpr (toStereo)
        {
            if (!uniPanValid)
            {
                for (auto u = 0; u < uc; ++u)
                {
                    auto bs = uni.detune(u) * this->getFloatParam(fpUniWidth);
                    auto pv = std::clamp(bs, -1.f, 1.f) * 0.5 + 0.5;

                    sst::basic_blocks::dsp::pan_laws::monoEqualPowerUnityGainAtExtrema(pv,
                                                                                       uniPans[u]);
                }
                uniPanValid = true;
            }
        }

        auto driftLevel = this->getFloatParam(fpDrift);
        for (int u = 0; u < uc; ++u)
        {
            auto driftVal = driftLFOs[u].next() * driftLevel * 0.5;

            auto baseFreq = 440.0 * this->note_to_pitch_ignoring_tuning(
                                        (keytrackOn) ? tune + driftVal + pitch + dt * uni.detune(u)
                                                     : tune + driftVal + dt * uni.detune(u));
            t[u].setFrequency(baseFreq);
            t[u].setSyncRatio(sr);
            if (toStereo)
            {
                if (u == 0)
                {
                    for (int i = 0; i < VFXConfig::blockSize; ++i)
                    {
                        auto s = t[u].step() * uni.attenuation();
                        dataoutL[i] = uniPans[u][0] * s;
                        dataoutR[i] = uniPans[u][3] * s;
                    }
                }
                else
                {
                    for (int i = 0; i < VFXConfig::blockSize; ++i)
                    {
                        auto s = t[u].step() * uni.attenuation();
                        dataoutL[i] += uniPans[u][0] * s;
                        dataoutR[i] += uniPans[u][3] * s;
                    }
                }
            }
            else
            {
                if (u == 0)
                {
                    for (int i = 0; i < VFXConfig::blockSize; ++i)
                        dataoutL[i] = t[u].step() * uni.attenuation();
                }
                else
                {
                    for (int i = 0; i < VFXConfig::blockSize; ++i)
                        dataoutL[i] += t[u].step() * uni.attenuation();
                }
            }
        }
    }

    template <bool toStereo> void processTo(float *dataoutL, float *dataoutR, float pitch)
    {
        int wave = this->getIntParam(ipWaveform);
        switch ((Wave)wave)
        {
        case Wave::SAW:
            genericProcess<toStereo>(sawOscs, dataoutL, dataoutR, pitch);
            break;
        case Wave::SEMISIN:
            genericProcess<toStereo>(semisinOscs, dataoutL, dataoutR, pitch);
            break;
        case Wave::PULSE:
        {
            auto uc = this->getIntParam(ipUnisonVoices);
            auto pw = std::clamp(this->getFloatParam(fpPulseWidth), 0.01f, 0.99f);
            for (int i = 0; i < uc; ++i)
            {
                pulseOscs[i].setWidth(pw);
            }
            genericProcess<toStereo>(pulseOscs, dataoutL, dataoutR, pitch);
        }
        break;
        case Wave::NEARSIN:
            genericProcess<toStereo>(sinOscs, dataoutL, dataoutR, pitch);
            break;
        case Wave::TRI:
            genericProcess<toStereo>(triOscs, dataoutL, dataoutR, pitch);
            break;
        }

        auto levT = std::clamp(this->getFloatParam(fpLevel), 0.f, 1.f);
        levT = levT * levT * levT;
        sLevelLerp.set_target(levT);

        if constexpr (!toStereo)
        {
            sLevelLerp.multiply_block(dataoutL);
        }
        else
        {
            sLevelLerp.multiply_2_blocks(dataoutL, dataoutR);
        }
    }

    void processMonoToMono(const float *const datainL, float *dataoutL, float pitch)
    {
        uniPanValid = false;
        processTo<false>(dataoutL, nullptr, pitch);
    }
    void processMonoToStereo(const float *const datainL, float *dataoutL, float *dataoutR,
                             float pitch)
    {
        processTo<true>(dataoutL, dataoutR, pitch);
    }

    void processStereo(const float *const datainL, const float *const datainR, float *dataoutL,
                       float *dataoutR, float pitch)
    {
        processTo<true>(dataoutL, dataoutR, pitch);
    }

    bool enableKeytrack(bool b)
    {
        auto res = (b != keytrackOn);
        keytrackOn = b;
        return res;
    }
    bool getKeytrack() const { return keytrackOn; }
    bool getKeytrackDefault() const { return true; }
    bool checkParameterConsistency() const { return true; }
    bool getMonoToStereoSetting() const { return this->getIntParam(ipStereo) > 0; }

  protected:
    bool keytrackOn{true};

    sst::basic_blocks::dsp::lipol_sse<VFXConfig::blockSize, true> sLevelLerp;

    using bpi_t = sst::basic_blocks::dsp::BlockInterpSmoothingStrategy<VFXConfig::blockSize>;
    using saw_t = sst::basic_blocks::dsp::EBSaw<bpi_t>;
    using semisin_t = sst::basic_blocks::dsp::EBApproxSemiSin<bpi_t>;
    using pulse_t = sst::basic_blocks::dsp::EBPulse<bpi_t>;
    using tri_t = sst::basic_blocks::dsp::EBTri<bpi_t>;
    using sin_t = sst::basic_blocks::dsp::EBApproxSin<bpi_t>;

    std::array<saw_t, maxUnison> sawOscs;
    std::array<semisin_t, maxUnison> semisinOscs;
    std::array<pulse_t, maxUnison> pulseOscs;
    std::array<tri_t, maxUnison> triOscs;
    std::array<sin_t, maxUnison> sinOscs;
    std::array<sst::basic_blocks::dsp::pan_laws::panmatrix_t, maxUnison> uniPans;

    std::array<sst::basic_blocks::dsp::DriftLFO, maxUnison> driftLFOs;
    sst::basic_blocks::dsp::UnisonSetup<float> uni;
    int lastUnison{1};
    float lastUniPanWidth{-10000.f};
    bool uniPanValid{false};

    sst::basic_blocks::dsp::RNG rng;

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
} // namespace sst::voice_effects::generator

#endif // SHORTCIRCUITXT_GENVA_H
