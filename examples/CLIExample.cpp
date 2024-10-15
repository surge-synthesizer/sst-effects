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

#include <memory>
#include <string>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <stdio.h>
#include <CLI/CLI.hpp>
#include <fmt/core.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"
#include "../tests/simd-test-include.h"
#include "sst/voice-effects/distortion/BitCrusher.h"
#include "sst/voice-effects/utilities/VolumeAndPan.h"
#include "sst/voice-effects/dynamics/Compressor.h"

#include "sst/effects/Reverb2.h"
#include "sst/effects/Bonsai.h"
#include "sst/effects/FloatyDelay.h"

struct DbToLinearProvider
{
    static constexpr size_t nPoints{512};
    float table_dB[nPoints];

    void init()
    {
        for (auto i = 0U; i < nPoints; i++)
            table_dB[i] = powf(10.f, 0.05f * ((float)i - 384.f));
    }

    float dbToLinear(float db) const
    {
        db += 384;
        int e = (int)db;
        float a = db - (float)e;
        return (1.f - a) * table_dB[e & (nPoints - 1)] + a * table_dB[(e + 1) & (nPoints - 1)];
    }
};

struct CLIArgBundle
{
    std::string infileName{};
    std::string outfileName{};
    std::string datfileName{};
    bool launchGnuplot{false};

    std::vector<float> fArgs;
    std::vector<int> iArgs;
};

struct SSTFX
{
    std::array<float, 256> fb{};
    std::array<int, 256> ib{};
    float sampleRate;
    DbToLinearProvider dbtlp;

    struct FxConfig
    {
        using BaseClass = SSTFX;
        static constexpr int blockSize{16};
        static void setFloatParam(BaseClass *b, int i, float f) { b->fb[i] = f; }
        static float getFloatParam(const BaseClass *b, int i) { return b->fb[i]; }

        static void setIntParam(BaseClass *b, int i, int v) { b->ib[i] = v; }
        static int getIntParam(const BaseClass *b, int i) { return b->ib[i]; }

        static float dbToLinear(const BaseClass *b, float f) { return b->dbtlp.dbToLinear(f); }

        static float equalNoteToPitch(const BaseClass *, float f)
        {
            return pow(2.f, (f + 69) / 12.f);
        }

        static float getSampleRate(const BaseClass *b) { return b->sampleRate; }
        static float getSampleRateInv(const BaseClass *b) { return 1.0 / b->sampleRate; }

        static void preReservePool(BaseClass *, size_t) {}

        static void preReserveSingleInstancePool(BaseClass *, size_t) {}

        static uint8_t *checkoutBlock(BaseClass *, size_t n)
        {
            printf("checkoutBlock %zu\n", n);
            uint8_t *ptr = (uint8_t *)malloc(n);
            return ptr;
        }

        static void returnBlock(BaseClass *, uint8_t *ptr, size_t n)
        {
            printf("returnBlock %zu\n", n);
            free(ptr);
        }
    };

    // std::unique_ptr<sst::voice_effects::distortion::BitCrusher<FxConfig>> fx;

    // std::unique_ptr<sst::voice_effects::dynamics::Compressor<FxConfig>> fx;

    SSTFX() { dbtlp.init(); }
};

struct ConcreteConfig
{
    struct BC
    {
        static constexpr uint16_t maxParamCount{20};
        float paramStorage[maxParamCount];
        template <typename... Types> BC(Types...) {}
    };
    struct GS
    {
        double sampleRate;
        DbToLinearProvider dbtlp;
        // It is painful that sst-filters makes us over-adapt
        // this class
        GS(double sr) : sampleRate(sr) {
          dbtlp.init();
        }
    };
    struct ES
    {
    };
    using BaseClass = BC;
    using GlobalStorage = GS;
    using EffectStorage = ES;
    using ValueStorage = float *;
    using BiquadAdapter = ConcreteConfig;

    static constexpr int blockSize{16};

    static inline float floatValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return e->paramStorage[idx];
    }
    static inline int intValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return (int)std::round(e->paramStorage[idx]);
    }
    static inline float envelopeRateLinear(GlobalStorage *s, float f)
    {
        return 1.f * blockSize / (s->sampleRate) * pow(-2, f);
    }
    static inline float temposyncRatio(GlobalStorage *s, EffectStorage *e, int idx) { return 1; }
    static inline bool isDeactivated(EffectStorage *e, int idx) { return false; }
    static inline bool isExtended(EffectStorage *e, int idx) { return false; }
    static inline float rand01(GlobalStorage *s) { return (float)rand() / (float)RAND_MAX; }
    static inline double sampleRate(GlobalStorage *s) { return s->sampleRate; }
    static inline double sampleRateInv(GlobalStorage *s) { return 1.0 / s->sampleRate; }
    static inline float noteToPitch(GlobalStorage *s, float p) { return pow(2.0, p / 12); }
    static inline float noteToPitchIgnoringTuning(GlobalStorage *s, float p) { return noteToPitch(s, p); }
    static inline float noteToPitchInv(GlobalStorage *s, float p) { return 1.0 / noteToPitch(s, p); }
    static inline float dbToLinear(GlobalStorage *s, float f) { return s->dbtlp.dbToLinear(f); }
};

int writeOutfile(std::string filename, int sampleRate, uint32_t sample_count, float* samples)
{
    drwav wav;
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_IEEE_FLOAT;
    format.channels = 2;
    format.sampleRate = sampleRate;
    format.bitsPerSample = 32;
    std::cout << "Writing " << sample_count << " r=" << sampleRate << " sample wav file to "
              << filename << std::endl;
    if (!drwav_init_file_write(&wav, filename.c_str(), &format, nullptr))
    {
        std::cout << "Cannot init file write the outfile" << std::endl;
        return 3;
    }
    drwav_uint64 framesWritten = drwav_write_pcm_frames(&wav, sample_count, samples);
    drwav_uninit(&wav);
    return 0;
}

int writeDatfile(std::string filename, uint32_t sample_count, float* samples)
{
    FILE *datFile = fopen(filename.c_str(), "w");
    
    if (!datFile)
    {
        std::cout << "Datfile not open at '" << filename << "'" << std::endl;
        return 4;
    } else {
        for (size_t s = 0; s < sample_count; s++) {
            fprintf(datFile, "%d %f %f\n", s, samples[s * 2], samples[s * 2 + 1]);
        }
        fclose(datFile);
    }
    return 0;
}

void launchGnuplot(std::string filename)
{
    auto cmd = fmt::format("gnuplot -p -e \"plot '{}' using 1:2 with lines, '' using "
                           "1:3 with lines;pause mouse close\"",
                           filename);
    std::cout << "Launching " << cmd << std::endl;
    system(cmd.c_str());
}

template <typename FXT> int voiceEffectExampleHarness(const CLIArgBundle &arg)
{
    unsigned int channels;
    unsigned int sampleRate;
    drwav_uint64 totalPCMFrameCount;
    float *pSampleData = drwav_open_file_and_read_pcm_frames_f32(
        arg.infileName.c_str(), &channels, &sampleRate, &totalPCMFrameCount, NULL);

    // TODO - how does this report errors?
    if (totalPCMFrameCount <= 0 || pSampleData == nullptr)
    {
        std::cout << "No samples in file. Exiting" << std::endl;
        return 2;
    }
    printf("sampleRate: %d channels: %d, totalPCMFrameCount: %llu\n", sampleRate, channels,
           totalPCMFrameCount);

    if (channels > 2)
    {
        printf("Only 1 or 2 channels wav files supported, exiting.\n");
        return 3;
    }

    auto fx = std::make_unique<FXT>();
    fx->sampleRate = sampleRate;
    fx->initVoiceEffectParams();

    int ai{0};
    for (const auto &f : arg.fArgs)
    {
        fx->setFloatParam(ai, f);
        ai++;
    }
    ai = 0;
    for (const auto &i : arg.iArgs)
    {
        fx->setIntParam(ai, i);
        ai++;
    }

    static constexpr auto blockSize = SSTFX::FxConfig::blockSize;

    uint32_t total_blocks = totalPCMFrameCount / blockSize;

    uint32_t sample_count = 0;

    // FIXME - if we can block this we probably should
    // FIXME - there are tails on effects and we need a way to specify how many sapmle tails
    auto outputSamples = new float[totalPCMFrameCount * 2];

    for (size_t block = 0; block < total_blocks; block++)
    {
        float inputL[blockSize];
        float inputR[blockSize];
        float outputL[blockSize];
        float outputR[blockSize];

        for (size_t s = 0; s < blockSize; s++)
        {
            if (channels == 2)
            {
                inputL[s] = pSampleData[(block * (blockSize * 2)) + (s * 2)];
                inputR[s] = pSampleData[(block * (blockSize * 2)) + (s * 2) + 1];
            }
            else
            {
                inputL[s] = pSampleData[(block * blockSize) + s];
                inputR[s] = inputL[s];
            }
        }

        fx->processStereo((const float *)&inputL[0], (const float *)&inputR[0], &outputL[0],
                          &outputR[0], 1);

        for (size_t sample_index = 0; sample_index < blockSize; sample_index++)
        {
            outputSamples[(block * (blockSize * 2)) + (sample_index * 2)] = outputL[sample_index];
            outputSamples[(block * (blockSize * 2)) + (sample_index * 2) + 1] =
                outputR[sample_index];

            sample_count++;
        }
    }

    int err;
    if (err = writeOutfile(arg.outfileName, sampleRate, sample_count, outputSamples)) return err;

    if (!arg.datfileName.empty() && (err = writeDatfile(arg.datfileName, sample_count, outputSamples)))
        return err;

    if (arg.launchGnuplot)
        launchGnuplot(arg.datfileName);

    delete[] outputSamples;

    return 0;
}

template <typename FXT> int effectExampleHarness(const CLIArgBundle &arg)
{
    unsigned int channels;
    unsigned int sampleRate;
    drwav_uint64 totalPCMFrameCount;
    float *pSampleData = drwav_open_file_and_read_pcm_frames_f32(
        arg.infileName.c_str(), &channels, &sampleRate, &totalPCMFrameCount, NULL);

    // TODO - how does this report errors?
    if (totalPCMFrameCount <= 0 || pSampleData == nullptr)
    {
        std::cout << "No samples in file. Exiting" << std::endl;
        return 2;
    }
    printf("sampleRate: %d channels: %d, totalPCMFrameCount: %llu\n", sampleRate, channels,
           totalPCMFrameCount);

    if (channels > 2)
    {
        printf("Only 1 or 2 channels wav files supported, exiting.\n");
        return 3;
    }

    auto gs = ConcreteConfig::GlobalStorage(sampleRate);
    auto es = ConcreteConfig::EffectStorage();

    auto fx = std::make_unique<FXT>(&gs, &es, nullptr);
    for (int i = 0; i < FXT::numParams; ++i)
      fx->paramStorage[i] = fx->paramAt(i).defaultVal;

    int ai{0};
    for (const auto &f : arg.fArgs)
    {
        fx->paramStorage[ai] = f;
        ai++;
    }
    fx->initialize();

    static constexpr auto blockSize = ConcreteConfig::blockSize;

    uint32_t total_blocks = totalPCMFrameCount / blockSize;

    uint32_t sample_count = 0;

    // FIXME - if we can block this we probably should
    // FIXME - there are tails on effects and we need a way to specify how many sapmle tails
    auto outputSamples = new float[totalPCMFrameCount * 2];

    for (size_t block = 0; block < total_blocks; block++)
    {
        float outputL[blockSize];
        float outputR[blockSize];

        for (size_t s = 0; s < blockSize; s++)
        {
            if (channels == 2)
            {
                outputL[s] = pSampleData[(block * (blockSize * 2)) + (s * 2)];
                outputR[s] = pSampleData[(block * (blockSize * 2)) + (s * 2) + 1];
            }
            else
            {
                outputL[s] = pSampleData[(block * blockSize) + s];
                outputR[s] = outputL[s];
            }
        }

        fx->processBlock(outputL, outputR);

        for (size_t sample_index = 0; sample_index < blockSize; sample_index++)
        {
            outputSamples[(block * (blockSize * 2)) + (sample_index * 2)] = outputL[sample_index];
            outputSamples[(block * (blockSize * 2)) + (sample_index * 2) + 1] =
                outputR[sample_index];

            sample_count++;
        }
    }

    int err;
    if (err = writeOutfile(arg.outfileName, sampleRate, sample_count, outputSamples)) return err;

    if (!arg.datfileName.empty() && (err = writeDatfile(arg.datfileName, sample_count, outputSamples)))
        return err;

    if (arg.launchGnuplot)
        launchGnuplot(arg.datfileName);

    delete[] outputSamples;

    return 0;
}

int main(int argc, char const *argv[])
{
    /*
     * Set up command line arguments
     */
    CLI::App app("..:: Voice Effects Example - Command Line player for SST Voice Effects ::..");
    CLIArgBundle arg;

    app.add_option("-i,--infile", arg.infileName, "Input wav file for session")->required();
    app.add_option("-o,--outfile", arg.outfileName, "Output wav file for session")->required();
    app.add_option("-d,--datfile", arg.datfileName, "Optional plain text dat file");
    app.add_flag("--gnuplot", arg.launchGnuplot, "Attempt to launch gnuplot on datfile");
    app.add_option("--fargs", arg.fArgs, "Floating arguments in order");
    app.add_option("--iargs", arg.iArgs, "Integer arguments in order");

    std::string fxType;
    app.add_option("-t,--type", fxType, "FX Type to run");

    // TODO
    // - Add a vec option (https://cliutils.github.io/CLI11/book/chapters/options.html)
    //   for float and int params
    // - RTAudio rather than file output
    // - Add a ringout option

    CLI11_PARSE(app, argc, argv);
    
    if (arg.launchGnuplot && arg.datfileName.empty())
    {
        std::cout << "To launch gnuplot you need to specify a datfile with -d" << std::endl;
        return 2;
    }

    std::vector<std::string> types;
#define ADD_VFX_TYPE(key, cls)                                                                          \
    {                                                                                              \
        types.push_back(std::string(key) + " -> " + #cls);                                         \
        if (fxType == std::string(key))                                                            \
            return voiceEffectExampleHarness<sst::voice_effects::cls<SSTFX::FxConfig>>(arg);                  \
    }
#define ADD_FX_TYPE(key, cls)                                                                          \
    {                                                                                              \
        types.push_back(std::string(key) + " -> " + #cls);                                         \
        if (fxType == std::string(key))                                                            \
            return effectExampleHarness<sst::effects::cls<ConcreteConfig>>(arg);                  \
    }
    ADD_VFX_TYPE("volpan", utilities::VolumeAndPan);
    ADD_VFX_TYPE("bitcrush", distortion::BitCrusher);
    
    ADD_FX_TYPE("reverb2", reverb2::Reverb2);
    ADD_FX_TYPE("floaty", floatydelay::FloatyDelay);
    ADD_FX_TYPE("bonsai", bonsai::Bonsai);

    // If we get kere no keys matched
    std::cout << "Unable to find fx '" << fxType << "'. Available options are :\n";
    for (const auto &opt : types)
    {
        std::cout << "   -t " << opt << "\n";
    }
    std::cout << std::endl;
    return 1;
}