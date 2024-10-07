#include <memory>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"
#include "../tests/simd-test-include.h"
#include "sst/voice-effects/distortion/BitCrusher.h"
#include "sst/voice-effects/utilities/VolumeAndPan.h"

struct DbToLinearProvider
{
  static constexpr size_t nPoints{512};
  float table_dB[nPoints];
  void init() { for (auto i = 0U; i < nPoints; i++) table_dB[i] = powf(10.f, 0.05f * ((float)i - 384.f)); }
  float dbToLinear(float db) const { db += 384;int e = (int)db;float a = db - (float)e;
    return (1.f - a) * table_dB[e & (nPoints - 1)] + a * table_dB[(e + 1) & (nPoints - 1)];
  }
} dbtlp;

struct FxConfig
{
    struct BaseClass
    {
        std::array<float, 256> fb{};
        std::array<int, 256> ib{};
    };
    static constexpr int blockSize{16};
    static void setFloatParam(BaseClass *b, int i, float f) { b->fb[i] = f; }
    static float getFloatParam(const BaseClass *b, int i) { return b->fb[i]; }

    static void setIntParam(BaseClass *b, int i, int v) { b->ib[i] = v; }
    static int getIntParam(const BaseClass *b, int i) { return b->ib[i]; }

    static float dbToLinear(const BaseClass *, float f) { return dbtlp.dbToLinear(f); }
    static float equalNoteToPitch(const BaseClass *, float f) { return pow(2.f, (f + 69) / 12.f); }
    static float getSampleRate(const BaseClass *) { return 44100.f; }
    static float getSampleRateInv(const BaseClass *) { return 1.0 / 44100.f; }

    static void preReservePool(BaseClass *, size_t) {}
    static void preReserveSingleInstancePool(BaseClass *, size_t) {}
    static uint8_t *checkoutBlock(BaseClass *, size_t n) {
      printf("checkoutBlock %d\n", n);
      uint8_t* ptr = (uint8_t*)malloc(n);
      return ptr;
    }
    static void returnBlock(BaseClass *, uint8_t * ptr, size_t n) {
      printf("returnBlock %d\n", n);
      free(ptr);
    }
};

int main(int argc, char const *argv[]) {
  {
    dbtlp.init();
    auto fx = std::make_unique<sst::voice_effects::utilities::VolumeAndPan<FxConfig>>();
    fx->initVoiceEffectParams();
    fx->setFloatParam(sst::voice_effects::utilities::VolumeAndPan<FxConfig>::fpVolume, 8);
    
    unsigned int channels;
    unsigned int sampleRate;
    drwav_uint64 totalPCMFrameCount;
    float* pSampleData = drwav_open_file_and_read_pcm_frames_f32(argv[1], &channels, &sampleRate, &totalPCMFrameCount, NULL);
    
    printf("sampleRate: %d channels: %d, totalPCMFrameCount: %d\n", sampleRate, channels, totalPCMFrameCount);
    
    uint32_t total = 44100 * 2;
    uint32_t total_blocks = total / FxConfig::blockSize;
    
    uint32_t cur_sample = 0;
    
    FILE* datFile = fopen("/tmp/data.dat", "w" );
    for (size_t i = 0; i < total_blocks; i++) {
      float inputL[16];
      float inputR[16];
      float outputL[16];
      float outputR[16];
      
      for (size_t s = 0; s < 16; s++) {
        inputL[s] = pSampleData[(i * 16) + (s * 2)];
        inputR[s] = pSampleData[(i * 16) + (s * 2) + 1];
      }
      
      fx->processStereo((const float *)&inputL[0],(const float *)&inputR[0],&outputL[0],&outputR[0],1);
      for (size_t sample_index = 0; sample_index < 16; sample_index++) {
        fprintf(datFile, "%d %f %f\n", cur_sample, inputL[sample_index], outputL[sample_index]);
        cur_sample++;
      }
      
      
    }
    fclose(datFile);

    system("gnuplot -p -e \"plot '/tmp/data.dat' using 1:2 with lines, '' using 1:3 with lines\"");
  }
  return 0;
}