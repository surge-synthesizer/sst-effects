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

#ifndef INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_FXCONFIGFROMVFXCONFIG_H
#define INCLUDE_SST_VOICE_EFFECTS_LIFTED_BUS_EFFECTS_FXCONFIGFROMVFXCONFIG_H

#include "../VoiceEffectCore.h"
#include "sst/basic-blocks/mechanics/block-ops.h"

namespace sst::voice_effects::liftbus
{

namespace mech = sst::basic_blocks::mechanics;
template <typename BASE> struct FXConfigFromVFXConfig
{
    using GlobalStorage = BASE;
    using EffectStorage = BASE;
    using BiquadAdapter = BASE;
    using ValueStorage = float;

    struct BaseClass
    {
        BaseClass(GlobalStorage *, EffectStorage *, ValueStorage *) {}
    };

    static constexpr size_t blockSize{BASE::config_t::blockSize};

    static inline float floatValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return v[idx];
    }
    static inline int intValueAt(const BaseClass *const e, const ValueStorage *const v, int idx)
    {
        return (int)std::round(v[idx]);
    }

    static inline float envelopeRateLinear(GlobalStorage *s, float f)
    {
        return s->envelope_rate_linear_nowrap(f);
    }

    static inline float temposyncRatio(GlobalStorage *s, EffectStorage *e, int idx)
    {
        return s->getTempoSyncRatio();
    }

    static inline bool isDeactivated(EffectStorage *e, int idx) { return false; }

    static inline bool isExtended(EffectStorage *s, int idx) { return false; }

    static inline float rand01(GlobalStorage *s)
    {
        assert(false);
        return (float)rand() / (float)RAND_MAX;
    }

    static inline double sampleRate(GlobalStorage *s) { return s->getSampleRate(); }

    static inline float noteToPitch(GlobalStorage *s, float p) { return s->equalNoteToPitch(p); }
    static inline float noteToPitchIgnoringTuning(GlobalStorage *s, float p)
    {
        return noteToPitch(s, p);
    }

    static inline float noteToPitchInv(GlobalStorage *s, float p)
    {
        return 1.0 / noteToPitch(s, p);
    }

    static inline float dbToLinear(GlobalStorage *s, float f) { return s->dbToLinear(f); }
};

template <typename Inside, typename FX> struct LiftHelper
{
    static constexpr size_t memChunkSize{sizeof(FX)};
    uint8_t *busFXMem{nullptr};
    FX *busFX{nullptr};
    Inside *that;
    LiftHelper(Inside *is) : that(is) { that->preReserveSingleInstancePool(memChunkSize); }
    ~LiftHelper()
    {
        if (busFX)
        {
            busFX->~FX();
            that->returnBlock(busFXMem, memChunkSize);
        }
    }

    void init()
    {
        if (!busFX)
        {
            busFXMem = that->checkoutBlock(memChunkSize);
            busFX = new (busFXMem) FX(that, that, &(valuesForFX[0]));
        }

        that->setupValues();
        busFX->initialize();
    }
    float valuesForFX[20];
};

} // namespace sst::voice_effects::liftbus
#endif // SHORTCIRCUITXT_FXCONFIGFROMVFXCONFIG_H
