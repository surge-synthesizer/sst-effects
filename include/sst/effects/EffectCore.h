/*
 * flarg
 */


#ifndef PREMD_FXCORE
#define PREMD_FXCORE

/*
 * OK so here's the deal
 *
 * In this pre-factoring folder is stuff which will live in sst-basic-blocks or sst-effects
 * and you can tell by the namespace name
 *
 * in basic-blocks::params we have parameter medatadat. its just a class describing a param
 * and pretty boring. The core is really here
 *
 * We say that all fx have a global storage, which answers questions like sample rate,
 * an fx storage, which answers questions about particular confiugration, and a value storage
 * which answers questions about actual float or int values. We make no assumptions about these
 * classes other than (1) they have pointer semantics and are owned outside us longer than us and
 * (2) they can be passed as pointers to a template type which then probes them.
 *
 * That template type is `FXConfig`. You can see in this base class the questions we
 * ask of it, but for instance, to get the value of a particular param we ask
 * `FXConfig::floatValueAt(ValueStorage *, int idx)` and to see if an item is
 * tempo synced we ask `FXConfig::tempoSyncRation(GlobalStorage*, FXStorage *, int)`.
 * This forms an API which means we can be neutral as to how these three bundles work.
 *
 * WIth that in hand, we can rewrite the flanger. Check out Flanger.h. It's pretty obvious
 * but basically (1) advertise param metadata, (2) handle process, init, etc.. and
 * (3) use the FXConfig to probe values and state.
 *
 * Now once we have that we have to go back to surge. The next header you want to
 * read ins src/common/dsp/effects/SurgeSSTFXAdapter.h. This gives us two things
 *
 * 1. It gives us an instance of FXCOnfig with the appropriate typedefs and methods and
 * 2. It gives us a base class which inherits from the virtual base class Effect and
 *    also inherits from a T, which is intended to be something the shape of Flanger.
 *
 *  WIth those two inheritances it can then implement a vast subset of Effect in a
 *  generic fashion just redirecting to the template.
 *
 *  And finally, you retain FlangerEffect.h and FLangerEfffect.cpp but these are massively
 *  reduced. They basically implement only the things which are surge specific. For
 *  instance the group pos api remains. And init_ctrltypes is still there with the old
 *  surge parameter, but we check that those parameters are configured to be consisten
 *  tiwht the metadata (obvioulsy more to do there). See the 'configureControlsFromFXMetadata`
 *  call at the end of init_ctrltypes.
 *
 *  And voila. Most of the effect is reuasable and decoupled.
 *
 */

namespace sst::fx
{
// Todo: as we port consider this FXConfig::BaseClass being a bit more configurable.
template<typename FXConfig>
struct EffectTemplateBase : public FXConfig::BaseClass
{
    static_assert(std::is_class<typename FXConfig::BaseClass>::value);
    static_assert(std::is_pointer<typename FXConfig::GlobalStorage *>::value);
    static_assert(std::is_pointer<typename FXConfig::EffectStorage *>::value);
    static_assert(std::is_pointer<typename FXConfig::ValueStorage *>::value);
    static_assert(std::is_same<decltype(FXConfig::floatValueAt), float(const typename FXConfig::BaseClass * const, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::intValueAt), int(const typename FXConfig::BaseClass * const, int)>::value);
    static_assert(std::is_same<decltype(FXConfig::temposyncRatio), float(typename FXConfig::GlobalStorage *, typename FXConfig::EffectStorage *, int)>::value);

    typename FXConfig::GlobalStorage *globalStorage{nullptr};
    typename FXConfig::EffectStorage *fxStorage{nullptr};
    typename FXConfig::ValueStorage *valueStorage{nullptr};

    EffectTemplateBase(typename FXConfig::GlobalStorage *s,
                       typename FXConfig::EffectStorage *e,
                       typename FXConfig::ValueStorage *p) : FXConfig::BaseClass(s,e,p), globalStorage(s), fxStorage(e), valueStorage(p) {
    }

    inline typename FXConfig::BaseClass *asBase() {
        return static_cast<typename FXConfig::BaseClass *>(this);
    }

    inline const typename FXConfig::BaseClass * const asBase() const {
        return static_cast<const typename FXConfig::BaseClass * const>(this);
    }

    inline float floatValue(int idx) const
    {
        return FXConfig::floatValueAt(asBase(), idx);
    }

    inline float intValue(int idx) const
    {
        return FXConfig::intValueAt(asBase(), idx);
    }

    inline float temposyncRatio(int idx) const
    {
        return FXConfig::temposyncRatio(globalStorage, fxStorage, idx);
    }

    inline bool isDeactivated(int idx) const
    {
        return FXConfig::isDeactivated(fxStorage, idx);
    }

    inline float envelopeRateLinear(float f) const
    {
        return FXConfig::envelopeRateLinear(globalStorage, f);
    }

    inline float storageRand01()
    {
        return FXConfig::rand01(globalStorage);
    }

    inline double sampleRate()
    {
        return FXConfig::sampleRate(globalStorage);
    }

    inline float noteToPitch(float p)
    {
        return FXConfig::noteToPitch(globalStorage, p);
    }

    inline float noteToPitchIgnoringTuning(float p)
    {
        return FXConfig::noteToPitchIgnoringTuning(globalStorage, p);
    }

    inline float noteToPitchInv(float p)
    {
        return FXConfig::noteToPitchInv(globalStorage, p);
    }

    inline float dbToLinear(float f)
    {
        return FXConfig::dbToLinear(globalStorage, f);
    }

    template<typename lipol>
    inline void applyWidth(float *__restrict L, float *__restrict R, lipol &width)
    {
        namespace sdsp = sst::basic_blocks::dsp;
        float M alignas(16)[BLOCK_SIZE], S alignas(16)[BLOCK_SIZE];
        sdsp::encodeMS<BLOCK_SIZE>(L, R, M, S);
        width.multiply_block(S, BLOCK_SIZE_QUAD);
        sdsp::decodeMS<BLOCK_SIZE>(M, S, L, R);
    }

};
}

#endif