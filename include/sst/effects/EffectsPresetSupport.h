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

#ifndef INCLUDE_SST_EFFECTS_EFFECTSPRESETSUPPORT_H
#define INCLUDE_SST_EFFECTS_EFFECTSPRESETSUPPORT_H

#if SST_EFFECTS_PRESETS
#include "tinyxml/tinyxml.h"
#include "preset-api.h"
#include <vector>
#include <string>

namespace sst::effects::presets
{

/**
 * Why separate concepts for getting and setting rather than do it
 * from an instance? Well in some cases (including short circuit) we
 * have the storage for an effect in theui thread but not the instance
 * so we want to be able to adapt.
 *
 * If you want to use an instance, its easy to make a template adaptation
 * from these receiver classes to an instance, but I haven't written that
 * code yet. It would look like a template-to-instance adapter basically
 * though.
 */

template <typename RC>
concept PresetReceiver = requires(RC a, size_t idx, float f, bool b, int v, std::string s) {
    { a.canReceiveForStreamingName(s) } -> std::same_as<bool>;

    { a.receiveStreamingVersion(v) } -> std::same_as<bool>;
    { a.receiveFloatParam(idx, f) } -> std::same_as<bool>;
    { a.receiveIntParam(idx, f) } -> std::same_as<bool>;
    { a.receiveDeactivated(idx, b) } -> std::same_as<bool>;
    { a.receiveExtended(idx, b) } -> std::same_as<bool>;
    { a.receiveTemposync(idx, b) } -> std::same_as<bool>;

    { a.onError(s) } -> std::same_as<void>;
};

template <typename RC>
concept PresetProvider = requires(const RC a, size_t idx, float f, bool b, std::string s) {
    { a.provideParamCount() } -> std::convertible_to<size_t>;
    { a.isParamInt(idx) } -> std::same_as<bool>;
    { a.provideFloatParam(idx) } -> std::same_as<float>;
    { a.provideIntParam(idx) } -> std::same_as<int>;
    { a.provideDeactivated(idx) } -> std::same_as<bool>;
    { a.provideTemposync(idx) } -> std::same_as<bool>;
    { a.provideExtended(idx) } -> std::same_as<bool>;
    { a.provideStreamingVersion() } -> std::convertible_to<int>;
    { a.provideStreamingName() } -> std::convertible_to<std::string>;
};

inline std::string toPreset(const PresetProvider auto &pv)
{
    TiXmlDocument doc("sst-effect");
    TiXmlElement rootNode("sst-effect");
    rootNode.SetAttribute("effect-streaming-version", pv.provideStreamingVersion());
    rootNode.SetAttribute("preset-streaming-version", 1);
    rootNode.SetAttribute("effect", pv.provideStreamingName());

    for (int i = 0; i < pv.provideParamCount(); ++i)
    {
        TiXmlElement fpar("param");
        fpar.SetAttribute("index", i);
        if (pv.isParamInt(i))
        {
            fpar.SetAttribute("type", "i");
            fpar.SetAttribute("value", pv.provideIntParam(i));
        }
        else
        {
            fpar.SetAttribute("type", "f");
            fpar.SetDoubleAttribute("value", pv.provideFloatParam(i));
        }
        fpar.SetAttribute("deactivated", pv.provideDeactivated(i));
        fpar.SetAttribute("extended", pv.provideExtended(i));
        rootNode.InsertEndChild(fpar);
    }

    doc.InsertEndChild(rootNode);
    TiXmlPrinter printer;
    doc.Accept(&printer);

    return printer.Str();
}

inline bool fromPreset(const std::string &s, PresetReceiver auto &r)
{
    TiXmlDocument doc;
    doc.Parse(s.c_str());
    if (doc.Error())
    {
        r.onError("Failed to parse preset XML: " +
                  std::string(doc.ErrorDesc() ? doc.ErrorDesc() : "unknown error"));
        return false;
    }

    auto *root = doc.FirstChildElement("sst-effect");
    if (!root)
    {
        r.onError("Missing root 'sst-effect' element in preset XML");
        return false;
    }

    // Validate preset streaming version
    int presetStreamingVersion = 0;
    if (root->QueryIntAttribute("preset-streaming-version", &presetStreamingVersion) !=
        TIXML_SUCCESS)
    {
        r.onError("Missing 'preset-streaming-version' attribute in preset XML");
        return false;
    }
    if (presetStreamingVersion != 1)
    {
        r.onError("Unsupported preset streaming version: " +
                  std::to_string(presetStreamingVersion));
        return false;
    }

    // Effect name and streaming version
    const char *effectNameC = root->Attribute("effect");
    if (!effectNameC)
    {
        r.onError("Missing 'effect' attribute in preset XML");
        return false;
    }
    std::string effectName(effectNameC);
    if (!r.canReceiveForStreamingName(effectName))
    {
        r.onError("Receiver cannot accept preset for effect: " + effectName);
        return false;
    }

    int effectStreamingVersion = 0;
    if (root->QueryIntAttribute("effect-streaming-version", &effectStreamingVersion) !=
        TIXML_SUCCESS)
    {
        r.onError("Missing 'effect-streaming-version' attribute in preset XML");
        return false;
    }
    if (!r.receiveStreamingVersion(effectStreamingVersion))
    {
        r.onError("Receiver rejected effect streaming version: " +
                  std::to_string(effectStreamingVersion));
        return false;
    }

    // Iterate parameters
    for (auto *par = root->FirstChildElement("param"); par; par = par->NextSiblingElement("param"))
    {
        int idx = -1;
        if (par->QueryIntAttribute("index", &idx) != TIXML_SUCCESS || idx < 0)
        {
            r.onError("Param element missing or has invalid 'index' attribute");
            return false;
        }

        const char *typeC = par->Attribute("type");
        if (!typeC)
        {
            r.onError("Param element missing 'type' attribute (index: " + std::to_string(idx) +
                      ")");
            return false;
        }

        // Optional attributes with defaults
        int deactI = 0, extI = 0, tsI = 0;
        par->QueryIntAttribute("deactivated", &deactI);
        par->QueryIntAttribute("extended", &extI);
        // Not emitted by toPreset currently, but accept if present
        par->QueryIntAttribute("temposync", &tsI);

        // Apply flags first so receiver can use them if it wants prior to value
        if (!r.receiveDeactivated(idx, deactI != 0))
        {
            r.onError("Receiver rejected 'deactivated' for index " + std::to_string(idx));
            return false;
        }
        if (!r.receiveExtended(idx, extI != 0))
        {
            r.onError("Receiver rejected 'extended' for index " + std::to_string(idx));
            return false;
        }
        if (!r.receiveTemposync(idx, tsI != 0))
        {
            r.onError("Receiver rejected 'temposync' for index " + std::to_string(idx));
            return false;
        }

        if (typeC[0] == 'i')
        {
            int ival = 0;
            if (par->QueryIntAttribute("value", &ival) != TIXML_SUCCESS)
            {
                r.onError("Param index " + std::to_string(idx) + " missing int 'value'");
                return false;
            }
            if (!r.receiveIntParam(idx, ival))
            {
                r.onError("Receiver rejected int param at index " + std::to_string(idx));
                return false;
            }
        }
        else if (typeC[0] == 'f')
        {
            double dval = 0.0;
            if (par->QueryDoubleAttribute("value", &dval) != TIXML_SUCCESS)
            {
                r.onError("Param index " + std::to_string(idx) + " missing float 'value'");
                return false;
            }
            if (!r.receiveFloatParam(idx, static_cast<float>(dval)))
            {
                r.onError("Receiver rejected float param at index " + std::to_string(idx));
                return false;
            }
        }
        else
        {
            r.onError("Param index " + std::to_string(idx) +
                      " has unknown type: " + std::string(typeC));
            return false;
        }
    }

    return true;
}

inline std::vector<std::string> factoryPresetPathsFor(const std::string &streamingName)
{
    return sst::effects::presets::recurseBelow("presets/effects/" + streamingName);
}

inline std::string factoryPresetContentByPath(const std::string &path)
{
    return sst::effects::presets::loadContent(path);
}

} // namespace sst::effects::presets
#endif

#endif // SHORTCIRCUITXT_VOICEEFFECTSPRESETSUPPORT_H
