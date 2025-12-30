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

#ifndef INCLUDE_SST_VOICE_EFFECTS_VOICEEFFECTSPRESETSUPPORT_H
#define INCLUDE_SST_VOICE_EFFECTS_VOICEEFFECTSPRESETSUPPORT_H

#if SST_EFFECTS_PRESETS
#include "tinyxml/tinyxml.h"

namespace sst::voice_effects::presets
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
    { a.receiveDeactivated(idx, b) } -> std::same_as<bool>;
    { a.receiveIntParam(idx, f) } -> std::same_as<bool>;
    { a.receiveTemposync(b) } -> std::same_as<bool>;
    { a.receiveKeytrack(b) } -> std::same_as<bool>;

    { a.onError(s) } -> std::same_as<void>;
};

template <typename RC>
concept PresetProvider = requires(RC a, size_t idx, float f, bool b, std::string s) {
    { a.provideFloatParamCount() } -> std::convertible_to<size_t>;
    { a.provideIntParamCount() } -> std::convertible_to<size_t>;
    { a.provideFloatParam(idx) } -> std::same_as<float>;
    { a.provideDeactivated(idx) } -> std::same_as<bool>;
    { a.provideIntParam(idx) } -> std::same_as<int>;
    { a.provideTemposync() } -> std::same_as<bool>;
    { a.provideKeytrack() } -> std::same_as<bool>;
    { a.provideStreamingVersion() } -> std::convertible_to<int>;
    { a.provideStreamingName() } -> std::convertible_to<std::string>;
};

inline std::string toPreset(const PresetProvider auto &pv)
{
    TiXmlDocument doc("sst-voice-effect");
    TiXmlElement rootNode("sst-voice-effect");
    rootNode.SetAttribute("effect-streaming-version", pv.provideStreamingVersion());
    rootNode.SetAttribute("preset-streaming-version", 1);
    rootNode.SetAttribute("effect", pv.provideStreamingName());
    rootNode.SetAttribute("temposync", pv.provideTemposync());
    rootNode.SetAttribute("keytrack", pv.provideKeytrack());

    for (int i = 0; i < pv.provideFloatParamCount(); ++i)
    {
        TiXmlElement fpar("float-param");
        fpar.SetAttribute("index", i);
        fpar.SetDoubleAttribute("value", pv.provideFloatParam(i));
        fpar.SetAttribute("deactivated", pv.provideDeactivated(i));
        rootNode.InsertEndChild(fpar);
    }

    for (int i = 0; i < pv.provideIntParamCount(); ++i)
    {
        TiXmlElement fpar("int-param");
        fpar.SetAttribute("index", i);
        fpar.SetDoubleAttribute("value", pv.provideIntParam(i));
        rootNode.InsertEndChild(fpar);
    }

    doc.InsertEndChild(rootNode);
    TiXmlPrinter printer;
    doc.Accept(&printer);

    return printer.Str();
}

inline bool fromPreset(const std::string &s, PresetReceiver auto &r)
{
    auto d = TiXmlDocument();
    if (!d.Parse(s.c_str()))
    {
        r.onError(std::string("XML Parse Error: ") + d.ErrorDesc());
        return false;
    }

    auto root = d.RootElement();
    if (!root)
    {
        r.onError("No root element found in preset XML");
        return false;
    }

    if (root->ValueStr() != "sst-voice-effect")
    {
        r.onError("Invalid root element: expected 'sst-voice-effect', got '" + root->ValueStr() +
                  "'");
        return false;
    }

    std::string sn;
    if (root->QueryStringAttribute("effect", &sn) != TIXML_SUCCESS)
    {
        r.onError("Root node contains no effect name");
        return false;
    }
    if (!r.canReceiveForStreamingName(sn))
    {
        r.onError("Effect type Mismatch. Receiver unable to receive streaming name " + sn);
        return false;
    }

    int fv;
    if (root->QueryIntAttribute("effect-streaming-version", &fv) != TIXML_SUCCESS)
    {
        r.onError("Root node contains no effect streaming version");
        return false;
    }
    if (!r.receiveStreamingVersion(fv))
    {
        r.onError("Receiver unable to receive streaming version");
        return false;
    }

    // Parse temposync attribute
    int temposyncVal = 0;
    if (root->QueryIntAttribute("temposync", &temposyncVal) == TIXML_SUCCESS)
    {
        if (!r.receiveTemposync(temposyncVal != 0))
        {
            r.onError("Failed to set temposync");
            return false;
        }
    }

    // Parse keytrack attribute
    int keytrackVal = 0;
    if (root->QueryIntAttribute("keytrack", &keytrackVal) == TIXML_SUCCESS)
    {
        if (!r.receiveKeytrack(keytrackVal != 0))
        {
            r.onError("Failed to set keytrack");
            return false;
        }
    }

    // Parse float parameters
    for (auto child = root->FirstChildElement("float-param"); child != nullptr;
         child = child->NextSiblingElement("float-param"))
    {
        int index = -1;
        if (child->QueryIntAttribute("index", &index) != TIXML_SUCCESS || index < 0)
        {
            r.onError("Invalid or missing index in float-param");
            return false;
        }

        double value = 0.0;
        if (child->QueryDoubleAttribute("value", &value) != TIXML_SUCCESS)
        {
            r.onError("Invalid or missing value in float-param at index " + std::to_string(index));
            return false;
        }

        if (!r.receiveFloatParam(index, static_cast<float>(value)))
        {
            r.onError("Failed to set float parameter at index " + std::to_string(index));
            return false;
        }

        int deactivated = 0;
        if (child->QueryIntAttribute("deactivated", &deactivated) == TIXML_SUCCESS)
        {
            if (!r.receiveDeactivated(index, deactivated != 0))
            {
                r.onError("Failed to set deactivated state at index " + std::to_string(index));
                return false;
            }
        }
    }

    // Parse int parameters
    for (auto child = root->FirstChildElement("int-param"); child != nullptr;
         child = child->NextSiblingElement("int-param"))
    {
        int index = -1;
        if (child->QueryIntAttribute("index", &index) != TIXML_SUCCESS || index < 0)
        {
            r.onError("Invalid or missing index in int-param");
            return false;
        }

        int value = 0;
        if (child->QueryIntAttribute("value", &value) != TIXML_SUCCESS)
        {
            r.onError("Invalid or missing value in int-param at index " + std::to_string(index));
            return false;
        }

        if (!r.receiveIntParam(index, value))
        {
            r.onError("Failed to set int parameter at index " + std::to_string(index));
            return false;
        }
    }

    return true;
}

} // namespace sst::voice_effects::presets
#endif

#endif // SHORTCIRCUITXT_VOICEEFFECTSPRESETSUPPORT_H
