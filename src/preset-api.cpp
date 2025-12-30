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

#include "preset-api.h"
#include "cmrc/cmrc.hpp"
#include <iostream>

CMRC_DECLARE(sst_effects_presets);

namespace sst::effects::presets
{

std::vector<std::string> recurseBelow(const std::string &parent)
{
    try
    {
        std::vector<std::string> res;
        auto fs = cmrc::sst_effects_presets::get_filesystem();
        for (const auto &c : fs.iterate_directory(parent))
        {
            res.push_back(parent + "/" + c.filename());
        }
        return res;
    }
    catch (std::exception &e)
    {
    }

    return {};
}

std::string loadContent(const std::string &s)
{
    try
    {
        std::vector<std::string> res;
        auto fs = cmrc::sst_effects_presets::get_filesystem();
        auto f = fs.open(s);
        return std::string(f.begin(), f.end());
    }
    catch (std::exception &e)
    {
    }
    return "";
}
} // namespace sst::effects::presets