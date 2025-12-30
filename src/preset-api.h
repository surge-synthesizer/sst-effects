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

#ifndef SST_EFFECTS_INTERNAL_PRESET_API
#define SST_EFFECTS_INTERNAL_PRESET_API

#include <string>
#include <vector>

namespace sst::effects::presets
{
std::vector<std::string> recurseBelow(const std::string &parent);
std::string loadContent(const std::string &);
} // namespace sst::effects::presets

#endif
