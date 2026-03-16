/** \copyright
 * Copyright (c) 2025, Rick Lull
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file SignalTypes_CSX1998.hxx
 *
 * Compile-time aspect tables for CSX Transportation Signal Rules (Jan 1998).
 * These tables define the LED output patterns for various signal mast
 * configurations based on the JMRI CSX-1998 signal definitions.
 *
 * @author Rick Lull
 * @date 16 Mar 2026
 */

#ifndef _OPENLCB_SIGNALTYPES_CSX1998_HXX_
#define _OPENLCB_SIGNALTYPES_CSX1998_HXX_

#include "openlcb/SignalDefs.hxx"

namespace openlcb
{

// ======================================================================
// CSX 1998 Double Head 4-4 (3-3 with Lunar) Color Light High Signal
// Based on JMRI CLS-3-3-hi + Restricting via lunar on lower head.
//
// LED layout (8 LEDs):
//   Index 0: Upper Head Green
//   Index 1: Upper Head Yellow
//   Index 2: Upper Head Red
//   Index 3: Upper Head Lunar White
//   Index 4: Lower Head Green
//   Index 5: Lower Head Yellow
//   Index 6: Lower Head Red
//   Index 7: Lower Head Lunar White
//
// 11 aspects.
// ======================================================================

/// Number of LEDs for the CSX 1998 double 4-light head mast.
static constexpr unsigned CSX_1998_44_NUM_LEDS = 8;
/// Number of aspects for the CSX 1998 double 4-light head mast.
static constexpr unsigned CSX_1998_44_NUM_ASPECTS = 11;

/// Aspect table for CSX 1998 double 4-light (with lunar) mast.
///                                          H1:G H1:Y H1:R H1:L H2:G H2:Y H2:R H2:L
static constexpr SignalAspect<8> CSX_1998_44_ASPECTS[] = {
    /* Clear             */ {{  I,   O,   O,   O,   O,   O,   I,   O}},
    /* Approach Limited  */ {{  O,   I,   O,   O,   F,   O,   O,   O}},
    /* Limited Clear     */ {{  O,   O,   I,   O,   F,   O,   O,   O}},
    /* Limited Approach  */ {{  O,   O,   I,   O,   O,   F,   O,   O}},
    /* Approach Medium   */ {{  O,   I,   O,   O,   I,   O,   O,   O}},
    /* Advance Approach  */ {{  O,   I,   O,   O,   O,   I,   O,   O}},
    /* Medium Clear      */ {{  O,   O,   I,   O,   I,   O,   O,   O}},
    /* Approach          */ {{  O,   I,   O,   O,   O,   O,   I,   O}},
    /* Medium Approach   */ {{  O,   O,   I,   O,   O,   I,   O,   O}},
    /* Restricting       */ {{  O,   O,   I,   O,   O,   O,   O,   I}},
    /* Stop              */ {{  O,   O,   I,   O,   O,   O,   I,   O}},
};

/// Aspect names for the CSX 1998 double 4-light mast (for CDI factory reset).
static constexpr const char *const CSX_1998_44_ASPECT_NAMES[] = {
    "Clear",
    "Approach Limited",
    "Limited Clear",
    "Limited Approach",
    "Approach Medium",
    "Advance Approach",
    "Medium Clear",
    "Approach",
    "Medium Approach",
    "Restricting",
    "Stop",
};

// ======================================================================
// CSX 1998 Triple Head 3-3-2A Color Light High Signal
// Based on JMRI CLS-3-3-2A-hi.
//
// LED layout (8 LEDs):
//   Index 0: Upper Head Green
//   Index 1: Upper Head Yellow
//   Index 2: Upper Head Red
//   Index 3: Middle Head Green
//   Index 4: Middle Head Yellow
//   Index 5: Middle Head Red
//   Index 6: Lower Head Yellow
//   Index 7: Lower Head Red
//
// 12 aspects.
// ======================================================================

/// Number of LEDs for the CSX 1998 triple head 3-3-2A mast.
static constexpr unsigned CSX_1998_332A_NUM_LEDS = 8;
/// Number of aspects for the CSX 1998 triple head 3-3-2A mast.
static constexpr unsigned CSX_1998_332A_NUM_ASPECTS = 12;

/// Aspect table for CSX 1998 triple head 3-3-2A mast.
///                                              H1:G H1:Y H1:R H2:G H2:Y H2:R H3:Y H3:R
static constexpr SignalAspect<8> CSX_1998_332A_ASPECTS[] = {
    /* Clear                  */ {{  I,   O,   O,   O,   O,   I,   O,   I}},
    /* Approach Limited       */ {{  O,   I,   O,   F,   O,   O,   O,   I}},
    /* Limited Clear          */ {{  O,   O,   I,   F,   O,   O,   O,   I}},
    /* Limited Approach       */ {{  O,   O,   I,   O,   F,   O,   O,   I}},
    /* Approach Medium        */ {{  O,   I,   O,   I,   O,   O,   O,   I}},
    /* Advance Approach       */ {{  O,   I,   O,   O,   I,   O,   O,   I}},
    /* Medium Clear           */ {{  O,   O,   I,   I,   O,   O,   O,   I}},
    /* Medium Advance Approach*/ {{  O,   O,   I,   O,   I,   O,   I,   O}},
    /* Approach               */ {{  O,   I,   O,   O,   O,   I,   O,   I}},
    /* Medium Approach        */ {{  O,   O,   I,   O,   I,   O,   O,   I}},
    /* Slow Approach          */ {{  O,   O,   I,   O,   O,   I,   I,   O}},
    /* Stop                   */ {{  O,   O,   I,   O,   O,   I,   O,   I}},
};

/// Aspect names for the CSX 1998 triple 3-3-2A mast (for CDI factory reset).
static constexpr const char *const CSX_1998_332A_ASPECT_NAMES[] = {
    "Clear",
    "Approach Limited",
    "Limited Clear",
    "Limited Approach",
    "Approach Medium",
    "Advance Approach",
    "Medium Clear",
    "Medium Adv Approach",
    "Approach",
    "Medium Approach",
    "Slow Approach",
    "Stop",
};

} // namespace openlcb

#endif // _OPENLCB_SIGNALTYPES_CSX1998_HXX_
