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
 * \file SignalDefs.hxx
 *
 * Definitions for signal mast LED states and aspect tables used by the
 * DefinedSignalMast class. A signal mast is defined as a compile-time table
 * mapping aspects to LED output states.
 *
 * @author Rick Lull
 * @date 16 Mar 2026
 */

#ifndef _OPENLCB_SIGNALDEFS_HXX_
#define _OPENLCB_SIGNALDEFS_HXX_

#include <cstdint>

namespace openlcb
{

/// State of a single LED output within a signal aspect.
enum class SignalLedState : uint8_t
{
    OFF = 0,      ///< LED is off (dark).
    ON = 1,       ///< LED is on (steady).
    FLASHING = 2, ///< LED is flashing at the standard rate.
};

/// Short aliases for use in aspect table definitions.
static constexpr SignalLedState O = SignalLedState::OFF;
static constexpr SignalLedState I = SignalLedState::ON;
static constexpr SignalLedState F = SignalLedState::FLASHING;

/// Defines the LED output states for a single aspect of a signal mast.
/// @tparam NUM_LEDS total number of physical LEDs on the mast.
template <unsigned NUM_LEDS>
struct SignalAspect
{
    /// LED states for this aspect, indexed by physical LED position.
    SignalLedState leds[NUM_LEDS];
};

} // namespace openlcb

#endif // _OPENLCB_SIGNALDEFS_HXX_
