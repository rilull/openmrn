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
 * \file SignalMastConfig.hxx
 *
 * CDI configuration groups for the DefinedSignalMast class.
 *
 * @author Rick Lull
 * @date 16 Mar 2026
 */

#ifndef _OPENLCB_SIGNALMASTCONFIG_HXX_
#define _OPENLCB_SIGNALMASTCONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace openlcb
{

/// CDI configuration for a single aspect of a defined signal mast.
/// Each aspect has a user-visible description (populated at factory reset
/// from the compile-time aspect name) and an event that selects this aspect.
CDI_GROUP(SignalMastAspectConfig);
/// User-visible description of this aspect (e.g. "Clear", "Approach").
CDI_GROUP_ENTRY(description, StringConfigEntry<20>, //
    Name("Aspect"), Description("Name of this signal aspect."));
/// Event that selects this aspect.
CDI_GROUP_ENTRY(event, EventConfigEntry, //
    Name("Event"),
    Description(
        "Receiving this event ID will display this aspect on the mast."));
CDI_GROUP_END();

} // namespace openlcb

#endif // _OPENLCB_SIGNALMASTCONFIG_HXX_
