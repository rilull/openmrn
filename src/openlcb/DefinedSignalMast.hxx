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
 * \file DefinedSignalMast.hxx
 *
 * Signal mast consumer that uses a compile-time aspect table to drive multiple
 * LEDs. Each aspect maps to a specific pattern of LED on/off/flashing states.
 * Events are configured via CDI; the aspect-to-LED mapping is fixed at compile
 * time.
 *
 * @author Rick Lull
 * @date 16 Mar 2026
 */

#ifndef _OPENLCB_DEFINEDSIGNALMAST_HXX_
#define _OPENLCB_DEFINEDSIGNALMAST_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/RefreshLoop.hxx"
#include "openlcb/SignalDefs.hxx"
#include "openlcb/SignalMastConfig.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

/// Signal mast consumer driven by a compile-time aspect table.
///
/// The template parameters define the physical mast geometry:
/// @tparam NUM_LEDS total number of LEDs (GPIO outputs) on the mast.
/// @tparam NUM_ASPECTS number of defined aspects (events) for the mast.
///
/// Usage: ```
///
/// // Define aspect table (constexpr, lives in flash).
/// // 8 LEDs: H1:G H1:Y H1:R H1:L  H2:G H2:Y H2:R H2:L
/// static constexpr SignalAspect<8> kMyAspects[] = {
///     // Clear:       upper=G, lower=R
///     {{I, O, O, O,  O, O, I, O}},
///     // Approach:    upper=Y, lower=R
///     {{O, I, O, O,  O, O, I, O}},
///     // Stop:        upper=R, lower=R
///     {{O, O, I, O,  O, O, I, O}},
/// };
///
/// static constexpr const char *const kMyAspectNames[] = {
///     "Clear", "Approach", "Stop",
/// };
///
/// constexpr const Gpio *const kMastGpio[] = {
///     H1G_Pin::instance(), H1Y_Pin::instance(), ...
/// };
///
/// openlcb::DefinedSignalMast<8, 3> my_mast(stack.node(),
///     kMastGpio, 8, kMyAspects, kMyAspectNames,
///     cfg.seg().my_signal_mast());
///
/// // Add to refresh loop for flashing support:
/// openlcb::RefreshLoop signal_loop(stack.node(),
///     {my_mast.polling()});
/// ```
template <unsigned NUM_LEDS, unsigned NUM_ASPECTS>
class DefinedSignalMast : public ConfigUpdateListener,
                          private SimpleEventHandler,
                          public Polling
{
public:
    using config_entry_type = SignalMastAspectConfig;

    /// Constructor.
    /// @param node is the OpenLCB node object from the stack.
    /// @param leds array of GPIO pointers for the LEDs. Can be constexpr.
    /// @param num_leds length of the leds array (must equal NUM_LEDS).
    /// @param aspects compile-time array of aspect definitions.
    /// @param aspect_names compile-time array of aspect name strings for
    ///        factory reset. May be nullptr to skip name initialization.
    /// @param config the repeated group from the CDI configuration.
    template <unsigned N>
    __attribute__((noinline)) DefinedSignalMast(Node *node,
        const Gpio *const *leds, unsigned num_leds,
        const SignalAspect<NUM_LEDS> *aspects, const char *const *aspect_names,
        const RepeatedGroup<config_entry_type, N> &config)
        : node_(node)
        , leds_(leds)
        , aspects_(aspects)
        , aspectNames_(aspect_names)
        , activeAspect_(NUM_ASPECTS) // no aspect active = all dark
        , flashPhase_(false)
        , flashCount_(0)
        , offset_(config)
    {
        HASSERT(num_leds == NUM_LEDS);
        HASSERT(N == NUM_ASPECTS);
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~DefinedSignalMast()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
    }

    /// Returns a Polling* pointer for use in RefreshLoop to support flashing.
    Polling *polling()
    {
        return this;
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
        BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            do_unregister();
        }
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < NUM_ASPECTS; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event = cfg_ref.event().read(fd);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event, i), 0);
        }
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < NUM_ASPECTS; ++i)
        {
            if (aspectNames_)
            {
                grp_ref.entry(i).description().write(fd, aspectNames_[i]);
            }
            else
            {
                grp_ref.entry(i).description().write(fd, "");
            }
        }
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->dst_node && event->dst_node != node_)
        {
            return done->notify();
        }
        SendConsumerIdentified(registry_entry, event, done);
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->event != registry_entry.event)
        {
            return done->notify();
        }
        SendConsumerIdentified(registry_entry, event, done);
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->event != registry_entry.event)
        {
            return done->notify();
        }

        unsigned aspect = registry_entry.user_arg;
        if (aspect < NUM_ASPECTS)
        {
            activeAspect_ = aspect;
            flashCount_ = 0;
            flashPhase_ = true;
            apply_aspect();
        }

        done->notify();
    }

    /// Called at ~33Hz by RefreshLoop. Handles flash toggling.
    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (activeAspect_ < NUM_ASPECTS && has_flashing_leds())
        {
            ++flashCount_;
            // Toggle at ~1Hz: ~17 ticks per half-cycle at 33Hz ≈ 515ms
            if (flashCount_ >= FLASH_TICKS)
            {
                flashCount_ = 0;
                flashPhase_ = !flashPhase_;
                apply_aspect();
            }
        }
        done->notify();
    }

private:
    /// Number of poll_33hz ticks per flash half-cycle.
    /// 17 ticks at 33Hz ≈ 515ms, giving ~60 flashes per minute.
    static constexpr uint8_t FLASH_TICKS = 17;

    /// Sentinel value meaning no aspect is active (all LEDs dark).
    static constexpr unsigned NO_ASPECT = NUM_ASPECTS;

    /// Applies the current aspect to the LED outputs.
    void apply_aspect()
    {
        if (activeAspect_ >= NUM_ASPECTS)
        {
            // No active aspect: all LEDs off.
            for (unsigned i = 0; i < NUM_LEDS; ++i)
            {
                leds_[i]->clr();
            }
            return;
        }
        const auto &aspect = aspects_[activeAspect_];
        for (unsigned i = 0; i < NUM_LEDS; ++i)
        {
            switch (aspect.leds[i])
            {
                case SignalLedState::ON:
                    leds_[i]->set();
                    break;
                case SignalLedState::FLASHING:
                    leds_[i]->write(flashPhase_ ? Gpio::SET : Gpio::CLR);
                    break;
                case SignalLedState::OFF:
                default:
                    leds_[i]->clr();
                    break;
            }
        }
    }

    /// Returns true if the current aspect has any flashing LEDs.
    bool has_flashing_leds() const
    {
        if (activeAspect_ >= NUM_ASPECTS)
        {
            return false;
        }
        const auto &aspect = aspects_[activeAspect_];
        for (unsigned i = 0; i < NUM_LEDS; ++i)
        {
            if (aspect.leds[i] == SignalLedState::FLASHING)
            {
                return true;
            }
        }
        return false;
    }

    /// Sends a ConsumerIdentified message for a registry entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        if (registry_entry.user_arg != activeAspect_)
        {
            mti++; // INVALID
        }
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done);
    }

    /// Unregisters all event handlers.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    Node *node_;                            ///< Virtual node.
    const Gpio *const *leds_;               ///< Array of LED GPIO pointers.
    const SignalAspect<NUM_LEDS> *aspects_;  ///< Compile-time aspect table.
    const char *const *aspectNames_;        ///< Aspect names for factory reset.
    unsigned activeAspect_;                 ///< Currently active aspect index.
    bool flashPhase_;                       ///< Current flash phase (on/off).
    uint8_t flashCount_;                    ///< Tick counter for flash timing.
    ConfigReference offset_;                ///< CDI config offset.
};

} // namespace openlcb

#endif // _OPENLCB_DEFINEDSIGNALMAST_HXX_
