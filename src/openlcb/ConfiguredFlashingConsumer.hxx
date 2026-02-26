/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file ConfiguredFlashingConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a GPIO pin to implement a
 * flashing (blinking) output. Receiving the "on" event starts the output
 * flashing at a configurable rate. Receiving the "off" event stops flashing
 * and turns the output off.
 *
 * @author Balazs Racz
 * @date 26 Feb 2025
 */

#ifndef _OPENLCB_CONFIGUREDFLASHINGCONSUMER_HXX_
#define _OPENLCB_CONFIGUREDFLASHINGCONSUMER_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/RefreshLoop.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"

namespace openlcb
{

/// CDI Configuration for a @ref ConfiguredFlashingConsumer.
CDI_GROUP(FlashingConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<8>, //
    Name("Description"), Description("User name of this output."));
/// Specifies the event ID to start the output flashing.
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Event On"),
    Description("Receiving this event ID will start the output flashing."));
/// Specifies the event ID to stop the output flashing.
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("Receiving this event ID will stop the output flashing "
                "and turn it off."));
/// Allows the user to configure the flash rate.
CDI_GROUP_ENTRY(period, Uint8ConfigEntry, Default(17), Min(1), Max(250), //
    Name("Flash period"),
    Description("Half-period of the flash in 30 msec units. Default of 17 "
                "gives approximately 1 Hz (prototype railroad signal rate). "
                "A value of 8 gives approximately 2 Hz."));
CDI_GROUP_END();

/// OpenLCB Consumer class integrating a CDI-based configuration for a GPIO
/// output that flashes (toggles on and off) at a configurable rate.
///
/// When the "on" event is received, the output begins flashing at the
/// configured rate. When the "off" event is received, flashing stops and the
/// output is turned off.
///
/// This is useful for railroad signal aspects that require a flashing
/// indication (e.g., flashing red, flashing yellow, flashing lunar).
///
/// The class implements the @ref Polling interface and must be registered
/// with a @ref RefreshLoop to receive the 33 Hz tick needed for timing.
///
/// Usage: ```
///
/// openlcb::ConfiguredFlashingConsumer flash_red(node,
///     cfg.seg().flash_red(), RED_Pin::instance());
///
/// // Register in a RefreshLoop along with other polled objects.
/// openlcb::RefreshLoop loop(node, {&flash_red, &other_polled});
/// ```
class ConfiguredFlashingConsumer : public ConfigUpdateListener,
                                   private SimpleEventHandler,
                                   public Polling
{
public:
    /// Constructor.
    ///
    /// @param node is the OpenLCB node object from the stack.
    /// @param cfg is the configuration entry from the CDI.
    /// @param gpio is the output GPIO pin to flash.
    ConfiguredFlashingConsumer(
        Node *node, const FlashingConsumerConfig &cfg, const Gpio *gpio)
        : node_(node)
        , gpio_(gpio)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    /// Constructor accepting a hardware pin template type.
    template <class HW>
    ConfiguredFlashingConsumer(Node *node, const FlashingConsumerConfig &cfg,
        const HW &, const Gpio *g = HW::instance())
        : node_(node)
        , gpio_(g)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredFlashingConsumer()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
        gpio_->clr();
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        uint8_t cfg_period = cfg_.period().read(fd);
        if (cfg_period < 1)
        {
            cfg_period = 1;
        }
        period_ = cfg_period;

        if (cfg_event_on == eventOn_ && cfg_event_off == eventOff_)
        {
            return UPDATED;
        }

        if (!initial_load)
        {
            do_unregister();
        }

        eventOn_ = cfg_event_on;
        eventOff_ = cfg_event_off;
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventOn_, USER_ARG_ON), 0);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventOff_, USER_ARG_OFF), 0);
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.period);
    }

    // SimpleEventHandler implementations.

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
        if (event->event == eventOn_)
        {
            flashing_ = true;
            pinState_ = true;
            remaining_ = period_;
            gpio_->set();
        }
        else if (event->event == eventOff_)
        {
            flashing_ = false;
            pinState_ = false;
            remaining_ = 0;
            gpio_->clr();
        }
        done->notify();
    }

    // Polling interface.

    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (flashing_)
        {
            if (remaining_ > 0)
            {
                --remaining_;
            }
            else
            {
                remaining_ = period_;
                pinState_ = !pinState_;
                if (pinState_)
                {
                    gpio_->set();
                }
                else
                {
                    gpio_->clr();
                }
            }
        }
        done->notify();
    }

    /// @return true if the consumer is currently in the flashing state.
    bool is_flashing() const
    {
        return flashing_;
    }

private:
    /// User arg value for the ON (start flashing) event.
    static constexpr unsigned USER_ARG_ON = 1;
    /// User arg value for the OFF (stop flashing) event.
    static constexpr unsigned USER_ARG_OFF = 0;

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        if (registry_entry.user_arg == USER_ARG_ON)
        {
            // The "on" event is VALID when flashing is active.
            if (!flashing_)
            {
                mti++; // INVALID
            }
        }
        else
        {
            // The "off" event is VALID when flashing is not active.
            if (flashing_)
            {
                mti++; // INVALID
            }
        }
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done);
    }

    /// Removes registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    Node *node_;                       //< virtual node to export the consumer on
    const Gpio *gpio_;                 //< hardware output pin to flash
    const FlashingConsumerConfig cfg_; //< CDI configuration reference
    EventId eventOn_{0};               //< event ID for start flashing
    EventId eventOff_{0};              //< event ID for stop flashing
    uint8_t period_{17};               //< half-period in 33Hz ticks
    uint8_t remaining_{0};             //< ticks remaining in current half
    bool flashing_{false};             //< true when output is flashing
    bool pinState_{false};             //< current output level
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDFLASHINGCONSUMER_HXX_
