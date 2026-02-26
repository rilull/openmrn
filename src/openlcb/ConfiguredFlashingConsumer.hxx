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
 * multi-mode output supporting steady on, flashing with configurable duty
 * cycle, and off states. Each state is activated by its own event.
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
/// Specifies the event ID to turn the output on steady.
CDI_GROUP_ENTRY(event_steady, EventConfigEntry, //
    Name("Event Steady On"),
    Description("Receiving this event ID will turn the output on steady."));
/// Specifies the event ID to start the output flashing.
CDI_GROUP_ENTRY(event_flashing, EventConfigEntry, //
    Name("Event Flashing"),
    Description("Receiving this event ID will start the output flashing."));
/// Specifies the event ID to turn the output off.
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("Receiving this event ID will turn the output off."));
/// Allows the user to configure the flash on-time.
CDI_GROUP_ENTRY(on_period, Uint8ConfigEntry, Default(17), Min(1), Max(250), //
    Name("Flash on time"),
    Description("Duration the output stays on during each flash cycle, "
                "in units of 30 msec. Default of 17 gives approximately "
                "500 msec on time."));
/// Allows the user to configure the flash off-time.
CDI_GROUP_ENTRY(off_period, Uint8ConfigEntry, Default(17), Min(1), Max(250), //
    Name("Flash off time"),
    Description("Duration the output stays off during each flash cycle, "
                "in units of 30 msec. Default of 17 gives approximately "
                "500 msec off time. Together with the default on time, "
                "this produces approximately 1 Hz flashing."));
CDI_GROUP_END();

/// OpenLCB Consumer class integrating a CDI-based configuration for a GPIO
/// output that supports three modes: steady on, flashing with configurable
/// duty cycle, and off.
///
/// Three events control the output:
/// - The "steady on" event turns the output on and holds it.
/// - The "flashing" event starts the output toggling at a rate set by the
///   on_period and off_period CDI entries.
/// - The "off" event turns the output off.
///
/// The on_period and off_period allow an asymmetric duty cycle, which is
/// useful for effects beyond standard railroad signal flashing (beacons,
/// flickering lights, warning strobes, etc.).
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
    /// @param gpio is the output GPIO pin to drive.
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

        EventId cfg_event_steady = cfg_.event_steady().read(fd);
        EventId cfg_event_flashing = cfg_.event_flashing().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        uint8_t cfg_on_period = cfg_.on_period().read(fd);
        uint8_t cfg_off_period = cfg_.off_period().read(fd);
        if (cfg_on_period < 1)
        {
            cfg_on_period = 1;
        }
        if (cfg_off_period < 1)
        {
            cfg_off_period = 1;
        }
        onPeriod_ = cfg_on_period;
        offPeriod_ = cfg_off_period;

        if (cfg_event_steady == eventSteady_ &&
            cfg_event_flashing == eventFlashing_ &&
            cfg_event_off == eventOff_)
        {
            return UPDATED;
        }

        if (!initial_load)
        {
            do_unregister();
        }

        eventSteady_ = cfg_event_steady;
        eventFlashing_ = cfg_event_flashing;
        eventOff_ = cfg_event_off;
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventSteady_, USER_ARG_STEADY), 0);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventFlashing_, USER_ARG_FLASHING), 0);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, eventOff_, USER_ARG_OFF), 0);
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        cfg_.description().write(fd, "");
        CDI_FACTORY_RESET(cfg_.on_period);
        CDI_FACTORY_RESET(cfg_.off_period);
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
        if (event->event == eventSteady_)
        {
            state_ = STATE_STEADY;
            pinState_ = true;
            remaining_ = 0;
            gpio_->set();
        }
        else if (event->event == eventFlashing_)
        {
            state_ = STATE_FLASHING;
            pinState_ = true;
            remaining_ = onPeriod_;
            gpio_->set();
        }
        else if (event->event == eventOff_)
        {
            state_ = STATE_OFF;
            pinState_ = false;
            remaining_ = 0;
            gpio_->clr();
        }
        done->notify();
    }

    // Polling interface.

    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (state_ == STATE_FLASHING)
        {
            if (remaining_ > 0)
            {
                --remaining_;
            }
            else
            {
                pinState_ = !pinState_;
                if (pinState_)
                {
                    gpio_->set();
                    remaining_ = onPeriod_;
                }
                else
                {
                    gpio_->clr();
                    remaining_ = offPeriod_;
                }
            }
        }
        done->notify();
    }

    /// @return true if the consumer is currently in the flashing state.
    bool is_flashing() const
    {
        return state_ == STATE_FLASHING;
    }

    /// @return true if the consumer is currently in the steady-on state.
    bool is_steady() const
    {
        return state_ == STATE_STEADY;
    }

private:
    /// Output state values.
    enum State : uint8_t
    {
        STATE_OFF = 0,
        STATE_STEADY = 1,
        STATE_FLASHING = 2,
    };

    /// User arg value for the OFF event.
    static constexpr unsigned USER_ARG_OFF = 0;
    /// User arg value for the STEADY ON event.
    static constexpr unsigned USER_ARG_STEADY = 1;
    /// User arg value for the FLASHING event.
    static constexpr unsigned USER_ARG_FLASHING = 2;

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        bool is_valid = false;
        switch (registry_entry.user_arg)
        {
            case USER_ARG_OFF:
                is_valid = (state_ == STATE_OFF);
                break;
            case USER_ARG_STEADY:
                is_valid = (state_ == STATE_STEADY);
                break;
            case USER_ARG_FLASHING:
                is_valid = (state_ == STATE_FLASHING);
                break;
        }
        if (!is_valid)
        {
            mti++; // INVALID
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
    const Gpio *gpio_;                 //< hardware output pin to drive
    const FlashingConsumerConfig cfg_; //< CDI configuration reference
    EventId eventSteady_{0};           //< event ID for steady on
    EventId eventFlashing_{0};         //< event ID for start flashing
    EventId eventOff_{0};              //< event ID for off
    uint8_t onPeriod_{17};             //< flash on-time in 33Hz ticks
    uint8_t offPeriod_{17};            //< flash off-time in 33Hz ticks
    uint8_t remaining_{0};             //< ticks remaining in current half
    State state_{STATE_OFF};           //< current output state
    bool pinState_{false};             //< current output level
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDFLASHINGCONSUMER_HXX_
