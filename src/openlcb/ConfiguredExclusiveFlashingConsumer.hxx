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
 * \file ConfiguredExclusiveFlashingConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a group of GPIO pins to
 * implement mutually exclusive output selection where each output can be
 * activated in either steady or flashing mode. Receiving any event turns off
 * all other outputs and sets the selected output to steady on or flashing.
 *
 * @author Balazs Racz
 * @date 26 Feb 2025
 */

#ifndef _OPENLCB_CONFIGUREDEXCLUSIVEFLASHINGCONSUMER_HXX_
#define _OPENLCB_CONFIGUREDEXCLUSIVEFLASHINGCONSUMER_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/RefreshLoop.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

/// CDI Configuration for one output within an exclusive flashing group.
/// Each output gets two events: one for steady on, one for flashing.
/// When any event is received, all other outputs in the group are turned off.
CDI_GROUP(ExclusiveFlashingConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<8>, //
    Name("Description"), Description("User name of this output."));
/// Specifies the event ID to select this output in steady mode.
CDI_GROUP_ENTRY(event, EventConfigEntry, //
    Name("Event Steady On"),
    Description("Receiving this event ID will turn this output on steady "
                "and turn off all other outputs in the group."));
/// Specifies the event ID to select this output in flashing mode.
CDI_GROUP_ENTRY(event_flashing, EventConfigEntry, //
    Name("Event Flashing"),
    Description("Receiving this event ID will start this output flashing "
                "and turn off all other outputs in the group."));
/// Flash on-time for this output.
CDI_GROUP_ENTRY(on_period, Uint8ConfigEntry, Default(17), Min(1), Max(250), //
    Name("Flash on time"),
    Description("Duration the output stays on during each flash cycle, "
                "in units of 30 msec. Default of 17 gives approximately "
                "500 msec on time."));
/// Flash off-time for this output.
CDI_GROUP_ENTRY(off_period, Uint8ConfigEntry, Default(17), Min(1), Max(250), //
    Name("Flash off time"),
    Description("Duration the output stays off during each flash cycle, "
                "in units of 30 msec. Default of 17 gives approximately "
                "500 msec off time."));
CDI_GROUP_END();

/// Consumer class for a group of GPIO outputs where exactly one output is
/// active at a time, and each output can operate in either steady or flashing
/// mode. When any event is received, all other outputs in the group are turned
/// off, and the selected output is either turned on steady or starts flashing.
///
/// This is useful for signal masts where aspects include both steady and
/// flashing indications (e.g., red, flashing red, yellow, flashing yellow,
/// green, flashing green, lunar).
///
/// The class implements the @ref Polling interface and must be registered
/// with a @ref RefreshLoop to receive the 33 Hz tick needed for flash timing.
///
/// Usage: ```
///
/// constexpr const Gpio *const kSignalGpio[] = {
///     RED_Pin::instance(), YELLOW_Pin::instance(), GREEN_Pin::instance(),
/// };
/// openlcb::ConfiguredExclusiveFlashingConsumer signal_consumer(
///     stack.node(), kSignalGpio, ARRAYSIZE(kSignalGpio),
///     cfg.seg().signal_aspects());
///
/// // Register in a RefreshLoop.
/// openlcb::RefreshLoop loop(stack.node(),
///     {&signal_consumer, &other_polled});
/// ```
class ConfiguredExclusiveFlashingConsumer : public ConfigUpdateListener,
                                            private SimpleEventHandler,
                                            public Polling
{
public:
    typedef ExclusiveFlashingConsumerConfig config_entry_type;

    /// @param node is the OpenLCB node object from the stack.
    /// @param pins is the list of pins represented by the Gpio* object
    /// instances. Can be constant from FLASH space.
    /// @param size is the length of the list of pins array.
    /// @param config is the repeated group object from the configuration space
    /// that represents the locations of the events.
    template <unsigned N>
    __attribute__((noinline)) ConfiguredExclusiveFlashingConsumer(Node *node,
        const Gpio *const *pins, unsigned size,
        const RepeatedGroup<config_entry_type, N> &config)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , activeIndex_(NONE_ACTIVE)
        , activeFlashing_(false)
        , pinState_(false)
        , remaining_(0)
        , onPeriod_(17)
        , offPeriod_(17)
        , offset_(config)
    {
        HASSERT(size == N);
        onPeriods_ = new uint8_t[N];
        offPeriods_ = new uint8_t[N];
        memset(onPeriods_, 17, N);
        memset(offPeriods_, 17, N);
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredExclusiveFlashingConsumer()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
        delete[] onPeriods_;
        delete[] offPeriods_;
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
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event_steady = cfg_ref.event().read(fd);
            EventId cfg_event_flashing = cfg_ref.event_flashing().read(fd);
            uint8_t on_p = cfg_ref.on_period().read(fd);
            uint8_t off_p = cfg_ref.off_period().read(fd);
            if (on_p < 1)
            {
                on_p = 1;
            }
            if (off_p < 1)
            {
                off_p = 1;
            }
            onPeriods_[i] = on_p;
            offPeriods_[i] = off_p;

            // user_arg encoding: i * 2 for steady, i * 2 + 1 for flashing.
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_steady, i * 2), 0);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_flashing, i * 2 + 1), 0);
        }
        return REINIT_NEEDED;
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).description().write(fd, "");
            CDI_FACTORY_RESET(grp_ref.entry(i).on_period);
            CDI_FACTORY_RESET(grp_ref.entry(i).off_period);
        }
    }

    /// Factory reset helper function. Sets all names to something 1..N.
    /// @param fd passed on from factory reset argument.
    /// @param basename name of repeats.
    void factory_reset_names(int fd, const char *basename)
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            string v(basename);
            v.push_back(' ');
            char buf[10];
            unsigned_integer_to_buffer(i + 1, buf);
            v += buf;
            grp_ref.entry(i).description().write(fd, v);
        }
    }

    // SimpleEventHandler implementations.

    void handle_identify_global(const EventRegistryEntry &registry_entry,
                              EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        if (event->dst_node && event->dst_node != node_)
        {
            return done->notify();
        }
        SendConsumerIdentified(registry_entry, event, done);
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
                                EventReport *event, BarrierNotifiable *done)
        OVERRIDE
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

        unsigned output_idx = registry_entry.user_arg / 2;
        bool is_flash = (registry_entry.user_arg & 1) != 0;

        // Turn off all outputs in the group.
        for (unsigned i = 0; i < size_; ++i)
        {
            pins_[i]->clr();
        }

        activeIndex_ = output_idx;
        activeFlashing_ = is_flash;

        if (is_flash)
        {
            // Start flashing: begin with pin on.
            onPeriod_ = onPeriods_[output_idx];
            offPeriod_ = offPeriods_[output_idx];
            pinState_ = true;
            remaining_ = onPeriod_;
            pins_[output_idx]->set();
        }
        else
        {
            // Steady on.
            pinState_ = true;
            remaining_ = 0;
            pins_[output_idx]->set();
        }

        done->notify();
    }

    // Polling interface.

    void poll_33hz(WriteHelper *helper, Notifiable *done) OVERRIDE
    {
        if (activeIndex_ != NONE_ACTIVE && activeFlashing_)
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
                    pins_[activeIndex_]->set();
                    remaining_ = onPeriod_;
                }
                else
                {
                    pins_[activeIndex_]->clr();
                    remaining_ = offPeriod_;
                }
            }
        }
        done->notify();
    }

private:
    /// Sentinel value meaning no output is currently active.
    static constexpr unsigned NONE_ACTIVE = UINT_MAX;

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        unsigned output_idx = registry_entry.user_arg / 2;
        bool is_flash_event = (registry_entry.user_arg & 1) != 0;

        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        bool is_valid =
            (output_idx == activeIndex_) &&
            (is_flash_event == activeFlashing_);
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

    Node *node_;              //< virtual node to export the consumer on
    const Gpio *const *pins_; //< array of all GPIO pins to use
    size_t size_;             //< number of GPIO pins to export
    unsigned activeIndex_;    //< currently active output, or NONE_ACTIVE
    bool activeFlashing_;     //< true if the active output is flashing
    bool pinState_;           //< current pin level when flashing
    uint8_t remaining_;       //< ticks remaining in current flash half-cycle
    uint8_t onPeriod_;        //< on-time for currently active flash
    uint8_t offPeriod_;       //< off-time for currently active flash
    uint8_t *onPeriods_;      //< cached per-output on periods
    uint8_t *offPeriods_;     //< cached per-output off periods
    ConfigReference offset_;  //< offset in the configuration space
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDEXCLUSIVEFLASHINGCONSUMER_HXX_
