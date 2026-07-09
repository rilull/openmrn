/** \copyright
 * Copyright (c) 2026, Rick Lull
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
 * \file ConfiguredRoutedConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a group of GPIO pins to drive
 * turnout (switch machine) motors. Each output has individual CLOSED and THROWN
 * events for granular control, plus a route event. Receiving an output's route
 * event moves that turnout THROWN and moves every other turnout in the same
 * route group to CLOSED, allowing a single Event ID to line an entire path
 * (for example a yard ladder).
 *
 * Output changes are staggered by a configurable delay so that only a bounded
 * number of switch machine motors are ever moving (and drawing current) at the
 * same time. This is important for machines such as the MP10 that draw current
 * only while traveling; spacing out the movements keeps peak current within a
 * node's power budget.
 *
 * @author Rick Lull and Claude Code
 * @date 9 Jul 2026
 */

#ifndef _OPENLCB_CONFIGUREDROUTEDCONSUMER_HXX_
#define _OPENLCB_CONFIGUREDROUTEDCONSUMER_HXX_

#include "executor/Timer.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/Node.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

/// Dropdown map for the route group selector. Value 0 means the output does not
/// participate in route logic; values 1..16 assign the output to a route group.
static const char ROUTED_GROUP_MAP[] =
    "<relation><property>0</property><value>None</value></relation>"
    "<relation><property>1</property><value>Group 1</value></relation>"
    "<relation><property>2</property><value>Group 2</value></relation>"
    "<relation><property>3</property><value>Group 3</value></relation>"
    "<relation><property>4</property><value>Group 4</value></relation>"
    "<relation><property>5</property><value>Group 5</value></relation>"
    "<relation><property>6</property><value>Group 6</value></relation>"
    "<relation><property>7</property><value>Group 7</value></relation>"
    "<relation><property>8</property><value>Group 8</value></relation>"
    "<relation><property>9</property><value>Group 9</value></relation>"
    "<relation><property>10</property><value>Group 10</value></relation>"
    "<relation><property>11</property><value>Group 11</value></relation>"
    "<relation><property>12</property><value>Group 12</value></relation>"
    "<relation><property>13</property><value>Group 13</value></relation>"
    "<relation><property>14</property><value>Group 14</value></relation>"
    "<relation><property>15</property><value>Group 15</value></relation>"
    "<relation><property>16</property><value>Group 16</value></relation>";

/// Group value meaning the output does not participate in route logic.
static constexpr uint8_t ROUTED_GROUP_NONE = 0;

/// Minimum movement stagger delay, in units of 100 ms. Configured values below
/// this floor are clamped up to it so the consumer can never step output
/// changes faster than this safe rate.
static constexpr uint16_t ROUTED_MIN_STAGGER_DELAY = 10; // 1.0 s

/// CDI Configuration for one turnout output in a routed group. Each output has
/// individual CLOSED and THROWN events for granular control, a route event for
/// path selection, and a route group membership selector.
CDI_GROUP(RoutedConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<16>, //
    Name("Description"), Description("User name of this turnout."));
/// Selects the route group that this output belongs to.
CDI_GROUP_ENTRY(group, Uint8ConfigEntry, Default(ROUTED_GROUP_NONE),
    MapValues(ROUTED_GROUP_MAP), Name("Route Group"),
    Description("Route group membership. When a route event is received, all "
                "outputs sharing the selected group are moved to CLOSED except "
                "the one whose route event was received, which is moved to "
                "THROWN. Choose None to exclude this output from route logic."));
/// Specifies the event ID to move this turnout to CLOSED.
CDI_GROUP_ENTRY(event_closed, EventConfigEntry, //
    Name("Closed Event"),
    Description("Receiving this event ID will move this turnout to CLOSED."));
/// Specifies the event ID to move this turnout to THROWN.
CDI_GROUP_ENTRY(event_thrown, EventConfigEntry, //
    Name("Thrown Event"),
    Description("Receiving this event ID will move this turnout to THROWN."));
/// Specifies the route event ID for this turnout.
CDI_GROUP_ENTRY(event_route, EventConfigEntry, //
    Name("Route Event"),
    Description("Receiving this event ID will move this turnout to THROWN and "
                "move all other turnouts in its route group to CLOSED."));
CDI_GROUP_END();

/// Consumer class for a group of GPIO outputs that drive turnout motors. Each
/// output is a single GPIO pin (for example driving a TC4428 that provides the
/// differential outputs needed by a stall motor such as a Tortoise, or an MP10
/// in 2-wire mode). The output is driven constant-on with fixed polarity:
///
///   - GPIO set()  -> turnout THROWN
///   - GPIO clr()  -> turnout CLOSED
///
/// Every output exposes three consumer events: an individual CLOSED event, an
/// individual THROWN event, and a route event. The route event moves this
/// turnout to THROWN and moves every other turnout in the same route group to
/// CLOSED. This lets a single Event ID line an entire path -- for example a yard
/// ladder where one button selects a track and all feeding turnouts line
/// automatically. Outputs assigned the "None" group do not participate in route
/// logic; their route event simply throws themselves.
///
/// To bound the peak current from simultaneously moving motors, output changes
/// are not applied all at once. Instead they are committed one at a time, with a
/// configurable stagger delay between successive changes (a global throttle
/// applied to individual events and route events alike). When a route selects a
/// track, the selected (THROWN) output is committed first, then its siblings
/// close one by one, spaced by the stagger delay. The delay is read from a
/// Uint16ConfigEntry in units of 100 ms and is clamped up to a safe minimum
/// (@ref ROUTED_MIN_STAGGER_DELAY). Set the delay to at least a machine's travel
/// time (roughly 3 s for an MP10) if you want only one motor moving at a time;
/// shorter values allow bounded overlap and therefore higher peak current.
///
/// Usage: ```
///
/// constexpr const Gpio *const kTurnoutGpio[] = {
///     TDRV1_Pin::instance(), TDRV2_Pin::instance(),
///     TDRV3_Pin::instance(), TDRV4_Pin::instance(),
/// };
/// openlcb::ConfiguredRoutedConsumer turnout_consumer(stack.node(),
///    kTurnoutGpio, ARRAYSIZE(kTurnoutGpio),
///    cfg.seg().turnout_consumers(), cfg.seg().turnout_stagger_delay());
/// ```
class ConfiguredRoutedConsumer : public ConfigUpdateListener,
                                 private SimpleEventHandler
{
public:
    typedef RoutedConsumerConfig config_entry_type;

    /// Event type encoded in the low bits of the registry entry user_arg.
    enum EventType
    {
        EVENT_CLOSED = 0, //< individual close event
        EVENT_THROWN = 1, //< individual throw event
        EVENT_ROUTE = 2,  //< route (throw self, close group siblings) event
    };

    /// @param node is the OpenLCB node object from the stack.
    /// @param pins is the list of pins represented by the Gpio* object
    /// instances. Can be constant from FLASH space.
    /// @param size is the length of the list of pins array.
    /// @param config is the repeated group object from the configuration space
    /// that represents the locations of the events.
    /// @param stagger_delay is the configuration entry holding the movement
    /// stagger delay in units of 100 ms (clamped up to @ref
    /// ROUTED_MIN_STAGGER_DELAY).
    template <unsigned N>
    __attribute__((noinline)) ConfiguredRoutedConsumer(Node *node,
        const Gpio *const *pins, unsigned size,
        const RepeatedGroup<config_entry_type, N> &config,
        const Uint16ConfigEntry &stagger_delay)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , groups_(new uint8_t[N]())
        , desiredState_(new uint8_t[N]())
        , timer_(this, node)
        , offset_(config)
        , delayOffset_(stagger_delay)
    {
        // Mismatched sizing of the GPIO array from the configuration array.
        HASSERT(size == N);
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredRoutedConsumer()
    {
        do_unregister();
        timer_.cancel();
        ConfigUpdateService::instance()->unregister_update_listener(this);
        delete[] groups_;
        delete[] desiredState_;
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            // There is no way to figure out what the previously registered
            // eventid values were for the individual pins. Therefore we always
            // unregister everything and register them anew. It also causes us
            // to identify all. This is not a problem since apply_configuration
            // is coming from a user action.
            do_unregister();
        }
        // Read and clamp the movement stagger delay (100 ms units).
        uint16_t delay_units =
            Uint16ConfigEntry(delayOffset_.offset()).read(fd);
        if (delay_units < ROUTED_MIN_STAGGER_DELAY)
        {
            delay_units = ROUTED_MIN_STAGGER_DELAY;
        }
        delayNsec_ = MSEC_TO_NSEC((long long)delay_units * 100);

        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            // Cache the group membership so the event report hot path never
            // reads the configuration file.
            groups_[i] = cfg_ref.group().read(fd);
            // Sync the desired state to the current physical state so a
            // reconfiguration never causes a spurious movement.
            desiredState_[i] = pins_[i]->is_set() ? 1 : 0;
            EventId cfg_closed = cfg_ref.event_closed().read(fd);
            EventId cfg_thrown = cfg_ref.event_thrown().read(fd);
            EventId cfg_route = cfg_ref.event_route().read(fd);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_closed,
                    (i << 2) | EVENT_CLOSED), 0);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_thrown,
                    (i << 2) | EVENT_THROWN), 0);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_route,
                    (i << 2) | EVENT_ROUTE), 0);
        }
        priorityIndex_ = NO_PRIORITY;
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).description().write(fd, "");
            grp_ref.entry(i).group().write(fd, ROUTED_GROUP_NONE);
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

    // Implementations for the event handler functions.

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

        const unsigned index = registry_entry.user_arg >> 2;
        const unsigned type = registry_entry.user_arg & 3;

        // Update the desired state only; the staggered scheduler commits the
        // change(s) to the GPIO pins over time.
        switch (type)
        {
            case EVENT_CLOSED:
                desiredState_[index] = 0;
                break;
            case EVENT_THROWN:
                desiredState_[index] = 1;
                break;
            case EVENT_ROUTE:
                desiredState_[index] = 1;
                if (groups_[index] != ROUTED_GROUP_NONE)
                {
                    for (unsigned i = 0; i < size_; ++i)
                    {
                        if (i != index && groups_[i] == groups_[index])
                        {
                            desiredState_[i] = 0;
                        }
                    }
                }
                // The selected (THROWN) output is committed before its
                // siblings close.
                priorityIndex_ = index;
                break;
        }

        schedule_pending();
        done->notify();
    }

private:
    /// Sentinel meaning no output has commit priority.
    static constexpr unsigned NO_PRIORITY = UINT_MAX;

    /// Timer that drives the staggered application of pending output changes.
    /// Runs on the node's executor, i.e. the same thread as event handling, so
    /// no additional locking is needed.
    class StaggerTimer : public ::Timer
    {
    public:
        StaggerTimer(ConfiguredRoutedConsumer *parent, Node *node)
            : ::Timer(node->iface()->executor()->active_timers())
            , parent_(parent)
        {
        }

        long long timeout() OVERRIDE
        {
            return parent_->handle_timeout();
        }

    private:
        ConfiguredRoutedConsumer *parent_;
    };

    /// @return true if output @p i still needs to move to reach its desired
    /// state.
    bool is_pending(unsigned i) const
    {
        return (pins_[i]->is_set() ? 1 : 0) != desiredState_[i];
    }

    /// @return the index of the next output to commit (priority output first,
    /// then ascending index order), or NO_PRIORITY if nothing is pending.
    unsigned next_pending_index()
    {
        if (priorityIndex_ != NO_PRIORITY)
        {
            if (is_pending(priorityIndex_))
            {
                return priorityIndex_;
            }
            // The priority output is already in its desired state; drop it.
            priorityIndex_ = NO_PRIORITY;
        }
        for (unsigned i = 0; i < size_; ++i)
        {
            if (is_pending(i))
            {
                return i;
            }
        }
        return NO_PRIORITY;
    }

    /// Applies the desired state of a single output to its GPIO pin.
    void commit(unsigned i)
    {
        pins_[i]->write(desiredState_[i] ? Gpio::SET : Gpio::CLR);
        if (i == priorityIndex_)
        {
            priorityIndex_ = NO_PRIORITY;
        }
        lastCommitTimeNsec_ = OSTime::get_monotonic();
    }

    /// Ensures the stagger timer is running if there is pending work. Schedules
    /// the first commit no sooner than the stagger delay after the previous
    /// commit, so the global minimum spacing between movements is preserved.
    void schedule_pending()
    {
        if (timerRunning_)
        {
            // The running timer will pick up the newly desired changes on its
            // next tick.
            return;
        }
        if (next_pending_index() == NO_PRIORITY)
        {
            return;
        }
        timerRunning_ = true;
        // Fire immediately if enough time has already elapsed, otherwise wait
        // out the remainder of the stagger interval.
        timer_.start_absolute(lastCommitTimeNsec_ + delayNsec_);
    }

    /// Timer callback: commits the next pending output and re-arms if more
    /// remain. Runs on the node's executor.
    /// @return Timer::RESTART to keep going, Timer::NONE when idle.
    long long handle_timeout()
    {
        unsigned idx = next_pending_index();
        if (idx == NO_PRIORITY)
        {
            timerRunning_ = false;
            return Timer::NONE;
        }
        commit(idx);
        if (next_pending_index() != NO_PRIORITY)
        {
            // A positive return value reschedules the timer this many
            // nanoseconds from now, giving the next commit the full stagger
            // spacing (see Timer::run()).
            return delayNsec_;
        }
        timerRunning_ = false;
        return Timer::NONE;
    }

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry. The valid/invalid state is derived from whether the pin's current
    /// physical state matches the state implied by the event: CLOSED implies the
    /// pin is clear; THROWN and ROUTE imply the pin is set. Note that while a
    /// staggered movement is pending the physical state may briefly lag the
    /// desired state.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        const unsigned index = registry_entry.user_arg >> 2;
        const unsigned type = registry_entry.user_arg & 3;
        const bool pin_set = pins_[index]->is_set();
        // The event implies the pin should be set unless it is the CLOSED
        // event.
        const bool implies_set = (type != EVENT_CLOSED);
        if (pin_set != implies_set)
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
    uint8_t *groups_;         //< cached route group membership, one per output
    uint8_t *desiredState_;   //< desired state of each output (1 == THROWN)
    StaggerTimer timer_;      //< drives staggered application of changes
    ConfigReference offset_;  //< Offset in the configuration space for our
    // configs.
    ConfigReference delayOffset_;    //< Offset of the stagger delay config.
    long long delayNsec_{0};         //< stagger delay in nanoseconds
    long long lastCommitTimeNsec_{0}; //< monotonic time of the last commit
    unsigned priorityIndex_{NO_PRIORITY}; //< output to commit first, or none
    bool timerRunning_{false};       //< whether the stagger timer is armed
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDROUTEDCONSUMER_HXX_
