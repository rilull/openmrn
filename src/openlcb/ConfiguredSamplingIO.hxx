/** \copyright
 * Copyright (c) 2026, Balazs Racz
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
 * \file ConfiguredSamplingIO.hxx
 *
 * Producer-Consumer class that drives a set of GPIO pins which each behave as
 * an output (e.g. an LED) most of the time, but are periodically and briefly
 * switched to an input to sample a co-located pushbutton, then switched back
 * to output. This lets a single MCU pin serve both an output and an input
 * function.
 *
 * Each pin exposes three events: two consumed events that turn the output on
 * and off, and one produced event that is emitted on each debounced button
 * press.
 *
 * The design is two-tier:
 *  - A @ref Sampler state flow runs on a dedicated executor at a fixed period
 *    (default 10 ms). Each cycle it switches every pin to input, waits a short
 *    settle time, reads the pin, restores the output level and switches back to
 *    output, then feeds a per-pin debouncer and latches any press edge. This
 *    tight loop is deliberately isolated from the main executor so its timing
 *    is not perturbed by, and its brief blocking settle delay does not perturb,
 *    the rest of the stack.
 *  - The main executor's @ref RefreshLoop drains the latched press edges onto
 *    the bus asynchronously, reusing the proven producer flow.
 *
 * @author Balazs Racz
 * @date 24 Jul 2026
 */

#ifndef _OPENLCB_CONFIGUREDSAMPLINGIO_HXX_
#define _OPENLCB_CONFIGUREDSAMPLINGIO_HXX_

#include <memory>

#include "executor/Executable.hxx"
#include "executor/Notifiable.hxx"
#include "executor/StateFlow.hxx"
#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/RefreshLoop.hxx"
#include "os/os.h"
#include "utils/Atomic.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "utils/Debouncer.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

/// CDI configuration for a single sampling IO line (one pin that is both an
/// output and a sampled pushbutton input).
CDI_GROUP(SamplingPinConfig);
/// Allows the user to assign a name for this line.
CDI_GROUP_ENTRY(description, StringConfigEntry<20>, //
    Name("Description"), Description("User name of this line."));
/// Event that turns the output (LED) on.
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Output On"),
    Description("Receiving this event ID will turn the output on."));
/// Event that turns the output (LED) off.
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Output Off"),
    Description("Receiving this event ID will turn the output off."));
/// Event produced when the button is pressed.
CDI_GROUP_ENTRY(event_pressed, EventConfigEntry, //
    Name("Button Pressed"),
    Description("This event ID is produced when the button is pressed. A "
                "momentary pushbutton has no separate release event."));
/// Configures the debounce parameter.
CDI_GROUP_ENTRY(debounce, Uint8ConfigEntry, Name("Debounce parameter"),
    Default(2),
    Description("Number of consecutive samples for which the button must read "
                "the same value before the change is accepted. Each sample is "
                "one sampling period (10 msec by default) apart. A value of 2 "
                "works well for a typical pushbutton; increase it in noisy "
                "environments for a more stable but slower response."),
    Min(1), Max(255));
CDI_GROUP_END();

/// Producer-Consumer event handler that operates several GPIO pins in the
/// "output with periodic input sampling" mode. See the file comment for a
/// description of the two-tier design.
///
/// The pins are supplied as @ref Gpio* instances whose set_direction() is able
/// to switch the pin between output and input at runtime (see @ref
/// GpioSamplingWrapper). All hardware-specific detail lives behind that
/// interface, so this class is target independent.
class ConfiguredSamplingIO : public ConfigUpdateListener,
                             private SimpleEventHandler,
                             private Polling,
                             private Notifiable,
                             private Atomic
{
public:
    typedef SamplingPinConfig config_entry_type;
    typedef QuiesceDebouncer debouncer_type;

    /// Constructor.
    ///
    /// @param node is the OpenLCB node object from the stack.
    /// @param sampler_service is a Service running on an executor that is NOT
    /// the main executor. The periodic sampler state flow runs here; its brief
    /// per-cycle blocking settle delay must not be allowed to stall the main
    /// executor.
    /// @param pins is the list of pins, as Gpio* instances whose
    /// set_direction() dynamically reconfigures input/output (e.g. from @ref
    /// GpioSamplingWrapper). Can be constant from FLASH space.
    /// @param size is the length of the list of pins array; must equal N.
    /// @param config is the repeated group object from the configuration space
    /// that represents the events for each pin.
    /// @param invert if true (the default), the hardware is active-low: the
    /// output is asserted by driving the pin low (e.g. an LED whose cathode is
    /// on this pin, sinking current), and a pressed button reads as a low pin
    /// level. If false, the hardware is active-high.
    /// @param sample_period_msec is the interval between sampling cycles.
    /// @param settle_usec is the time to wait after switching a pin to input
    /// before reading it, to allow the (RC-limited) line to settle.
    template <unsigned N>
    __attribute__((noinline)) ConfiguredSamplingIO(Node *node,
        Service *sampler_service, const Gpio *const *pins, unsigned size,
        const RepeatedGroup<config_entry_type, N> &config, bool invert = true,
        unsigned sample_period_msec = 10, unsigned settle_usec = 20)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , offset_(config)
        , invert_(invert)
        , settleUsec_(settle_usec)
        , sampler_(this, sampler_service, sample_period_msec)
    {
        // Mismatched sizing of the GPIO array from the configuration array.
        HASSERT(size == N);
        ConfigUpdateService::instance()->register_update_listener(this);
        // Single allocation for all per-pin state (see @ref PerPin). Each
        // element is default-constructed with its in-class initializers.
        state_.reset(new PerPin[size_]);
        for (unsigned i = 0; i < size_; ++i)
        {
            // Drives the output to the safe (off) state to start with.
            write_output(i);
        }
        // Starts the periodic sampler only after all per-pin state is
        // constructed above, so the sampler thread never observes it
        // half-initialized.
        sampler_.start();
    }

    ~ConfiguredSamplingIO()
    {
        sampler_.shutdown();
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
    }

    /// @return the instance to give to the RefreshLoop object.
    Polling *polling()
    {
        return this;
    }

    /// Call from the refresh loop. Publishes any pending debounced button
    /// presses to the bus.
    void poll_33hz(WriteHelper *helper, Notifiable *done) override
    {
        nextPinToPoll_ = 0;
        pollingHelper_ = helper;
        pollingDone_ = done;
        this->notify();
    }

    /// Asynchronous callback when the previous polling message has left via the
    /// bus. Used as a poor man's iterative state machine to walk the pins.
    void notify() override
    {
        for (; nextPinToPoll_ < size_; ++nextPinToPoll_)
        {
            auto i = nextPinToPoll_;
            bool pressed = false;
            {
                AtomicHolder h(this);
                if (state_[i].pendingPress)
                {
                    state_[i].pendingPress = false;
                    pressed = true;
                }
            }
            if (pressed && state_[i].producedEvent)
            {
                ++nextPinToPoll_; // avoid infinite loop.
                pollingHelper_->WriteAsync(node_, Defs::MTI_EVENT_REPORT,
                    WriteHelper::global(),
                    eventid_to_buffer(state_[i].producedEvent), this);
                return;
            }
        }
        pollingDone_->notify();
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            // There is no way to figure out what the previously registered
            // eventid values were for the individual pins. Therefore we always
            // unregister everything and register them anew. This also causes us
            // to identify all, which is fine as apply_configuration comes from
            // a user action.
            do_unregister();
        }
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event_on = cfg_ref.event_on().read(fd);
            EventId cfg_event_off = cfg_ref.event_off().read(fd);
            EventId cfg_event_pressed = cfg_ref.event_pressed().read(fd);
            uint8_t param = cfg_ref.debounce().read(fd);
            // Consumer registrations for the output (LED).
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_off, i * 4 + SLOT_OFF), 0);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event_on, i * 4 + SLOT_ON), 0);
            // Producer registration for the button.
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(
                    this, cfg_event_pressed, i * 4 + SLOT_PRESSED),
                0);
            {
                AtomicHolder h(this);
                state_[i].producedEvent = cfg_event_pressed;
                state_[i].debouncer.reset_options(param ? param : 1);
                // The button is not pressed at rest; initialize accordingly.
                state_[i].debouncer.initialize(false);
                state_[i].pendingPress = false;
            }
        }
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).description().write(fd, "");
            CDI_FACTORY_RESET(grp_ref.entry(i).debounce);
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

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        if (slot_of(registry_entry) == SLOT_PRESSED)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
        else
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        if (slot_of(registry_entry) != SLOT_PRESSED)
        {
            SendConsumerIdentified(registry_entry, event, done);
        }
    }

    void handle_identify_producer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        if (slot_of(registry_entry) == SLOT_PRESSED)
        {
            SendProducerIdentified(registry_entry, event, done);
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->event != registry_entry.event)
        {
            return;
        }
        unsigned slot = slot_of(registry_entry);
        if (slot == SLOT_PRESSED)
        {
            return; // Producer-only event; nothing to consume.
        }
        unsigned pin = pin_of(registry_entry);
        // Only records the desired state; the pin's output is (re-)driven by
        // the sampler on its next cycle, so the LED updates within one sampling
        // period. All pin I/O is kept on the sampler thread on purpose.
        AtomicHolder h(this);
        state_[pin].ledActive = (slot == SLOT_ON);
    }

    /// Test-only hook: synchronously runs a single sampling cycle on the
    /// caller's thread. Intended for unit tests where the periodic sampler is
    /// configured with a long period and sampling is driven manually.
    void TEST_sample_once()
    {
        sample_all();
    }

private:
    /// All mutable state for one pin, in a single allocation (see @ref state_).
    struct PerPin
    {
        /// Event produced on button press. 0 when unconfigured.
        EventId producedEvent {0};
        /// Debouncer for the sampled button input. The initial count is a
        /// placeholder; apply_configuration() sets the configured value.
        debouncer_type debouncer {2};
        /// Desired logical output (LED) state.
        bool ledActive {false};
        /// Latched debounced press edge awaiting publication.
        bool pendingPress {false};
    };

    /// user_arg slot values, packed as pin * 4 + slot.
    enum Slot
    {
        SLOT_OFF = 0,     ///< consumer: turn output off
        SLOT_ON = 1,      ///< consumer: turn output on
        SLOT_PRESSED = 2, ///< producer: button pressed
    };

    /// @return the pin index encoded in a registry entry's user_arg.
    static unsigned pin_of(const EventRegistryEntry &e)
    {
        return e.user_arg >> 2;
    }

    /// @return the slot (see @ref Slot) encoded in a registry entry's user_arg.
    static unsigned slot_of(const EventRegistryEntry &e)
    {
        return e.user_arg & 3;
    }

    /// @return the current logical output (LED) state for a pin.
    bool get_led(unsigned pin)
    {
        AtomicHolder h(this);
        return state_[pin].ledActive;
    }

    /// Translates a logical output-active state to the electrical pin level,
    /// honoring the active-low/active-high polarity. This is the single place
    /// the output polarity rule lives.
    /// @param active logical "output on" state.
    /// @return the level to write to the pin.
    bool output_level(bool active) const
    {
        // active-low (invert_): asserted output => drive pin low.
        return invert_ ? !active : active;
    }

    /// Translates a raw sampled pin level to a logical "pressed" state.
    /// @param level the value read from the pin.
    /// @return true if the button is pressed.
    bool is_pressed(Gpio::Value level) const
    {
        bool high = (level == Gpio::VHIGH);
        // active-low: a pressed button pulls the line low.
        return invert_ ? !high : high;
    }

    /// Drives the given pin's output to its current logical state.
    /// @param pin index of the pin to drive.
    void write_output(unsigned pin)
    {
        pins_[pin]->write(output_level(get_led(pin)));
    }

    /// The periodic sampler. Runs on the dedicated executor and, every period,
    /// briefly switches each pin to an input to read the button, then restores
    /// output. Updates the debouncers and latches press edges for the main
    /// executor to publish.
    class Sampler : public StateFlowBase
    {
    public:
        /// Constructor.
        /// @param parent owning ConfiguredSamplingIO.
        /// @param service dedicated (non-main) executor's service.
        /// @param period_msec sampling interval in milliseconds.
        Sampler(ConfiguredSamplingIO *parent, Service *service,
            unsigned period_msec)
            : StateFlowBase(service)
            , parent_(parent)
            , periodMsec_(period_msec)
        {
        }

        /// Starts the periodic sampling loop.
        void start()
        {
            start_flow(STATE(wait_period));
        }

        /// Synchronously stops the sampling loop. Safe to call from any thread;
        /// blocks until the flow has terminated and its timer is no longer
        /// active, so that the object can be safely destroyed afterwards. This
        /// is the same set_terminated()+ensure_triggered() stop idiom as @ref
        /// RefreshLoop::stop(), extended to block the caller until the flow is
        /// actually done rather than leaving that wait to the caller.
        void shutdown()
        {
            SyncNotifiable sn;
            stopNotify_ = &sn;
            shutdown_ = true;
            // The timer may only be triggered from its own executor, so hop
            // there to wake a sleeping sampler. The flow then observes
            // shutdown_ and terminates.
            service()->executor()->add(new CallbackExecutable(
                [this]() { timer_.ensure_triggered(); }));
            sn.wait_for_notification();
        }

    private:
        Action wait_period()
        {
            if (shutdown_)
            {
                return terminate();
            }
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(periodMsec_), STATE(do_sample));
        }

        Action do_sample()
        {
            if (shutdown_)
            {
                return terminate();
            }
            parent_->sample_all();
            return call_immediately(STATE(wait_period));
        }

        /// Terminates the flow and notifies any waiter. After set_terminated()
        /// returns wait(), the executor will not touch this object's members
        /// again (same guarantee delete_this() relies on), so notifying here is
        /// safe against a concurrent destructor.
        Action terminate()
        {
            Action a = set_terminated();
            if (stopNotify_)
            {
                Notifiable *n = stopNotify_;
                stopNotify_ = nullptr;
                n->notify();
            }
            return a;
        }

        /// Timer helper for the periodic wakeup.
        StateFlowTimer timer_ {this};
        /// Owning object.
        ConfiguredSamplingIO *parent_;
        /// Sampling interval in milliseconds.
        unsigned periodMsec_;
        /// Set to request the loop to stop.
        bool shutdown_ {false};
        /// Notified once the flow has terminated. Owned by the caller of
        /// shutdown().
        Notifiable *stopNotify_ {nullptr};
    };

    /// Performs one sampling cycle across all pins. Runs on the dedicated
    /// sampler executor. The atomic critical sections are kept tiny and never
    /// span the blocking settle delay.
    void sample_all()
    {
        // Switch every pin to input first, then pay the RC settle delay once
        // for the whole bank (the settle time is a property of the line, not of
        // pin count). All pins are briefly high-impedance together.
        for (unsigned i = 0; i < size_; ++i)
        {
            pins_[i]->set_direction(Gpio::Direction::DINPUT);
        }
        usleep(settleUsec_);
        // Read each pin, restore its output, switch it back, and debounce.
        for (unsigned i = 0; i < size_; ++i)
        {
            const Gpio *pin = pins_[i];
            bool pressed = is_pressed(pin->read());
            // Snapshot the desired output state under the lock, then restore the
            // output level (writing the ODR while still an input avoids a
            // glitch) and re-enable the output driver.
            bool active;
            {
                AtomicHolder h(this);
                active = state_[i].ledActive;
            }
            pin->write(output_level(active));
            pin->set_direction(Gpio::Direction::DOUTPUT);
            AtomicHolder h(this);
            if (state_[i].debouncer.update_state(pressed) &&
                state_[i].debouncer.current_state())
            {
                // Rising edge of the debounced press; latch it for publishing.
                state_[i].pendingPress = true;
            }
        }
    }

    /// Removes registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Sends out a ConsumerIdentified message for the given registration entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        bool led = get_led(pin_of(registry_entry));
        // event_on is valid when the output is on; event_off when it is off.
        bool valid = (slot_of(registry_entry) == SLOT_ON) ? led : !led;
        Defs::MTI mti = valid ? Defs::MTI_CONSUMER_IDENTIFIED_VALID
                              : Defs::MTI_CONSUMER_IDENTIFIED_INVALID;
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    /// Sends out a ProducerIdentified message for the given registration entry.
    /// The button is a momentary input, so its state is reported as unknown.
    void SendProducerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        event->event_write_helper<4>()->WriteAsync(node_,
            Defs::MTI_PRODUCER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
            eventid_to_buffer(registry_entry.event), done->new_child());
    }

    // Variables used for asynchronous state during the polling loop.
    /// Which pin to next check when polling.
    unsigned nextPinToPoll_;
    /// Write helper to use for producing messages during the polling loop.
    WriteHelper *pollingHelper_;
    /// Notifiable to call when the polling loop is done.
    Notifiable *pollingDone_;

    /// Virtual node to export the consumer / producer on.
    Node *node_;
    /// Array of all GPIO pins to use. Externally owned.
    const Gpio *const *pins_;
    /// Number of GPIO pins to export.
    size_t size_;
    /// Offset in the configuration space for our configs.
    ConfigReference offset_;
    /// True if the hardware is active-low (output asserted by driving low, and
    /// a pressed button reads low).
    bool invert_;
    /// Settle time in microseconds after switching to input before reading.
    unsigned settleUsec_;
    /// Per-pin state, one owned allocation of size_ elements. The mutable
    /// fields are guarded by *this Atomic.
    std::unique_ptr<PerPin[]> state_;
    /// The periodic sampler. Declared last so it is constructed after the
    /// state it references.
    Sampler sampler_;
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDSAMPLINGIO_HXX_
