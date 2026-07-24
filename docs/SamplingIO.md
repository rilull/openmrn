# Sampling IO — Output pin with periodic pushbutton sampling

**Status:** Implemented, unit-tested on host, pending bench validation on STM32L431.
**Branch:** `claude/stm32-periodic-pin-sampling-ooJZu`
**Target hardware:** STM32L431 (STM32L4 family)

This document is the design reference for the "sampling IO" feature: a single MCU
pin that nominally drives an output (an LED) but periodically switches to an
input for a brief moment to sample a co-located pushbutton, then switches back
to output. It captures the design rationale, the hardware assumptions, the
OpenLCB/CDI surface, the code, and the open items so future changes have full
context.

---

## 1. Motivation and concept

We want a single GPIO pin to serve **both** an output and an input duty, so one
pin can drive an LED and also read a pushbutton. The pin is an output almost all
the time; on a fixed period it is briefly reconfigured as an input, the line is
sampled, and then it is returned to output. A very short button press may be
missed (acceptable); a normal human press is comfortably caught.

This mirrors the pattern OpenMRN already uses in the RailCom drivers
(`Stm32Railcom.hxx`), where a pin normally driving the track is briefly switched
to an input during the RailCom cutout to sample feedback. That was the proof the
approach is sound; this feature generalizes it into a reusable OpenLCB
producer/consumer.

---

## 2. Hardware support (STM32)

STM32 GPIOs have **no dedicated "sample mode" peripheral** — the pattern is
entirely software-managed. What makes it cheap:

- Pin mode is controlled by the **MODER** register (2 bits/pin). Switching
  output↔input is a single register write, a few CPU cycles.
- Reconfigured as input, the **IDR** immediately reflects the line state.
- Switched back to output, the **ODR retains its previous value**, so if the
  desired output level is written to the ODR *while still an input*, re-enabling
  the driver produces no glitch from the ODR side.

There is no timer/hardware sampler that does this automatically; the orchestration
comes from a software timer we run.

### 2.1 Reference bench circuit (the assumed wiring)

Per pin, active-low, common-anode LED plus pushbutton on one shared node:

```
            VCC                         VCC
             |                           |
            [1K]  (LED anode resistor)  [10K] (external pull-up)
             |                           |
           (LED)                         |
             |                           |
   MCU pin --[330R]---------+------------+---------(button)---- GND
             (series          shared node
              limiter)
```

- **LED:** driven by the MCU **sinking** current (common anode). LED **on** ⇒ the
  MCU pin drives **low**. Separate 1K resistor on the anode side.
- **Button:** connects the shared node to **GND** when pressed. Pressed ⇒ line
  **low**.
- **330R series resistor** on the MCU pin bounds the short-circuit current when
  the button is pressed while the pin is driving high — this is the key pin
  protection.
- **External 10K pull-up** holds the node high when the pin is high-impedance
  (during the input sampling window) and the button is not pressed.

Because both "LED on" and "button pressed" are **active-low** on this wiring, the
driver treats the pin uniformly as active-low (see `invert` below).

### 2.2 Known and accepted circuit interaction

When the LED is commanded **off** (pin driven/pulled high) and the user **presses
the button**, the button pulls the shared node — which is also the LED cathode —
toward GND. The LED cathode going low with the anode still pulled toward VCC lets
the LED **illuminate passively via the button**, independent of what the MCU
commands. This is a property of the passive network (LED + pull-up + button share
one node), not a software bug, and it happens regardless of push-pull vs
open-drain output.

**Decision:** This is **acceptable and desirable** — it is positive user
feedback that the press was registered. If it were ever unwanted, the fix is a
blocking diode between the button and the shared node, not a code change.

### 2.3 Push-pull vs open-drain

Push-pull is fine here: the 330R series resistor bounds contention current.
Open-drain would not change the passive LED-on-press behavior (that is downstream
of the drive stage either way), so the choice is preference/power, not safety.

---

## 3. Timing parameters

### 3.1 Sample period — 10 ms (compile-time constant, default)

Derived from physical constants, not per-installation tuning:

| Property | Value |
|---|---|
| Sample rate | 100 Hz |
| Output high-Z time per cycle | ~settle window only → **< 0.3% duty** |
| Min reliably detected press | ~20–30 ms (2–3 samples w/ debounce) |
| Min press that may be missed | < 20 ms (accepted) |
| Bounce handling | 2-sample debounce ⇒ 20 ms settle, outlasts typical mechanical bounce |

10 ms maps cleanly onto a FreeRTOS tick and a timer reload. The period is **not**
CDI-exposed — it is a hardware-derived constant; a misconfigured value (e.g.
500 ms) would make the input feel broken.

### 3.2 Settle delay — 20 µs (compile-time constant, default)

After switching to input, the line settles via an RC network dominated by the
**10K pull-up** against node capacitance:

| Node capacitance | 5τ settle |
|---|---|
| On-board only (~30 pF) | ~1.5 µs |
| A few feet of panel wire (~200 pF) | ~10 µs |
| Longer run (~500 pF) | ~25 µs |

Worst case is LED-on (node at 0 V) → switch to input, button **not** pressed:
the node must charge up through the 10K. Assumed resistors: **330R** series,
**1K** LED anode. **20 µs** comfortably covers on-board plus several feet of
cable while being ~0.2% of the 10 ms period. Verify on the bench with a scope
and tighten/loosen if wiring differs.

The realistic total output-disruption window (input switch + settle + read +
switch back) is ~**20–30 µs**, not the ~1–5 µs one might guess from HAL-call
overhead alone — negligible at 10 ms.

### 3.3 Debounce — CDI-exposed, default 2

Number of consecutive equal samples required before a state change is accepted.
Unit is one sample period (10 ms). Default 2 ⇒ 20 ms. Exposed so users can raise
it in noisy environments; uses `QuiesceDebouncer` from `utils/Debouncer.hxx`.

---

## 4. OpenLCB / CDI surface

### 4.1 Events per pin (3)

Each pin is simultaneously a **consumer** (LED) and a **producer** (button):

- `event_on` (consumed) — turns the LED **on**
- `event_off` (consumed) — turns the LED **off**
- `event_pressed` (produced) — emitted on each debounced **press**

A pushbutton is momentary, so there is **one** produced event (press only); an
"input off" event is not useful for a pushbutton. The user *may* assign the same
event ID to, say, `event_pressed` and `event_on` if they want a button to toggle
its own LED, but that is their choice, not wired in.

The producer is identified on the bus as **PRODUCER_IDENTIFIED_UNKNOWN**
(momentary). The consumer identifies **VALID/INVALID** reflecting current LED
state.

### 4.2 CDI widgets available (context)

CDI has no bespoke "sampling pin" widget. All configuration reduces to leaf
types: `<int>` (+`<map>` → dropdown, or min/max → spinner), `<eventid>` (event
picker), `<string>` (text). Groups/`RepeatedGroup` add sections. CDI cannot
conditionally show/hide fields based on another field's value.

### 4.3 CDI group

```cpp
CDI_GROUP(SamplingPinConfig);
CDI_GROUP_ENTRY(description,   StringConfigEntry<20>, Name("Description"), ...);
CDI_GROUP_ENTRY(event_on,      EventConfigEntry,      Name("Output On"), ...);
CDI_GROUP_ENTRY(event_off,     EventConfigEntry,      Name("Output Off"), ...);
CDI_GROUP_ENTRY(event_pressed, EventConfigEntry,      Name("Button Pressed"), ...);
CDI_GROUP_ENTRY(debounce,      Uint8ConfigEntry,      Name("Debounce parameter"),
    Default(2), Min(1), Max(255), Description(...));
CDI_GROUP_END();
```

Entry size = 20 + 8 + 8 + 8 + 1 = **45 bytes**. Use as
`RepeatedGroup<SamplingPinConfig, N>` for a bank of pins.

---

## 5. Software architecture

Modeled closely on `openlcb/MultiConfiguredPC.hxx` (the multi-pin
producer/consumer), extended for continuous input/output switching.

### 5.1 Two-tier design

1. **Sampler tier** — a `StateFlowBase` on a **dedicated executor** (like
   `SpiIOShiftRegister` in the nucleo_io app), self-rescheduling every 10 ms via
   `StateFlowTimer`/`sleep_and_call`. Each cycle:
   - switch **all** pins to input,
   - one `usleep(settle)` for the whole bank,
   - for each pin: read → restore output level (write ODR) → switch back to
     output → update debouncer → latch any press edge.

   Kept on a dedicated executor so its brief blocking settle delay cannot stall
   the main loop, and main-loop traffic cannot jitter the sampling.

2. **Publisher tier** — the main executor's `RefreshLoop` (`poll_33hz`) drains
   latched press edges onto the bus asynchronously, reusing the proven
   `MultiConfiguredPC` producer flow. Bus writes never happen on the sampler
   thread.

All pin I/O is confined to the sampler thread. `handle_event_report` (LED
on/off) only records the desired state; the pin is (re)driven on the next sample
cycle, so the LED updates within ≤ one period.

### 5.2 GPIO abstraction boundary (the key decision)

The generic `GpioWrapper::set_direction()` deliberately **HASSERTs** against
direction changes, and the plain `Gpio::set_direction()` would not preserve the
pull configuration. The pull/OD-preserving switching lives only in the **static**
`GpioHwPin::set_input()/set_output()` (`freertos_drivers/st/Stm32Gpio.hxx`).

So we introduced **`GpioSamplingWrapper<PIN>`** (derives from `GpioWrapper<PIN>`,
overrides only `set_direction()` to call `PIN::set_output()/set_input()`). The
driver consumes plain `const Gpio*`, staying fully hardware-agnostic; **all
target-specific detail (port, pull, F3-vs-L4 HAL) lives in the board's
`GPIO_XPIN(..., GpioHwPin, ...)` declaration.** This is what makes the class
reusable across MCUs.

### 5.3 Polarity (`invert`)

A single `invert` constructor flag (default **true** = active-low) captures the
common-anode/sink + button-to-ground wiring. It centralizes both senses:
- output: `output_level(active) = invert ? !active : active`
- input: `is_pressed(level) = invert ? (level==LOW) : (level==HIGH)`

Kept as a per-instance flag (not pushed to a per-pin `InvertedGpio`) so one
driver can serve a bank of identically-wired pins without per-pin wrapper
gymnastics. If mixed polarity per pin is ever needed, wrap individual pins in
`InvertedGpio` and set `invert=false`.

### 5.4 Per-pin state and concurrency

Per-pin state lives in a single owned allocation, `std::unique_ptr<PerPin[]>`:

```cpp
struct PerPin {
    EventId producedEvent {0};        // press event; 0 if unconfigured
    QuiesceDebouncer debouncer {2};   // button debouncer
    bool ledActive {false};           // desired output state
    bool pendingPress {false};        // latched press edge awaiting publish
};
```

Shared between the sampler thread and the main thread. Guarded by the class's
`Atomic` base with **tiny** critical sections that never span the blocking
settle delay: the sampler snapshots `ledActive` under lock, does pin I/O + settle
**without** the lock, then updates the debouncer + `pendingPress` under lock. The
publisher reads/clears `pendingPress` under lock; `handle_event_report` writes
`ledActive` under lock.

`user_arg` for event registrations is packed as `pin*4 + slot`, where slot ∈
{`SLOT_OFF`=0, `SLOT_ON`=1, `SLOT_PRESSED`=2}. Decoded via `>>2` / `&3`.

### 5.5 Sampler shutdown

`Sampler::shutdown()` is a synchronous stop, safe from any thread: it sets a flag
and hops onto the sampler's own executor (via `CallbackExecutable`) to call
`timer_.ensure_triggered()` (the `Timer` contract requires triggering on its own
executor), waking a sleeping sampler which then terminates and notifies a
`SyncNotifiable` the caller blocks on. This is the `RefreshLoop::stop()` idiom
(`set_terminated()` + `ensure_triggered()`) extended to block until the flow is
actually done, so the object can be destroyed safely. Objects are expected to
have static lifetime in normal use.

---

## 6. Files

| File | Purpose |
|---|---|
| `src/openlcb/ConfiguredSamplingIO.hxx` | The driver: CDI group + `ConfiguredSamplingIO` producer/consumer + nested `Sampler` state flow. |
| `src/freertos_drivers/common/GpioSamplingWrapper.hxx` | `Gpio` adapter whose `set_direction()` performs the pull-preserving input/output switch. |
| `src/openlcb/ConfiguredSamplingIO.cxxtest` | Host unit test (8 cases) using a fake sampling-capable `Gpio`. |

Reference/prior-art files: `src/openlcb/MultiConfiguredPC.hxx`,
`src/freertos_drivers/st/Stm32Railcom.hxx`,
`src/freertos_drivers/st/Stm32Gpio.hxx` (GpioHwPin `set_input`/`set_output`),
`applications/nucleo_io/.../main.cxx` (`SpiIOShiftRegister` timer pattern),
`src/utils/Debouncer.hxx` (`QuiesceDebouncer`), `src/openlcb/RefreshLoop.hxx`.

---

## 7. Usage sketch (board integration)

```cpp
// Board hardware declaration — hardware specifics live here.
GPIO_XPIN(IO1, GpioHwPin, A, 5, Output(), PullNone());
GPIO_XPIN(IO2, GpioHwPin, A, 6, Output(), PullNone());
// ... external 10K pull-ups on the board; internal pull = none.

// A dedicated executor for the sampler tier (isolated from the main loop).
Executor<1> io_executor("io_thread", 0, 1300);
Service io_service(&io_executor);

// Pin array via the sampling wrapper (runtime Gpio* interface).
const Gpio *const kSamplingPins[] = {
    GpioSamplingWrapper<IO1_Pin>::instance(),
    GpioSamplingWrapper<IO2_Pin>::instance(),
};

// The producer/consumer, bound to the CDI repeated group.
openlcb::ConfiguredSamplingIO sampling_io(
    stack.node(), &io_service, kSamplingPins, ARRAYSIZE(kSamplingPins),
    cfg.seg().sampling_io());   // RepeatedGroup<SamplingPinConfig, N>

// Publisher tier: add to the RefreshLoop like any other Polling member.
openlcb::RefreshLoop loop(stack.node(), {sampling_io.polling()});
```

Defaults: `invert=true`, `sample_period_msec=10`, `settle_usec=20`. Override in
the constructor to correct for different hardware.

---

## 8. Testing

### 8.1 Host unit tests (passing)

`ConfiguredSamplingIO.cxxtest` (8 cases), built under `targets/test`:

- `CreateDestroy` — construction + clean synchronous shutdown.
- `LedStartsOff` — default output is off.
- `ConsumerTurnsLedOnAndOff` — `event_on`/`event_off` drive the LED (visible
  after a sample cycle).
- `ConsumerPinsAreIndependent` — pins don't cross-talk.
- `ButtonPressProducesEvent` — a held press past debounce emits `event_pressed`.
- `ButtonReleaseProducesNoEvent` — release emits nothing (momentary).
- `DebounceRequiresStableSamples` — a bounce shorter than the debounce count is
  rejected; a stable hold is accepted.
- `PressDoesNotDisturbLed` — after sampling, the LED output level and output
  direction are restored.

The test uses a `FakeSamplingGpio` (supports `set_direction`, lets the test drive
the "button" line independently of the written output level) and drives sampling
manually via a `TEST_sample_once()` hook while the background sampler sleeps on a
long period — deterministic, no real-time waits.

Build/run (host):
```
cd targets/test && make openlcb/ConfiguredSamplingIO.test
./openlcb/ConfiguredSamplingIO.test
```
(Host test build requires googletest and `libavahi-client-dev`.)

### 8.2 Bench validation — TODO (STM32L431)

- [ ] Scope the shared node: confirm it settles within the 20 µs window for the
      actual wiring/cable length; adjust `settle_usec` if needed.
- [ ] Confirm the LED output glitch during sampling is imperceptible on the real
      load.
- [ ] Confirm presses of the accepted minimum duration are reliably detected,
      and debounce default (2) is adequate for the switch used.
- [ ] Confirm the accepted "LED lights while button pressed & LED off" feedback
      behaves as expected.
- [ ] Verify pull configuration: internal pull = none (external 10K present);
      ensure no internal pull-down fights the external pull-up.

---

## 9. Design decisions log (for future changes)

| # | Decision | Rationale |
|---|---|---|
| 1 | One pin = combined producer + consumer | LED (consumer) + button (producer) on the same pin. |
| 2 | 3 events/pin (on, off, pressed) | Momentary button needs only a press event. |
| 3 | Sample period 10 ms, compile-time | Human-factors/bounce derived; not user-tunable. |
| 4 | Settle 20 µs, compile-time | RC settle for 10K + realistic cable; verify on bench. |
| 5 | Debounce CDI-exposed, default 2 | Per-environment noise tuning is legitimate. |
| 6 | Runtime `Gpio*` + `GpioSamplingWrapper` | Keeps driver hardware-agnostic; HW specifics in `GPIO_XPIN`. |
| 7 | Two-tier: dedicated sampler executor + RefreshLoop publish | Isolates blocking settle from the stack; bus writes stay async on main. |
| 8 | Dedicated executor + inline `usleep` settle | Follows `SpiIOShiftRegister`; simplest for 20 µs. |
| 9 | `invert` per-instance flag, default true | Serves a bank of identically active-low pins; centralizes polarity. |
| 10 | LED off by default | Safe startup state for common-anode wiring. |
| 11 | Batch settle: all pins→input, one usleep | Settle is a line property, not per-pin; avoids N serial delays. |
| 12 | Single `PerPin` allocation | One heap block vs four; better RAM/locality on the L431. |

### Deliberately deferred / not done

- **`invert` → per-pin `InvertedGpio`:** kept the per-instance flag; revisit only
  if mixed-polarity pins on one driver instance are required.
- **Immediate LED drive on `event_report`:** intentionally deferred to the next
  sample cycle to keep all pin I/O on the sampler thread (≤ 10 ms latency,
  imperceptible for an LED).

---

## 10. Open questions to revisit after bench testing

- Does 20 µs settle hold for the real cable runs, or should it be raised (and if
  so, is it worth making it a constructor parameter per install)?
- Is debounce default 2 right for the chosen switch, or should the default move?
- Any measurable LED flicker at 10 ms for particular LED types/loads that would
  argue for a shorter settle or period?
- Timer budget on the L431 if this coexists with other periodic features — is the
  dedicated executor's stack size (sketch uses ~1300) adequate?
