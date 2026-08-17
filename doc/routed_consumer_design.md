# ConfiguredRoutedConsumer — Design History and Guide

## Purpose

`ConfiguredRoutedConsumer` is a CDI-configured OpenLCB consumer that drives a
group of GPIO outputs used to control turnout (switch machine) motors such as a
Tortoise or an MP10 in 2-wire mode. It provides two layers of control:

1. **Granular per-turnout control** — each output has its own CLOSED and THROWN
   events, so any single turnout can be moved independently.
2. **Route (yard-ladder) control** — each output additionally has a *route*
   event. Receiving an output's route event moves that turnout to THROWN and
   moves every other turnout in the same *route group* to CLOSED. This lets a
   single button select a track and line the entire path automatically.

## Motivating use case

A node drives 16 turnouts. On a yard ladder, the operator wants to push one
button to select a destination track: the selected turnout throws and all the
other turnouts feeding that ladder line to closed, so the path is set with a
single Event ID. At the same time, individual closed/thrown control of each
turnout must remain available.

Grouping must be flexible: one group of 16, two groups of 8, four groups of 4,
or arbitrary mixes. Grouping is decoupled from physical output order so any
output can belong to any group.

## Hardware / output model

- Each turnout is driven by a **single GPIO pin** connected to a **TC4428**
  driver chip. The TC4428 provides one standard and one inverted output, giving
  the differential drive a stall motor (Tortoise) needs from a single GPIO.
- MP10 machines are used in **2-wire mode** so they behave like a Tortoise
  (no constant-current requirement).
- Outputs are **constant-on** (sustained level). No pulsed/momentary output
  support is needed at this time.
- **Output polarity is fixed** by convention (see below). No per-output invert
  option — polarity is handled in wiring to keep the configuration simple.

### Polarity convention

- `GPIO set()`  → turnout **THROWN**
- `GPIO clr()`  → turnout **CLOSED**

## Configuration (CDI)

The consumer is configured through a `RepeatedGroup<RoutedConsumerConfig, N>`,
consistent with `MultiConfiguredConsumer` and `ConfiguredExclusiveConsumer`.

Per-output config group (`RoutedConsumerConfig`):

| Entry           | Type                         | Meaning                                                        |
|-----------------|------------------------------|----------------------------------------------------------------|
| `description`   | `StringConfigEntry`          | User name of this turnout.                                      |
| `group`         | dropdown (`MapConfigEntry`)  | Route group membership. `None` plus `Group 1 … Group 16`.       |
| `event_closed`  | `EventConfigEntry`           | Receiving this event moves this turnout to CLOSED.             |
| `event_thrown`  | `EventConfigEntry`           | Receiving this event moves this turnout to THROWN.            |
| `event_route`   | `EventConfigEntry`           | Moves this turnout THROWN and all group siblings to CLOSED.   |

### Group dropdown

Rendered as a dropdown with an explicit **`None`** choice plus `Group 1 …
Group 16`. `None` means the output does not participate in route logic — its
route event simply throws itself and closes no siblings. Outputs sharing the
same group value are siblings for routing purposes.

Group IDs are read once during `apply_configuration` and cached in a small
`uint8_t groups_[N]` array so `handle_event_report` never reads the config fd
on the hot path.

## Event semantics

For output `i`:

- `event_closed` → `pins_[i]->clr()`
- `event_thrown` → `pins_[i]->set()`
- `event_route`  → `pins_[i]->set()`, and for every `j` where
  `groups_[j] == groups_[i]` (and the group is not `None`), `pins_[j]->clr()`.

There is a single route event per output; it always throws self and closes
siblings. There is no separate "route to closed" event.

## Route tables (cascade ladders)

The group model above fits a yard ladder where every turnout hangs off a single
common track (each track has exactly one turnout on its path). The **most common**
real ladder is a *cascade*: reaching a track requires an arbitrary pattern of
turnout positions, and turnouts off the chosen path are **don't-care**. The
group model cannot express don't-care, so a separate **route table** is provided
alongside it (both coexist; individual CLOSED/THROWN events remain too).

A route is a trigger event plus a **sparse list of `(turnout, state)` actions**.
Firing the event moves each listed turnout to its state and leaves every other
turnout unchanged — omission *is* the don't-care. This expresses cascade ladders,
crossovers, and any arbitrary route.

### CDI shape

Modeled on the nested repeated-group pattern in
`applications/dmx_controller/DmxSceneConfig.hxx` (whose "channel = 0 → change
nothing" is the same sparse/omit idea):

- `RouteActionConfig` = `turnout` (`Uint8`, dropdown `None` + `Turnout 1..16`) +
  `state` (`Uint8`, dropdown `Closed`/`Thrown`). A slot with turnout `None` is
  ignored.
- `RouteConfig` = `description` (`StringConfigEntry<16>`) + `event`
  (`EventConfigEntry`) + `actions`
  (`RepeatedGroup<RouteActionConfig, MAX_ROUTE_ACTIONS>`).
- `MAX_ROUTE_ACTIONS` (default 8) bounds how many turnouts one route may set;
  raise it for deep cascades. Every route reserves this many slots, so it also
  drives config size.

### Semantics and reuse

- `user_arg` reuses the spare low-2-bit `type` value **3** (`EVENT_ROUTE_TABLE`);
  the upper bits hold the route index. Output events keep types 0/1/2.
- Route actions are cached in RAM (`routeTurnout_[]`, `routeState_[]`) at
  `apply_configuration`, bounds-checked against the pin count, so
  `handle_event_report` never reads the config fd.
- `apply_route()` writes the actions into `desiredState_[]` and the **existing
  staggering engine** commits them in ascending output-index order (no priority
  output for table routes) — current limiting applies unchanged.
- Identify reports a route **VALID** iff every turnout it lists is already at its
  target state, else INVALID.

## Staggered movement (current limiting)

Switch machine motors draw current only while traveling (an MP10 pulls ~100 mA
for roughly 3 s per move, then nothing). A route event can change many outputs
at once, and closing a sibling makes *its* motor travel too, so an unthrottled
route would start many motors simultaneously and spike the supply current.

To bound peak current, output changes are **staggered**: they are committed one
at a time with a configurable minimum spacing between successive movements. This
is a **global throttle** — it applies to individual CLOSED/THROWN events and
route events alike, so no burst of events can exceed the movement rate.

- **Delay config**: a single node-global `Uint16ConfigEntry` in units of 100 ms,
  passed to the constructor separately from the per-output repeated group. It is
  **clamped up to a safe minimum** (`ROUTED_MIN_STAGGER_DELAY`, 1.0 s) on read,
  so the consumer can never step faster than that floor. Default is 1.0 s.
- **Concurrency guidance** (surfaced in the CDI description): the number of
  motors that can be moving at once is roughly `ceil(travel_time / delay)`. Set
  the delay to at least the machine's travel time (~3 s for an MP10) for strict
  one-at-a-time movement; shorter values allow bounded overlap and higher peak
  current. The 1.0 s floor guarantees a sane rate, not strict single-mover.
- **Commit order**: for a route, the selected (THROWN) output commits first
  (immediate operator feedback), then its siblings close in ascending index
  order, each spaced by the delay.

### Mechanism

A **desired-state + deferred-commit** model rather than a FIFO queue:

1. `desiredState_[N]` holds the intended state of each output.
2. Event handlers update `desiredState_` only; they never touch GPIO directly.
   (A route computes "throw self, close group siblings" into `desiredState_` and
   marks the selected output as the priority commit.)
3. A `Timer` on the node's executor commits the next output whose physical state
   differs from its desired state — priority output first, then ascending index
   — then re-arms for `delay` until everything matches.

This is **self-coalescing and latest-wins**: a newer event that supersedes a
pending change just overwrites `desiredState_`, so there is no stale-movement
backlog and no unbounded queue. Spacing is preserved across sequences by
scheduling each commit no sooner than `delay` after the previous one
(`lastCommitTimeNsec_`). The timer runs on the same executor as event handling,
so no locking is required. A stagger delay of 0 is never used — the floor keeps
it at 1.0 s — so a single individual move still applies effectively immediately
(the first commit fires as soon as the executor picks it up), while a second
change within the window waits its turn.

## Implementation notes

- Base classes: `ConfigUpdateListener` + `SimpleEventHandler`, mirroring
  `MultiConfiguredConsumer` and `ConfiguredExclusiveConsumer`.
- **`user_arg` encoding**: three events per output are distinguished by encoding
  `user_arg = (i << 2) | type`, where `type ∈ {0 = closed, 1 = thrown,
  2 = route}`. Decode with `i = user_arg >> 2`, `type = user_arg & 3`.
- `apply_configuration` unregisters all handlers (on non-initial load) and
  re-registers `event_closed`, `event_thrown`, and `event_route` for every
  output, then returns `REINIT_NEEDED` to trigger an events-identify.
- **ConsumerIdentified** valid/invalid is derived from `pins_[i]->is_set()`
  versus the state implied by the event (THROWN and route imply set; CLOSED
  implies clear).
- Provide `factory_reset` and a `factory_reset_names(fd, basename)` helper,
  matching `ConfiguredExclusiveConsumer`.
- Include usage documentation and a usage example in the header, in the same
  style as the existing consumer classes.

## Startup / initial state

For now, startup assumes a fixed default state (no restore). A future
enhancement is to persist turnout state to non-volatile storage and restore the
previous state on boot; that is explicitly out of scope for this iteration.

## Naming

The class is named `ConfiguredRoutedConsumer`. The routing concept is more
general than turnouts — it is "exclusive-within-group selection with individual
override" — but the name reflects the routing behavior that motivates it.

## Relationship to existing classes

- `MultiConfiguredConsumer` — many GPIO pins, two events each, independent. The
  routed consumer adds a third (route) event and group-based cross-output
  actions.
- `ConfiguredExclusiveConsumer` — one group where exactly one output is active
  at a time, single event per output. The routed consumer generalizes this to
  multiple groups and adds independent closed/thrown control per output.

## Example wiring

A worked example lives in the Nucleo F303 IO board target
(`applications/nucleo_io/targets/freertos.armv7m.st-stm32f303re-nucleo-dev-board`).
Defining `PORTDE_ROUTED` in that target's `config.hxx` drives all 16 Port D/E
outputs as routed turnouts:

- `config.hxx` adds `using RoutedTurnouts = RepeatedGroup<RoutedConsumerConfig,
  16>`, a `routed_stagger_delay` (`Uint16ConfigEntry`, default 10 = 1.0 s), a
  `routed_turnouts` CDI group entry, and a `routed_routes`
  (`RepeatedGroup<RouteConfig, 8>`) route table.
- `main.cxx` instantiates `openlcb::ConfiguredRoutedConsumer` over the 16
  `PORTD_LINE*`/`PORTE_LINE*` outputs, passing the output group, the route
  table, and the stagger-delay entry.
- Enabling `PORTDE_ROUTED` raises the config to ~7150 bytes, so the target's
  `CONFIG_FILE_SIZE` `static_assert` is 7300 when `PORTDE_ROUTED` is defined and
  7000 otherwise (still ≥10% spare on the 8192-byte EEPROM).

Because it consumes the same physical lines, `PORTDE_ROUTED` is mutually
exclusive with `PORTD_EXCLUSIVE`, `PORTE_EXCLUSIVE`, and `PORTD_SNAP`; a
`#error` guard enforces this. The option is disabled by default, so the
shipping configuration is unchanged.

## Out of scope (future work)

- Pulsed / momentary output support.
- Per-output polarity inversion.
- Non-volatile state persistence and restore on startup.
