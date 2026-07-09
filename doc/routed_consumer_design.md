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

## Out of scope (future work)

- Pulsed / momentary output support.
- Per-output polarity inversion.
- Non-volatile state persistence and restore on startup.
