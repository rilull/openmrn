# ConfiguredRoutedConsumer — Plan and Outcome

This is the project journal for the `ConfiguredRoutedConsumer` effort: the
running plan, the decisions made along the way, and the outcome. It complements
`doc/routed_consumer_design.md`, which is the technical design guide for the
finished class.

## Context and goal

A node drives up to 16 switch machine motors (Tortoise via a TC4428 driver, or
MP10 in 2-wire mode). The goal was two layers of control from OpenLCB events:

1. **Individual control** — move any single turnout to CLOSED or THROWN.
2. **Single-button routing** — one Event ID lines a whole path so an operator
   pushes one button to select a track and the routing happens automatically.

Development was incremental and discussion-driven: each capability was agreed on,
implemented, host-verified, committed, and pushed before moving to the next.

## Branch

All work lives on a single branch, **`claude/turnout-control-consumer-p18t1x`**
(tracking `origin/claude/turnout-control-consumer-p18t1x`). It was **not** split
across fresh branches; the commits below are stacked in order on top of the
pre-existing exclusive-consumer work (`ConfiguredExclusiveConsumer`).

## Timeline of increments

Each step lists the decision that shaped it and the commit(s) that delivered it.

1. **Grouped routing consumer + design guide** — `eaa1997`, `11b7e6a`
   - `ConfiguredRoutedConsumer`: per-output CLOSED/THROWN events plus a per-output
     *route* event and a route-group dropdown. A route event throws its output
     and closes all same-group siblings (exclusive-within-group).
   - Decisions: single GPIO per turnout; fixed polarity (`set` = THROWN, `clr` =
     CLOSED); group dropdown with an explicit **None** opt-out; constant-on
     outputs (no pulse support); default startup state (NV restore deferred).

2. **Nucleo F303 usage example** — `1665837`, `3bf80b6`
   - Wired the consumer into the Nucleo F303 IO board target over the 16
     shift-register **Port D + E** outputs, behind an opt-in `PORTDE_ROUTED`
     macro (off by default; `#error`-guarded as mutually exclusive with the
     exclusive/snap Port D/E options). Bumped `CANONICAL_VERSION`.
   - Decision: use the board's shift-register Port D/E lines (16) rather than raw
     MCU Port D pins, to avoid the SRV8/PD2 conflict and stay consistent with the
     existing exclusive-consumer example.

3. **Staggered movement / current limiting** — `1b66034`
   - Output changes are throttled: committed one at a time with a configurable
     minimum spacing, so only a bounded number of motors move (draw current) at
     once. MP10s pull ~100 mA only while traveling (~3 s), so spacing bounds peak
     current for LCC compliance.
   - Decisions: **global** throttle (individual + route events alike); a
     node-global stagger delay in 100 ms units, **clamped up to a 1.0 s safety
     floor**; a route commits its **thrown/selected output first**, then siblings
     in index order. Mechanism: desired-state + deferred-commit driven by a Timer
     on the node's executor (latest-wins, self-coalescing, no unbounded queue).

4. **Sparse route tables for cascade ladders** — `ecb9196`
   - Added an explicit **route table** alongside the group model. A route is a
     trigger event plus a **sparse `(turnout, state)` action list**; turnouts not
     listed are left unchanged (true don't-care). This lines cascade ladders,
     where reaching a track needs an arbitrary pattern of turnout positions and
     turnouts off the path do not matter.
   - Decisions: **extend the one class** (group model, individual events, and
     route table all coexist and feed the same staggering engine); **sparse**
     representation (omission = don't-care); **8 routes** in the example;
     `MAX_ROUTE_ACTIONS = 8`. `user_arg` reuses the spare `type = 3` code; route
     actions are cached in RAM at `apply_configuration`; identify reports a route
     VALID iff every listed turnout already matches.

## Key design decisions (summary)

| Area | Decision |
|------|----------|
| Output drive | Single GPIO per turnout (TC4428 / MP10 2-wire); constant-on |
| Polarity | Fixed: `set` = THROWN, `clr` = CLOSED (no invert option) |
| Group routing | Exclusive-within-group; explicit **None** opt-out |
| Current limiting | Global stagger throttle; 100 ms units; **1.0 s floor**; thrown-first |
| Cascade routing | **Route table**, sparse `(turnout, state)`; omission = don't-care |
| Class structure | One class carries individual + group + route-table routing |
| Example scale | Nucleo F303, 16 Port D/E outputs, 8 routes, opt-in `PORTDE_ROUTED` |
| EEPROM budget | `CONFIG_FILE_SIZE` cap 7300 when `PORTDE_ROUTED` else 7000 |

## Outcome and status

**Implemented and host-verified** (compile/type-check with `g++ -std=c++14`,
gtest/ARM toolchains unavailable in the dev environment):

- `src/openlcb/ConfiguredRoutedConsumer.hxx` — full class (individual events,
  group routing, staggering engine, sparse route table, identify). Header and the
  templated constructor body type-check via forced instantiation.
- `src/openlcb/ConfiguredRoutedConsumer.cxxtest` — unit tests for individual
  control, in-group/full-group routing, None group, staggering (thrown-first,
  spacing), cascade route don't-care, route re-lining, and identify.
- Nucleo F303 example (`config.hxx` + shared `main.cxx`) compiles in both
  `PORTDE_ROUTED` off (default) and on branches; nested `routed_routes(...)`
  accessors resolve.
- **Measured config sizes** (F303): routed OFF **6812 B** (≤ 7000), routed ON
  **7150 B** (≤ 7300). `NUM_MCPIOS = 8`.

**Not yet run** (require environment/hardware not available here):

- gtest unit tests (`make tests`) — note they run in real wall-clock time because
  of the 1 s stagger floor.
- ARM firmware build/link (`arm-none-eabi-g++`) of the F303 target.
- JMRI review of the generated `cdi.xml` for the route table's ergonomics.
- On-hardware validation with real Tortoise/MP10 machines.

## Open items and next steps

- Run the unit tests and an ARM build of the F303 target with `PORTDE_ROUTED`
  enabled; inspect `cdi.xml` in JMRI before flashing.
- **Revisit if warranted**: if the sparse route entry proves cumbersome to
  configure in JMRI, switch to a dense per-turnout representation or split the
  class — both are contained changes.
- Raise `MAX_ROUTE_ACTIONS` if a layout needs routes deeper than 8 turnouts.
- Future feature: persist turnout state to non-volatile storage and restore on
  startup (explicitly deferred throughout).
