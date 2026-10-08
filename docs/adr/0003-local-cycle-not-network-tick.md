# ADR 0003: Local Cycles Instead Of Network Ticks

## Status

Accepted for staging.

## Decision

Each vehicle generates cycle events locally from a shared epoch and synchronized
system clock. The ground station does not broadcast a tick every cycle.

## Rationale

Per-cycle ground-station ticks add wireless latency, jitter, and a central
runtime dependency. A shared epoch plus synchronized clock lets each vehicle
advance cycles locally while still exposing timing evidence.

## Consequences

- Clock quality is a hard runtime input.
- XRPC start request must include a shared epoch and period.
- Missed historical cycles are skipped rather than replayed.
- The runtime reports expected time, actual time, jitter, and clock quality for
  each cycle.
