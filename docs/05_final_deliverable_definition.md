# 05. Final Deliverable Definition

## Package Boundary

The final runtime package is a ROS1 package surface around a ROS-free
synchronized task communication core. It is not a full ROS1 graph bridge, a
flight-control safety policy, an MPC fallback library, a trajectory prediction
library, or a replacement for ROS2/DDS.

## Required Runtime Capabilities

- Synchronized local cycle generation.
- Task-semantic Zenoh sample exchange.
- Deadline-aware receive and publish classification.
- Opaque payload envelope handling.
- Per-cycle snapshots.
- Clock health monitoring.
- Runtime health and statistics.
- ROS1 import and export allowlists.
- Ground-station session start, stop, and monitoring.

## ROS1 Surface

Messages:

- SyncedCycle
- PeerSampleStatus
- CycleSnapshot
- RuntimeHealth
- SampleStats
- Existing legacy Sync* messages remain available.

Services:

- XRPC start request
- XRPC stop request
- XRPC status snapshot

Config:

- session.yaml
- channels.yaml
- ros1_adapters.yaml
- qos_policy.yaml

Launch:

- swarm_runtime.launch
- ground_station.launch

Tools:

- scripts/check_chrony.sh
- tools/netem_profiles.sh

## Integration Status

This first-phase staging adds resources only. Build metadata, package
dependencies, generated service targets, install rules, source files, and
headers still require a later integration step.
