# XGC2 Runtime Sync

ROS1 runtime synchronization messages and coordinator for XGC2 multi-agent systems.

This repository currently provides the `periodic_sync` ROS package. It coordinates synchronized periodic process triggering with participant readiness, trigger sequence IDs, timeout detection, cycle snapshots, and health statistics.

The weak-network layer is implemented under `swarm_sync_core/weaknet` and remains payload opaque. It provides channel QoS profiles, receive cutoff plus late-pocket classification, payload and traffic budget guards, per-peer/channel link health, deterministic send staggering, communication-only recommendations, and a GOOD/DEGRADED/CONGESTED/PARTITIONED/RECOVERING state machine. It does not apply algorithm fallback and it does not add per-sample ACKs to realtime traffic.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-runtime-sync
```

## Smoke Test

```bash
rospack find periodic_sync
rosrun periodic_sync sync_coordinator _num_uavs:=1
```

## Runtime

```bash
roslaunch periodic_sync swarm_runtime.launch control_socket:="$GRANTED_CONTROL_SOCKET"
roslaunch periodic_sync ground_station.launch participant_count:=3 frequency_hz:=20.0
```

Useful weak-network topics:

```text
/budget_report
/peer_link_health
/recommendation
/weaknet_state
```

## Clock Safety

Runtime Sync assumes the ground station is the LAN time authority. Vehicles
should synchronize only to that source during flight; external NTP/GPS sources
belong upstream of the ground station unless a deployment explicitly proves
otherwise.

Default gate:

```text
preflight offset <= 2 ms
preflight uncertainty <= 2 ms
selected chrony source == ground station
leap status == Normal
```

Preflight may use system-level chrony configuration such as `makestep` to
correct a large boot-time error before core ROS nodes or sessions start.
Runtime Sync itself never runs `chronyc makestep`.

In flight, system time step is treated as disallowed. Keep chrony slewing and
monitor `/swarm_sync/runtime_health` plus the XRPC `/v1/runtime-sync/status` snapshot;
when offset or uncertainty leaves the gate, mark the runtime degraded rather
than forcing a resync.

For synchronized actions, send a future execution timestamp and let each
vehicle execute when its synchronized local system time reaches that timestamp.
Do not trigger an NTP resync immediately before the action. Local control-loop
`dt` should use a monotonic/steady clock; cross-machine timestamps should use
the synchronized ROS/system time.

Preflight check:

```bash
GROUND_TIME_SOURCE=192.168.10.10 MAX_OFFSET_MS=2.0 MAX_UNCERTAINTY_MS=2.0 \
  rosrun periodic_sync check_chrony.sh
```

Local session control uses [Runtime Sync v1](contracts/runtime-sync-v1.md). The runtime owner must grant `GRANTED_CONTROL_SOCKET` in an existing private runtime directory. Platform ROS control services have been removed.
