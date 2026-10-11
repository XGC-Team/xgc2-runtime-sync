# XGC2 Runtime Sync

ROS1 coordinator and messages that trigger several processes **on one host** in lock step.

This repository provides the ROS package `periodic_sync`: the `sync_coordinator` node and the messages it uses
(`SyncReady`, `SyncTrigger`, `SyncAck`, `SyncStatistics`, `SyncParticipantStats`, `SyncEvent`).

## Scope

- **Same-host, multi-process synchronized triggering only.** The coordinator and all participants are processes on the
  same machine, in one ROS graph, on one system clock.
- The purpose is early validation of distributed synchronous MPC such as the TRO DMPC: every agent runs in its own
  process and solves its own problem, and all agents start a cycle on the same trigger. The algorithm stays
  distributed. The coordinator only aligns the cycles; it does not solve anything.
- **This is not a multi-host or networked synchronization product.** There is no cross-host transport, no clock
  synchronization (no chrony, NTP or PTP handling) and no weak-network adaptation in this package, and there is no
  roadmap for any of them. An earlier framework for these topics was never connected and was removed in 2.0.0; it
  remains in the git history up to commit `e521e06`.
- The package is in maintenance mode. It is kept so that the existing early validation can be reproduced; changes are
  limited to what that use directly needs.

## Install

For ROS Noetic on Ubuntu 20.04:

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-runtime-sync
rospack find periodic_sync
```

## Protocol

One round trip per cycle: Ready, then a periodic sequenced Trigger, then an Ack from every participant.

| Topic (default) | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/sync/ready` | `SyncReady` | participant to coordinator | Publish latched, so the coordinator sees it even if it starts later. |
| `/sync/trigger` | `SyncTrigger` | coordinator to participants | `sequence_id` counts from 1 without gaps. |
| `/sync/ack` | `SyncAck` | participant to coordinator | One per trigger, with the trigger's `sequence_id`. |
| `/sync/statistics` | `SyncStatistics` | coordinator to observers | Latched. |
| `/sync/events` | `SyncEvent` | coordinator to observers | Not latched. |

1. **Ready.** Each participant publishes `SyncReady{participant_id, node_name, ready_time}` once it can run a cycle.
   Participant ids are `first_participant_id` to `first_participant_id + participant_count - 1`; messages from other ids
   are reported as `EVENT_UNKNOWN_PARTICIPANT` and ignored. When all participants are ready the coordinator publishes
   `EVENT_ALL_READY` and starts its timer.
2. **Trigger.** Every `sync_period` seconds the coordinator publishes
   `SyncTrigger{sequence_id, trigger_time, active_participant_ids}`. `trigger_time` is the coordinator's ROS time at
   publication. A participant runs exactly one cycle per trigger that lists it as active.
3. **Ack.** The participant answers with
   `SyncAck{participant_id, sequence_id, received_time, finished_time, computation_ms, success, error_msg}`. The
   cycle time is `finished_time - received_time`; both stamps come from the ROS clock of the participant's host.
   `success: false` with an `error_msg` reports a failed cycle.
4. **Timeouts and statistics.** A participant that has not acked `timeout_threshold_ms` after a trigger gets
   `EVENT_MISSED`. After `max_consecutive_timeouts` consecutive misses, slow cycles or failed acks it is
   `STATUS_DEGRADED` (`EVENT_DEGRADED`); it recovers after `recovery_good_cycles` consecutive healthy acks
   (`EVENT_RECOVERED`). `SyncStatistics` reports, for the latest round, the expected, actual and missing response
   counts, the round finish time and the straggler, and for every participant the readiness, counters, status and the
   EWMA, mean, max and standard deviation of computation and cycle times (`SyncParticipantStats`).

`SyncEvent` types: `EVENT_READY`, `EVENT_ALL_READY`, `EVENT_SLOW`, `EVENT_MISSED`, `EVENT_LATE_ACK`, `EVENT_FAILURE`,
`EVENT_RECOVERED`, `EVENT_UNKNOWN_PARTICIPANT`, `EVENT_CLOCK_ANOMALY`, `EVENT_DEGRADED`.
`SyncParticipantStats` statuses: `STATUS_OK`, `STATUS_SLOW`, `STATUS_MISSING`, `STATUS_FAILED`, `STATUS_DEGRADED`.

## Parameters of `sync_coordinator`

All parameters are private (`~`).

| Parameter | Default | Meaning |
| --- | --- | --- |
| `participant_count` | if unset: `num_participants`, then `num_uavs`, then 4 | Number of participants. |
| `first_participant_id` | 1 | Id of the first participant; ids are consecutive. |
| `sync_period` | `sampling_time` | Trigger period in seconds. |
| `sampling_time` | 0.1 | Default for `sync_period`. |
| `timeout_threshold_ms` | 100.0 | Time a participant has to ack a trigger. |
| `max_consecutive_timeouts` | 3 | Consecutive misses, slow cycles or failures before `STATUS_DEGRADED`. |
| `slow_ratio_threshold` | 0.8 | A cycle longer than `sync_period` times this ratio is slow. |
| `recovery_good_cycles` | 5 | Consecutive healthy acks that end a degraded or abnormal state. |
| `stats_window_size` | 100 | Samples per participant for mean, max and standard deviation. |
| `statistics_publish_period` | 1.0 | Seconds between periodic statistics; 0 disables the timer. |
| `statistics_publish_every_n_cycles` | 0 | Also publish statistics every N triggers; 0 disables. |
| `ready_topic`, `trigger_topic`, `ack_topic`, `statistics_topic`, `events_topic` | `/sync/ready`, `/sync/trigger`, `/sync/ack`, `/sync/statistics`, `/sync/events` | Topic names. |
| `sync_queue_size` | `participant_count * 4`, at least 50 | Subscriber queue size for ready and ack. |
| `spin_rate_hz` | 200.0 | Rate of the coordinator's spin loop. |

The coordinator also publishes statistics when a participant becomes ready, when all are ready and after each
response timeout check.

## Usage

Quick start with three participants:

```bash
rosrun periodic_sync sync_coordinator _participant_count:=3 _sync_period:=0.1
```

The existing consumer is `unicycle_ugv_controller/launch/xgc_mixed_planning.launch`, which starts the node with the
scenario file and the topic names of the TRO DMPC deployment:

```xml
<node name="sync_coordinator" pkg="periodic_sync" type="sync_coordinator" required="true" output="$(arg node_output)">
  <rosparam command="load" file="$(arg scenario_dir)/scenario.yaml"/>
  <rosparam command="load" file="$(find unicycle_ugv_controller)/config/launch/xgc_mixed_planning_8.yaml"/>
</node>
```

`xgc_mixed_planning_8.yaml` contains:

```yaml
participant_count: 9
first_participant_id: 1
ready_topic: /formation/ready
trigger_topic: /formation/sync_trigger
ack_topic: /formation/sync_ack
statistics_topic: /formation/sync_statistics
events_topic: /formation/sync_events
```

## Message stability

The messages and the coordinator are unchanged since the first release. Their MD5 sums are identical to the
`periodic_sync` copy frozen with the TRO DMPC submission (`alg_ros1_ws` `b030e8e8`). A node built against a
`periodic_sync` copy with different message definitions does not connect to nodes built against this one, so the
definitions must not change. The test below asserts the sums.

## Tests

```bash
catkin_make run_tests_periodic_sync   # in a catkin workspace that contains this package
```

`test/sync_coordinator.test` is a rostest that starts `sync_coordinator` with three `fake_sync_participant` processes
on the `/formation/*` topics and checks, from a fourth process that records everything the coordinator publishes:

- no trigger before all three participants are ready;
- consecutive trigger sequence ids from 1, the active participant ids and the trigger period;
- the acks show up in the statistics (last acked sequence, computation and cycle times, round completeness);
- a participant that stops acking is reported as missed, then degraded, with the matching statistics, and recovers
  once it acks again;
- an unknown participant id is reported and ignored;
- the MD5 sums of the six messages.

This covers the coordination path only. It does not run any algorithm and does not replace the TRO DMPC reproduction,
which needs the academic packages, their acados solvers, MAVROS/PX4 and Gazebo.
