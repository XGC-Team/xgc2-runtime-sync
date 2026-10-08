# 04. Test And Acceptance Standards

## Acceptance Goal

Acceptance proves that the middleware can provide synchronized cycle events,
deadline-aware sample communication, structured cycle snapshots, clock and
network health, ROS1 adapter behavior, ground-station session control, and
explainable drop or status statistics under weak links.

## Test Environment

Minimum:

- One ground station.
- Two vehicles or Linux hosts.
- Same LAN.
- chrony or a mocked clock monitor.
- Zenoh router.
- ROS Noetic.

Recommended:

- One ground station.
- Three to five vehicles or Linux hosts.
- Wireless link or controllable network.
- `tc netem` fault injection.
- Log collection or dashboarding.

## Unit Tests

Cycle scheduler:

- Frequency/period validation accepts 0.5 Hz through 50 Hz and rejects values outside that range.
- Future epoch does not trigger early.
- cycle_id starts at 0 when epoch is reached.
- 20 Hz for 100 cycles increments continuously.
- Delayed callbacks do not replay historical cycles.
- XRPC stop request stops new cycles.

Envelope codec:

- Normal payload round trips.
- Empty payload round trips while metadata fields remain explicit.
- Bad checksum reports BadPayload.
- Wrong schema id reports BadSchema.
- Unsupported envelope schema_version is rejected; current supported value is 1.
- session_id, task_id, participant_id, channel, and schema_id are required and follow the task-key field rules.
- Payload larger than the configured limit is rejected; the default limit is 1 MiB.
- Bad magic, unsupported codec format version, truncated bytes, invalid field lengths, and oversized payload lengths are rejected before sample use.

Transport and task-semantic keys:

- Sample keys are task-semantic and include session, task, channel, and schema fields.
- Empty key fields and ROS graph-like names with slash separators are rejected.
- In-memory transport dispatches only to subscribers on the exact same key.
- Different task keys remain isolated.
- Unsubscribe stops further delivery for the removed subscriber.

Deadline checker:

- Target cycle before cutoff is Fresh.
- Arrival after cutoff is Late.
- Old target cycle is WrongCycle or Expired.
- Future target cycle is buffered.
- TTL expiry is marked unusable.

Sample buffer:

- One peer, channel, and cycle stores successfully.
- Duplicate sequence is marked Duplicate.
- Sequence gaps are counted.
- Multiple peers and channels remain isolated.
- Last-good lookup returns the latest usable sample.

Snapshot builder:

- All required peers fresh sets all_required_fresh.
- Missing required peer is reported Missing.
- Bad sender clock is reported SenderClockBad.
- Bad schema and late samples are counted.

## Integration Tests

Single-machine loopback:

- Start runtime.
- Publish a local sample.
- Receive it through loopback.
- Generate snapshot.

Two-machine Zenoh router:

- Ground station runs zenohd.
- Two runtimes connect to zenohd.
- Each side publishes channel `solution`.
- Each snapshot sees the other peer as Fresh.

Ground-station session start:

- Ground station distributes config.
- Vehicles ACK.
- Ground station sends XRPC start request with a shared epoch.
- Vehicles enter RUNNING at the same epoch and cycle id.

ROS1 adapter:

- Local algorithm publishes `/local_solution`.
- Runtime exports to Zenoh.
- Remote runtime imports to `/remote_solution/<peer>`.
- Unconfigured topics are not forwarded.
- Adapter rules are loaded from the `ros1_adapters` rosparam tree and startup
  fails when a configured tree is invalid or a `ros1_adapters_config` path is
  provided without loading the corresponding rosparam tree.
- Each adapter rule declares `name`, `topic`, `channel`, `schema_id`,
  `payload_type`, `enabled`, `max_payload_bytes`, and `required`.
- `name`, `channel`, and `schema_id` follow the task-semantic F3 field rules;
  `topic` starts with `/`; `max_payload_bytes` never exceeds the envelope
  payload limit.
- `/tf`, `/tf_static`, `/rosout`, `/clock`, and topic names containing
  `image`, `pointcloud`, `debug`, `camera`, or `raw` are rejected by default
  unless the rule explicitly sets `allow_large_payload: true`.
- Duplicate rule names and duplicate enabled `channel`/`schema_id` mappings
  are rejected.
- Runtime status or health reason includes the adapter rule summary, including
  the enabled/total rule count.

## Weak-Network Tests

Use `tools/netem_profiles.sh` to inject delay, loss, reorder, duplicate, and
corrupt conditions. Acceptance requires that late, missing, wrong-cycle,
duplicate, bad-payload, and sequence-gap counters remain explainable and that
old samples do not block new cycles.

## Time-Sync Tests

- Good chrony state allows session start.
- Unsynchronized chrony rejects start when required.
- Bad sender clock marks outgoing and incoming samples.
- High clock uncertainty produces degraded or bad health.
- Selected chrony source must match the configured ground station unless the
  deployment explicitly disables that gate.
- Default preflight start gate rejects offset or uncertainty above 2 ms.
- In-flight phase publishes degraded or bad clock health without calling
  `chronyc makestep`, `ntpdate`, or any other step command.
- `check_chrony.sh` is tested with fake `chronyc` output for PASS and FAIL
  paths.

## Performance Tests

Run 0.5, 2, 10, 20, and 50 Hz profiles for at least 10 minutes per profile.
Record CPU, memory, jitter p95 and p99, drops, sequence gaps, late samples, and
snapshot build cost. Initial acceptance expects no memory growth and no queue
backlog at 50 Hz.

## ROS1 Package Acceptance

After build metadata and source integration:

- `catkin_make` or `catkin build` passes.
- `roslaunch periodic_sync swarm_runtime.launch` starts.
- `/swarm_sync/cycle` publishes.
- `cycle_snapshot` publishes once per runtime cycle under the launch namespace.
- `sample_stats` publishes per-cycle sample counts under the launch namespace.
- Empty peer/sample input still publishes a CycleSnapshot with expected peer
  entries marked missing, without blocking cycle advancement.
- Runtime smoke tests receive both SyncedCycle and CycleSnapshot and verify
  matching cycle ids.
- XRPC `POST /v1/runtime-sync/status` responds.
- XRPC `POST /v1/runtime-sync/status` reports clock offset, uncertainty, quality,
  source, and phase.
- The configured allowlist controls all topic forwarding.
