# Runtime Sync v1

The `swarm_runtime_node` process owns one HTTP XRPC service, one native session,
and its lifecycle. C++20/GCC >=11 with matching libstdc++, Boost.JSON >=1.75 and
`XgcXrpc::http` are build requirements. ROS carries user cycle/sample/health
data; no ROS start, stop or status service is generated or advertised.

The process owner grants an existing private runtime directory and passes
`~control_socket` at startup (`swarm_runtime.launch control_socket:=...`).
The host never discovers or creates a durable data root. `GET
/v1/runtime-sync/describe` is the only unfenced discovery route. All other
calls are POST JSON with the SDK request ID, instance binding and finite
deadline. Exact fields below are enforced; additional fields are rejected.

| Method under `/v1/runtime-sync/` | Request |
| --- | --- |
| `status` | `{}` |
| `effective-policy` | `{}` |
| `start` | `{"expected_revision":1,"session_id":"run-2","task_id":"distributed-task","epoch_ns":1791482390000000000,"period_ns":50000000}` |
| `stop` | `{"expected_revision":2,"session_id":"run-2","reason":"operator stop"}` (`reason` optional) |
| `receipt` | `{"operation_id":"start-request-1"}` |
| `observe` | `{"after_revision":12}` |

Session/task/reason strings are at most 128 bytes; required IDs are nonempty.
Epoch is a nonnegative signed-64-bit nanosecond value; period is 20 ms–2 s
(0.5–50 Hz). `expected_revision` compares the desired configuration revision.
Stop also fences the session ID. Replacing an active or armed session requires
an explicit stop. Invalid fields, a bad clock gate and revision/session
conflicts do not replace the configuration. Roster and adapter settings are
startup inputs; live edits of those require restart.

An actual start acceptance response has this shape (additional clock, cycle
and task fields are maintained native state):

```json
{"schema_version":1,"instance_id":"fresh-instance","revision":12,
 "desired_revision":2,"applied_revision":1,"persisted_revision":null,
 "state":"ARMED","running":false,"session_id":"run-2",
 "receipt":{"operation_id":"start-request-1","state":"accepted",
 "desired_revision":2,"failure_stage":""}}
```

Only the native scheduler's ARMED→RUNNING transition sets applied revision 2
and marks the receipt `succeeded`. A clock failure before RUNNING marks it
`failed`; superseding a still pending start marks it `cancelled`. Terminal
receipts remain terminal after later stops/reconfigurations. Stop completes
when the native scheduler is stopped, so it cannot emit a new cycle. Already
published ROS data can remain in consumers' data queues.

`status` reads a cached native snapshot and never invokes chrony or ROS graph
probes. `observe` holds one reply until the native snapshot revision increases;
future revisions conflict. Clock/source/phase/quality describe the sampled
native state, not a new query-time probe. `start` and native cycle gates retain
the existing clock monitor. Its subprocess-based chrony sampling remains an
explicit performance gap; mock-clock native tests do not establish production
chrony latency.

The same request ID and identical route/body returns the retained receipt.
Reuse with another body conflicts; concurrent reuse conflicts. Receipts are
in memory, last 64 mutation results, and expire by oldest-result replacement.
Evicted IDs/receipts and process restarts do not promise durable deduplication
or exactly-once effects. No transport mutation is automatically replayed.

Resources: one IO thread, at most 16 connections, 16 in-flight calls, 16 native
command slots, 16 observation slots, 4 KiB requests and 16 KiB responses. Native
work keeps its slot until actual completion even if its caller disconnects.
Cancellation observed before native dispatch drops the command without effects.
A deadline/disconnect after native execution begins can mean outcome unknown;
use the retained receipt on the same service instance. ROS/native callbacks
are the sole session/scheduler writer; the IO owner parses and serializes JSON.

All configuration is ephemeral: desired/applied revisions start fresh in each
instance, persisted revision is null, and no scene/database/config file is
written. Runtime socket and SDK lease lock belong to the process owner's
private runtime allocation; SDK teardown removes the owned socket. The process
supervisor owns runtime-directory cleanup and bounded stderr/ROS log retention.
Deployment must allocate ROS_LOG_DIR/ROS_HOME rather than relying on developer
HOME. Managed deployment logging remains an integration gate. The composition root snapshots only XGC2_XRPC_ environment inputs once, resolves the formal C++ RuntimePolicy with declared product defaults/ceilings, and passes its effective HttpLimits to the host. `effective-policy` returns supported fields with value/source/ceiling metadata. Unknown, empty, unsupported and over-ceiling settings reject startup; diagnostics, client-registry and gRPC settings are not claimed by this HTTP host.

Native evidence: `tests/xrpc_native_smoke.sh` runs a dedicated rostest master
with a fresh transient allocation. It verifies real cycle data, removed ROS
services, revision conflicts, strict fields, stop postcondition, request-ID
replay/reuse, ARMED versus applied RUNNING, held observation and completion.
No live station is used and no deployment/weak-network claim follows from it.
