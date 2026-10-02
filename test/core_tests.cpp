#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm_sync_core/clock_monitor.hpp"
#include "swarm_sync_core/cycle_scheduler.hpp"
#include "swarm_sync_core/deadline_checker.hpp"
#include "swarm_sync_core/envelope_codec.hpp"
#include "swarm_sync_core/sample_buffer.hpp"
#include "swarm_sync_core/session.hpp"
#include "swarm_sync_core/snapshot_builder.hpp"
#include "swarm_sync_core/transport.hpp"
#include "swarm_sync_core/transport_keys.hpp"
#include "swarm_sync_core/weaknet/weaknet.hpp"
#include "swarm_sync_ros1/adapter_config.hpp"

namespace {

using namespace swarm_sync;

SampleEnvelope makeEnvelope(uint64_t cycle, uint64_t seq = 1) {
  SampleEnvelope envelope;
  envelope.session_id = "session-a";
  envelope.team_id = "team-a";
  envelope.task_id = "formation";
  envelope.participant_id = "uav2";
  envelope.channel = "solution";
  envelope.sender_id = "uav2";
  envelope.node_id = "planner";
  envelope.seq = seq;
  envelope.produce_cycle = cycle;
  envelope.target_cycle = cycle;
  envelope.t_cycle_start_ns = 1000000000;
  envelope.t_produce_start_ns = 1000001000;
  envelope.t_produce_finish_ns = 1000002000;
  envelope.t_publish_ns = 1000003000;
  envelope.period_ns = 50000000;
  envelope.publish_deadline_ns = 10000000;
  envelope.ttl_ns = 40000000;
  envelope.producer_status = ProducerStatus::Success;
  envelope.clock_ok = true;
  envelope.payload_type = "bytes";
  envelope.schema_id = "solution.v1";
  envelope.payload = {1, 2, 3, 4};
  envelope.payload_crc32c = EnvelopeCodec::crc32c(envelope.payload.data(), envelope.payload.size());
  return envelope;
}

SessionConfig makeSessionConfig() {
  SessionConfig config;
  config.team_id = "team-a";
  config.session_id = "session-a";
  config.task_id = "formation";
  config.self_id = "uav1";
  config.required_participants = {"uav1", "uav2"};
  config.epoch_ns = 1000;
  config.period_ns = 100;
  return config;
}

void writeU32At(std::vector<uint8_t>* bytes, size_t offset, uint32_t value) {
  ASSERT_LE(offset + 4, bytes->size());
  for (int i = 0; i < 4; ++i) {
    (*bytes)[offset + static_cast<size_t>(i)] =
        static_cast<uint8_t>((value >> (i * 8)) & 0xffu);
  }
}

}  // namespace

TEST(SessionManagerTest, ConfiguresArmsAndStartsWhenDue) {
  SessionManager manager;
  const auto config = makeSessionConfig();

  EXPECT_TRUE(manager.configure(config));
  EXPECT_EQ(SessionState::CONFIGURED, manager.state());
  EXPECT_TRUE(manager.arm(2000));
  EXPECT_EQ(SessionState::ARMED, manager.state());
  EXPECT_FALSE(manager.startIfDue(1999));
  EXPECT_EQ(SessionState::ARMED, manager.state());
  EXPECT_TRUE(manager.startIfDue(2000));
  EXPECT_TRUE(manager.running());
}

TEST(SessionManagerTest, RejectsInvalidConfigAndBadClock) {
  SessionManager manager;
  auto config = makeSessionConfig();
  config.period_ns = 0;
  EXPECT_FALSE(manager.configure(config));
  EXPECT_EQ(SessionState::ERROR, manager.state());

  EXPECT_TRUE(manager.configure(makeSessionConfig()));
  ClockState bad_clock;
  bad_clock.clock_ok = false;
  bad_clock.quality = ClockQuality::BAD;
  EXPECT_FALSE(manager.arm(2000, bad_clock));
  EXPECT_EQ(SessionState::ERROR, manager.state());
}

TEST(ClockMonitorTest, ParsesChronyTrackingAndSelectedSource) {
  const std::string tracking_text =
      "Reference ID    : C0A80A0A (192.168.10.10)\n"
      "System time     : 0.000500000 seconds fast of NTP time\n"
      "Root delay      : 0.000400000 seconds\n"
      "Root dispersion : 0.000300000 seconds\n"
      "Leap status     : Normal\n";
  const std::string sources_text =
      "MS Name/IP address         Stratum Poll Reach LastRx Last sample\n"
      "===============================================================================\n"
      "^* 192.168.10.10                 1   6   377    12   +500us[ +500us] +/-  900us\n"
      "^- pool.ntp.org                  2   6   377    11   -800us[ -800us] +/-   20ms\n";

  const auto tracking = ClockMonitor::parseTracking(tracking_text);
  const auto sources = ClockMonitor::parseSources(sources_text);

  EXPECT_TRUE(tracking.ok);
  EXPECT_EQ("C0A80A0A", tracking.reference_id);
  EXPECT_EQ("192.168.10.10", tracking.reference_name);
  EXPECT_DOUBLE_EQ(0.0005, tracking.system_time_s);
  EXPECT_EQ("Normal", tracking.leap_status);
  EXPECT_TRUE(sources.ok);
  EXPECT_EQ("192.168.10.10", sources.selected_source);
}

TEST(ClockMonitorTest, AllowsStartWhenGroundSourceWithinTwoMilliseconds) {
  ClockMonitorSnapshot snapshot;
  snapshot.tracking = ClockMonitor::parseTracking(
      "Reference ID    : C0A80A0A (192.168.10.10)\n"
      "System time     : 0.001500000 seconds slow of NTP time\n"
      "Root delay      : 0.000200000 seconds\n"
      "Root dispersion : 0.000100000 seconds\n"
      "Leap status     : Normal\n");
  snapshot.sources = ClockMonitor::parseSources(
      "^* 192.168.10.10 1 6 377 12 +1500us[+1500us] +/- 500us\n");
  ClockPolicy policy;
  policy.ground_time_source = "192.168.10.10";

  const auto result = ClockMonitor::evaluateSnapshot(snapshot, policy, ClockPhase::Preflight);

  EXPECT_TRUE(result.state.clock_ok);
  EXPECT_TRUE(result.start_allowed);
  EXPECT_EQ(ClockQuality::OK, result.state.quality);
  EXPECT_EQ(-1500000, result.state.offset_ns);
  EXPECT_LE(result.state.uncertainty_ns, 2000000u);
}

TEST(ClockMonitorTest, RejectsLargeOffsetWrongSourceAndLeapAnomaly) {
  ClockPolicy policy;
  policy.ground_time_source = "192.168.10.10";

  ClockMonitorSnapshot large_offset;
  large_offset.tracking = ClockMonitor::parseTracking(
      "Reference ID    : C0A80A0A (192.168.10.10)\n"
      "System time     : 0.004000000 seconds fast of NTP time\n"
      "Root delay      : 0.000100000 seconds\n"
      "Root dispersion : 0.000100000 seconds\n"
      "Leap status     : Normal\n");
  large_offset.sources = ClockMonitor::parseSources(
      "^* 192.168.10.10 1 6 377 12 +4000us[+4000us] +/- 500us\n");
  const auto large_offset_result =
      ClockMonitor::evaluateSnapshot(large_offset, policy, ClockPhase::Preflight);
  EXPECT_FALSE(large_offset_result.state.clock_ok);
  EXPECT_FALSE(large_offset_result.start_allowed);
  EXPECT_NE(std::string::npos, large_offset_result.reason.find("offset"));

  ClockMonitorSnapshot wrong_source = large_offset;
  wrong_source.tracking = ClockMonitor::parseTracking(
      "Reference ID    : 8.8.8.8 (time.google.com)\n"
      "System time     : 0.000100000 seconds fast of NTP time\n"
      "Root delay      : 0.000100000 seconds\n"
      "Root dispersion : 0.000100000 seconds\n"
      "Leap status     : Normal\n");
  wrong_source.sources = ClockMonitor::parseSources(
      "^* time.google.com 1 6 377 12 +100us[+100us] +/- 500us\n");
  const auto wrong_source_result =
      ClockMonitor::evaluateSnapshot(wrong_source, policy, ClockPhase::Preflight);
  EXPECT_FALSE(wrong_source_result.state.clock_ok);
  EXPECT_NE(std::string::npos, wrong_source_result.reason.find("ground station"));

  ClockMonitorSnapshot leap_bad = large_offset;
  leap_bad.tracking = ClockMonitor::parseTracking(
      "Reference ID    : C0A80A0A (192.168.10.10)\n"
      "System time     : 0.000100000 seconds fast of NTP time\n"
      "Root delay      : 0.000100000 seconds\n"
      "Root dispersion : 0.000100000 seconds\n"
      "Leap status     : Not synchronised\n");
  leap_bad.sources = ClockMonitor::parseSources(
      "^* 192.168.10.10 1 6 377 12 +100us[+100us] +/- 500us\n");
  const auto leap_result =
      ClockMonitor::evaluateSnapshot(leap_bad, policy, ClockPhase::Preflight);
  EXPECT_FALSE(leap_result.state.clock_ok);
  EXPECT_NE(std::string::npos, leap_result.reason.find("leap"));
}

TEST(CycleSchedulerTest, TickIsControlledAndDoesNotBackfillMissedCycles) {
  CycleScheduler scheduler;
  CycleSchedulerConfig config;
  config.session_id = "session-a";
  config.task_id = "formation";
  config.epoch_ns = 1000000000;
  config.period_ns = 50000000;
  ASSERT_TRUE(scheduler.configure(config));

  std::vector<uint64_t> callback_cycles;
  scheduler.setCallback([&callback_cycles](const CycleEvent& event) {
    callback_cycles.push_back(event.cycle_id);
  });
  scheduler.start();

  EXPECT_FALSE(scheduler.tick(999999999).has_value());
  auto first = scheduler.tick(1000000000);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(0u, first->cycle_id);
  EXPECT_EQ(0, first->jitter_ns);

  EXPECT_FALSE(scheduler.tick(1000001000).has_value());
  auto skipped_to_three = scheduler.tick(1150005000);
  ASSERT_TRUE(skipped_to_three.has_value());
  EXPECT_EQ(3u, skipped_to_three->cycle_id);
  EXPECT_EQ(1000000000 + 3 * 50000000, skipped_to_three->expected_time_ns);
  EXPECT_EQ(1150005000, skipped_to_three->actual_time_ns);
  EXPECT_EQ(5000, skipped_to_three->jitter_ns);
  EXPECT_EQ((std::vector<uint64_t>{0, 3}), callback_cycles);

  scheduler.stop();
  EXPECT_FALSE(scheduler.tick(1200000000).has_value());
}

TEST(CycleSchedulerTest, ValidatesFrequencyAndPeriodBounds) {
  EXPECT_TRUE(CycleScheduler::isValidFrequencyHz(0.5));
  EXPECT_TRUE(CycleScheduler::isValidFrequencyHz(20.0));
  EXPECT_TRUE(CycleScheduler::isValidFrequencyHz(50.0));
  EXPECT_FALSE(CycleScheduler::isValidFrequencyHz(0.49));
  EXPECT_FALSE(CycleScheduler::isValidFrequencyHz(50.1));
  EXPECT_FALSE(CycleScheduler::isValidFrequencyHz(0.0));
  EXPECT_FALSE(CycleScheduler::isValidFrequencyHz(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(CycleScheduler::isValidFrequencyHz(std::numeric_limits<double>::quiet_NaN()));

  ASSERT_TRUE(CycleScheduler::periodNsFromFrequencyHz(0.5).has_value());
  EXPECT_EQ(CycleScheduler::kMaxPeriodNs, *CycleScheduler::periodNsFromFrequencyHz(0.5));
  ASSERT_TRUE(CycleScheduler::periodNsFromFrequencyHz(50.0).has_value());
  EXPECT_EQ(CycleScheduler::kMinPeriodNs, *CycleScheduler::periodNsFromFrequencyHz(50.0));
  EXPECT_FALSE(CycleScheduler::periodNsFromFrequencyHz(0.49).has_value());
  EXPECT_FALSE(CycleScheduler::periodNsFromFrequencyHz(50.1).has_value());

  EXPECT_TRUE(CycleScheduler::isValidPeriodNs(CycleScheduler::kMinPeriodNs));
  EXPECT_TRUE(CycleScheduler::isValidPeriodNs(CycleScheduler::kMaxPeriodNs));
  EXPECT_FALSE(CycleScheduler::isValidPeriodNs(CycleScheduler::kMinPeriodNs - 1));
  EXPECT_FALSE(CycleScheduler::isValidPeriodNs(CycleScheduler::kMaxPeriodNs + 1));
  EXPECT_FALSE(CycleScheduler::isValidPeriodNs(0));
  EXPECT_FALSE(CycleScheduler::isValidPeriodNs(-1));

  CycleSchedulerConfig config;
  config.session_id = "session-a";
  config.task_id = "formation";
  config.epoch_ns = 1000000000;
  config.period_ns = CycleScheduler::kMinPeriodNs - 1;
  CycleScheduler scheduler;
  EXPECT_FALSE(scheduler.configure(config));

  config.period_ns = CycleScheduler::kMaxPeriodNs + 1;
  EXPECT_FALSE(scheduler.configure(config));

  config.period_ns = 50000000;
  EXPECT_TRUE(scheduler.configure(config));
}

TEST(CycleSchedulerTest, EmitsExpectedActualAndJitterForTwentyHzHundredCycles) {
  constexpr int64_t kEpochNs = 1000000000;
  constexpr int64_t kPeriodNs = 50000000;
  CycleScheduler scheduler;
  CycleSchedulerConfig config;
  config.session_id = "session-a";
  config.task_id = "formation";
  config.epoch_ns = kEpochNs;
  config.period_ns = kPeriodNs;
  ASSERT_TRUE(scheduler.configure(config));
  scheduler.start();

  for (uint64_t cycle = 0; cycle < 100; ++cycle) {
    const int64_t expected_time_ns = kEpochNs + static_cast<int64_t>(cycle) * kPeriodNs;
    const int64_t injected_jitter_ns = static_cast<int64_t>(cycle % 7) * 1000;
    const int64_t actual_time_ns = expected_time_ns + injected_jitter_ns;
    const auto event = scheduler.tick(actual_time_ns);
    ASSERT_TRUE(event.has_value()) << "cycle " << cycle;
    EXPECT_EQ(cycle, event->cycle_id);
    EXPECT_EQ(expected_time_ns, event->expected_time_ns);
    EXPECT_EQ(actual_time_ns, event->actual_time_ns);
    EXPECT_EQ(injected_jitter_ns, event->jitter_ns);
    EXPECT_EQ(kPeriodNs, event->period_ns);
  }
  EXPECT_FALSE(scheduler.tick(kEpochNs + 99 * kPeriodNs + 10000).has_value());
}

TEST(CycleSchedulerTest, DueMatchesTickWithoutSideEffects) {
  CycleScheduler scheduler;
  CycleSchedulerConfig config;
  config.session_id = "session-a";
  config.task_id = "formation";
  config.epoch_ns = 1000000000;
  config.period_ns = 50000000;
  config.start_cycle = 2;
  config.max_cycle = 8;
  ASSERT_TRUE(scheduler.configure(config));
  EXPECT_FALSE(scheduler.due(config.epoch_ns));  // not started

  scheduler.start();
  // 5 ms poll ticks from before the epoch past max_cycle, plus a jump that
  // skips cycles and repeated polls inside one period.
  std::vector<int64_t> times;
  for (int64_t t = config.epoch_ns - 20000000; t < config.epoch_ns + 500000000; t += 5000000) {
    times.push_back(t);
    times.push_back(t);
  }
  times.push_back(config.epoch_ns + 120000000);
  size_t emitted = 0;
  for (const int64_t t : times) {
    const bool due = scheduler.due(t);
    EXPECT_EQ(due, scheduler.due(t)) << t;
    const auto event = scheduler.tick(t);
    EXPECT_EQ(due, event.has_value()) << t;
    emitted += event.has_value() ? 1u : 0u;
  }
  EXPECT_EQ(7u, emitted);  // cycles 2..8, one emission each
  EXPECT_FALSE(scheduler.running());

  scheduler.start();
  scheduler.stop();
  EXPECT_FALSE(scheduler.due(config.epoch_ns + 1000000000));
}

TEST(CycleSchedulerTest, StopPreventsNewCycles) {
  CycleScheduler scheduler;
  CycleSchedulerConfig config;
  config.session_id = "session-a";
  config.task_id = "formation";
  config.epoch_ns = 1000000000;
  config.period_ns = 50000000;
  ASSERT_TRUE(scheduler.configure(config));
  scheduler.start();

  ASSERT_TRUE(scheduler.tick(1000000000).has_value());
  scheduler.stop();

  EXPECT_FALSE(scheduler.running());
  EXPECT_FALSE(scheduler.tick(1050000000).has_value());
  EXPECT_FALSE(scheduler.tick(1100000000).has_value());
}

TEST(EnvelopeCodecTest, EncodesDecodesAndValidatesSchemaAndCrc) {
  EnvelopeCodec codec;
  codec.allowSchema("solution", "bytes", "solution.v1");

  const auto envelope = makeEnvelope(5);
  const auto bytes = codec.encode(envelope);
  const auto decoded = codec.decode(bytes);

  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_EQ(envelope.session_id, decoded.envelope.session_id);
  EXPECT_EQ(envelope.participant_id, decoded.envelope.participant_id);
  EXPECT_EQ(envelope.channel, decoded.envelope.channel);
  EXPECT_EQ(envelope.payload, decoded.envelope.payload);
  EXPECT_EQ(EnvelopeCodec::crc32c(envelope.payload.data(), envelope.payload.size()),
            decoded.envelope.payload_crc32c);

  EnvelopeCodec schema_rejecting_codec;
  schema_rejecting_codec.allowSchema("solution", "bytes", "solution.v2");
  const auto bad_schema = schema_rejecting_codec.decode(bytes);
  EXPECT_FALSE(bad_schema.ok);
  EXPECT_EQ(SampleStatus::BadSchema, bad_schema.status);

  auto corrupted = bytes;
  corrupted.back() ^= 0xffu;
  const auto bad_crc = codec.decode(corrupted);
  EXPECT_FALSE(bad_crc.ok);
  EXPECT_EQ(SampleStatus::BadPayload, bad_crc.status);
}

TEST(EnvelopeCodecTest, AllowsEmptyPayloadWithMetadataIntact) {
  EnvelopeCodec codec;
  auto envelope = makeEnvelope(6);
  envelope.payload.clear();
  envelope.payload_crc32c = EnvelopeCodec::crc32c(envelope.payload.data(), envelope.payload.size());

  const auto decoded = codec.decode(codec.encode(envelope));

  ASSERT_TRUE(decoded.ok) << decoded.error;
  EXPECT_TRUE(decoded.envelope.payload.empty());
  EXPECT_EQ("session-a", decoded.envelope.session_id);
  EXPECT_EQ("formation", decoded.envelope.task_id);
  EXPECT_EQ("uav2", decoded.envelope.participant_id);
  EXPECT_EQ("solution", decoded.envelope.channel);
  EXPECT_EQ("solution.v1", decoded.envelope.schema_id);
}

TEST(EnvelopeCodecTest, RejectsOversizedPayload) {
  EnvelopeCodec codec;
  auto envelope = makeEnvelope(6);
  envelope.payload.assign(kDefaultMaxPayloadBytes + 1, 0x42);

  const auto validation = validateSampleEnvelope(
      envelope, SampleEnvelopeValidationOptions{kDefaultMaxPayloadBytes, false});
  EXPECT_FALSE(validation.ok);
  EXPECT_EQ(SampleStatus::BadPayload, validation.status);
  EXPECT_NE(std::string::npos, validation.error.find("payload"));
  EXPECT_THROW(codec.encode(envelope), std::invalid_argument);
}

TEST(EnvelopeCodecTest, RejectsBadSchemaVersion) {
  EnvelopeCodec codec;
  auto envelope = makeEnvelope(6);
  envelope.schema_version = 2;

  const auto validation = validateSampleEnvelope(
      envelope, SampleEnvelopeValidationOptions{kDefaultMaxPayloadBytes, false});
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(std::string::npos, validation.error.find("schema_version"));
  EXPECT_THROW(codec.encode(envelope), std::invalid_argument);

  auto bytes = codec.encode(makeEnvelope(6));
  writeU32At(&bytes, 8, 2);
  const auto decoded = codec.decode(bytes);
  EXPECT_FALSE(decoded.ok);
  EXPECT_NE(std::string::npos, decoded.error.find("schema_version"));
}

TEST(EnvelopeCodecTest, RejectsMissingRequiredFields) {
  EnvelopeCodec codec;
  auto envelope = makeEnvelope(6);
  envelope.participant_id.clear();
  envelope.payload_crc32c = EnvelopeCodec::crc32c(envelope.payload.data(), envelope.payload.size());

  const auto validation = validateSampleEnvelope(envelope);
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(std::string::npos, validation.error.find("participant_id"));
  EXPECT_THROW(codec.encode(envelope), std::invalid_argument);
}

TEST(EnvelopeCodecTest, RejectsIllegalKeyFieldCharacters) {
  EnvelopeCodec codec;
  auto envelope = makeEnvelope(6);
  envelope.task_id = "/planner/local_solution";
  envelope.payload_crc32c = EnvelopeCodec::crc32c(envelope.payload.data(), envelope.payload.size());

  const auto validation = validateSampleEnvelope(envelope);
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(std::string::npos, validation.error.find("task_id"));
  EXPECT_THROW(codec.encode(envelope), std::invalid_argument);
}

TEST(Ros1AdapterConfigTest, AcceptsValidControlledRule) {
  XmlRpc::XmlRpcValue root;
  root["deny_by_default"] = true;
  root["rules"] = XmlRpc::XmlRpcValue();
  root["rules"].setSize(1);
  root["rules"][0]["name"] = "local_solution";
  root["rules"][0]["topic"] = "/local_solution";
  root["rules"][0]["channel"] = "solution";
  root["rules"][0]["schema_id"] = "user.solution.v1";
  root["rules"][0]["payload_type"] = "bytes";
  root["rules"][0]["enabled"] = true;
  root["rules"][0]["max_payload_bytes"] = 4096;
  root["rules"][0]["required"] = true;

  const auto parsed = swarm_sync_ros1::parseAdapterConfigXmlRpc(root);

  ASSERT_TRUE(parsed.ok) << parsed.error;
  EXPECT_TRUE(parsed.config.deny_by_default);
  ASSERT_EQ(1u, parsed.config.rules.size());
  EXPECT_EQ(1u, parsed.config.enabledRuleCount());
  EXPECT_EQ("/local_solution", parsed.config.rules.front().topic);
  EXPECT_EQ("solution", parsed.config.rules.front().channel);
  EXPECT_EQ("user.solution.v1", parsed.config.rules.front().schema_id);
  EXPECT_TRUE(parsed.config.rules.front().required);
}

TEST(Ros1AdapterConfigTest, RejectsInvalidTopic) {
  swarm_sync_ros1::AdapterConfig config;
  swarm_sync_ros1::AdapterRule rule;
  rule.name = "bad_topic";
  rule.topic = "local_solution";
  rule.channel = "solution";
  rule.schema_id = "user.solution.v1";
  config.rules.push_back(rule);

  const auto result = swarm_sync_ros1::validateAdapterConfig(config);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(std::string::npos, result.error.find("topic"));
}

TEST(Ros1AdapterConfigTest, RejectsDefaultDeniedTopicsWithoutExplicitOverride) {
  EXPECT_TRUE(swarm_sync_ros1::isDefaultDeniedTopic("/tf"));
  EXPECT_TRUE(swarm_sync_ros1::isDefaultDeniedTopic("/camera/image_raw"));

  swarm_sync_ros1::AdapterConfig config;
  swarm_sync_ros1::AdapterRule rule;
  rule.name = "camera_image";
  rule.topic = "/camera/image_raw";
  rule.channel = "camera_image";
  rule.schema_id = "user.camera_image.v1";
  config.rules.push_back(rule);

  const auto result = swarm_sync_ros1::validateAdapterConfig(config);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(std::string::npos, result.error.find("default-denied"));
}

TEST(Ros1AdapterConfigTest, AllowsLargePayloadWithExplicitOverrideWithinEnvelopeLimit) {
  swarm_sync_ros1::AdapterConfig config;
  swarm_sync_ros1::AdapterRule rule;
  rule.name = "camera_image";
  rule.topic = "/camera/image_raw";
  rule.channel = "camera_image";
  rule.schema_id = "user.camera_image.v1";
  rule.allow_large_payload = true;
  rule.max_payload_bytes = kDefaultMaxPayloadBytes;
  config.rules.push_back(rule);

  const auto result = swarm_sync_ros1::validateAdapterConfig(config);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(1u, result.config.rules.size());
  EXPECT_EQ(kDefaultMaxPayloadBytes, result.config.rules.front().max_payload_bytes);
}

TEST(Ros1AdapterConfigTest, RejectsDuplicateRuleName) {
  swarm_sync_ros1::AdapterConfig config;
  swarm_sync_ros1::AdapterRule first;
  first.name = "solution";
  first.topic = "/local_solution";
  first.channel = "solution";
  first.schema_id = "user.solution.v1";
  config.rules.push_back(first);

  swarm_sync_ros1::AdapterRule second = first;
  second.topic = "/other_solution";
  second.channel = "other_solution";
  second.schema_id = "user.other_solution.v1";
  config.rules.push_back(second);

  const auto result = swarm_sync_ros1::validateAdapterConfig(config);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(std::string::npos, result.error.find("duplicate rule name"));
}

TEST(Ros1AdapterConfigTest, RejectsEnabledChannelSchemaConflict) {
  swarm_sync_ros1::AdapterConfig config;
  swarm_sync_ros1::AdapterRule first;
  first.name = "solution_a";
  first.topic = "/solution_a";
  first.channel = "solution";
  first.schema_id = "user.solution.v1";
  config.rules.push_back(first);

  swarm_sync_ros1::AdapterRule second = first;
  second.name = "solution_b";
  second.topic = "/solution_b";
  config.rules.push_back(second);

  const auto result = swarm_sync_ros1::validateAdapterConfig(config);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(std::string::npos, result.error.find("channel/schema"));
}

TEST(EnvelopeCodecTest, DetectsMalformedEncodedData) {
  EnvelopeCodec codec;
  const auto bytes = codec.encode(makeEnvelope(6));

  auto bad_magic = bytes;
  bad_magic[0] ^= 0xffu;
  EXPECT_FALSE(codec.decode(bad_magic).ok);
  EXPECT_NE(std::string::npos, codec.decode(bad_magic).error.find("magic"));

  auto bad_codec_version = bytes;
  writeU32At(&bad_codec_version, 4, 2);
  EXPECT_FALSE(codec.decode(bad_codec_version).ok);
  EXPECT_NE(std::string::npos, codec.decode(bad_codec_version).error.find("codec"));

  auto bad_field_length = bytes;
  writeU32At(&bad_field_length, 12, kMaxEnvelopeFieldBytes + 1);
  const auto bad_length_result = codec.decode(bad_field_length);
  EXPECT_FALSE(bad_length_result.ok);
  EXPECT_NE(std::string::npos, bad_length_result.error.find("session_id"));

  EnvelopeCodec small_payload_codec(2);
  const auto oversized_decode_result = small_payload_codec.decode(bytes);
  EXPECT_FALSE(oversized_decode_result.ok);
  EXPECT_NE(std::string::npos, oversized_decode_result.error.find("payload"));

  auto truncated = bytes;
  truncated.pop_back();
  const auto truncated_result = codec.decode(truncated);
  EXPECT_FALSE(truncated_result.ok);
  EXPECT_NE(std::string::npos, truncated_result.error.find("payload"));
}

TEST(TransportKeyTest, BuildsTaskSemanticKeyAndRejectsBadFields) {
  const auto envelope = makeEnvelope(3);
  const auto key = buildTaskSampleKey(envelope);
  ASSERT_TRUE(key.ok) << key.error;
  EXPECT_EQ("swarm_sync/v1/session/session-a/task/formation/channel/solution/schema/solution.v1",
            key.key);
  EXPECT_TRUE(validateTaskSampleKey(key.key));

  TransportKeyFields missing_schema;
  missing_schema.session_id = "session-a";
  missing_schema.task_id = "formation";
  missing_schema.channel = "solution";
  EXPECT_FALSE(buildTaskSampleKey(missing_schema).ok);

  TransportKeyFields ros_graph_like;
  ros_graph_like.session_id = "session-a";
  ros_graph_like.task_id = "/planner/local_solution";
  ros_graph_like.channel = "solution";
  ros_graph_like.schema_id = "solution.v1";
  EXPECT_FALSE(buildTaskSampleKey(ros_graph_like).ok);

  std::string reason;
  EXPECT_FALSE(validateTaskSampleKey("/local_solution", &reason));
  EXPECT_FALSE(reason.empty());
  EXPECT_FALSE(validateTaskSampleKey(key.key + "/extra"));
}

TEST(InMemoryTransportTest, DispatchesOnlyToMatchingKey) {
  const auto solution_key = buildTaskSampleKey(makeEnvelope(10));
  ASSERT_TRUE(solution_key.ok) << solution_key.error;

  auto status_envelope = makeEnvelope(10);
  status_envelope.channel = "status";
  status_envelope.schema_id = "status.v1";
  const auto status_key = buildTaskSampleKey(status_envelope);
  ASSERT_TRUE(status_key.ok) << status_key.error;

  InMemoryTransport transport;
  std::vector<TransportMessage> solution_messages;
  std::vector<TransportMessage> status_messages;

  const auto solution_subscription = transport.subscribe(
      solution_key.key,
      [&solution_messages](const TransportMessage& message) {
        solution_messages.push_back(message);
      });
  ASSERT_TRUE(solution_subscription);

  const auto status_subscription = transport.subscribe(
      status_key.key,
      [&status_messages](const TransportMessage& message) {
        status_messages.push_back(message);
      });
  ASSERT_TRUE(status_subscription);

  TransportMessage solution_message;
  solution_message.key = solution_key.key;
  solution_message.cycle_id = 10;
  solution_message.participant_id = "uav2";
  solution_message.payload = makeEnvelope(10);

  ASSERT_TRUE(transport.publish(solution_message));
  ASSERT_EQ(1u, solution_messages.size());
  EXPECT_EQ(solution_key.key, solution_messages.front().key);
  EXPECT_EQ(10u, solution_messages.front().cycle_id);
  EXPECT_EQ("uav2", solution_messages.front().participant_id);
  EXPECT_EQ(solution_message.payload.payload, solution_messages.front().payload.payload);
  EXPECT_TRUE(status_messages.empty());

  TransportMessage status_message;
  status_message.key = status_key.key;
  status_message.cycle_id = 10;
  status_message.participant_id = "uav3";
  status_message.payload = status_envelope;

  ASSERT_TRUE(transport.publish(status_message));
  ASSERT_EQ(1u, solution_messages.size());
  ASSERT_EQ(1u, status_messages.size());
  EXPECT_EQ(status_key.key, status_messages.front().key);
}

TEST(InMemoryTransportTest, UnsubscribeStopsDelivery) {
  const auto key = buildTaskSampleKey(makeEnvelope(11));
  ASSERT_TRUE(key.ok) << key.error;

  InMemoryTransport transport;
  uint32_t received_count = 0;
  auto subscription = transport.subscribe(key.key, [&received_count](const TransportMessage&) {
    ++received_count;
  });
  ASSERT_TRUE(subscription);
  EXPECT_TRUE(subscription->active());

  TransportMessage message;
  message.key = key.key;
  message.cycle_id = 11;
  message.participant_id = "uav2";
  message.payload = makeEnvelope(11);

  EXPECT_TRUE(transport.publish(message));
  EXPECT_EQ(1u, received_count);

  transport.unsubscribe(subscription);
  EXPECT_FALSE(subscription->active());
  EXPECT_TRUE(transport.publish(message));
  EXPECT_EQ(1u, received_count);
}

TEST(DeadlineCheckerTest, ClassifiesFreshLateWrongCycleAndBadSchema) {
  DeadlineChecker checker;
  ChannelTiming timing;
  timing.publish_deadline_ns = 10000000;
  timing.receive_cutoff_ns = 20000000;
  timing.ttl_ns = 40000000;
  timing.payload_type = "bytes";
  timing.schema_id = "solution.v1";

  const auto envelope = makeEnvelope(7);
  auto decision = checker.checkReceivedSample(envelope, 7, 1000000000, 1000005000, timing);
  EXPECT_EQ(SampleStatus::Fresh, decision.status);
  EXPECT_TRUE(decision.usable);

  decision = checker.checkReceivedSample(envelope, 8, 1000000000, 1000005000, timing);
  EXPECT_EQ(SampleStatus::WrongCycle, decision.status);

  decision = checker.checkReceivedSample(envelope, 7, 1000000000, 1030000000, timing);
  EXPECT_EQ(SampleStatus::Late, decision.status);

  auto bad_schema = envelope;
  bad_schema.schema_id = "solution.v2";
  decision = checker.checkReceivedSample(bad_schema, 7, 1000000000, 1000005000, timing);
  EXPECT_EQ(SampleStatus::BadSchema, decision.status);
}

TEST(SampleBufferTest, StoresSamplesDetectsDuplicateAndSeqGap) {
  SampleBuffer buffer;
  const auto first = makeEnvelope(1, 1);
  auto result = buffer.add(first, 100);
  EXPECT_TRUE(result.inserted);
  EXPECT_EQ(SampleStatus::Fresh, result.status);
  ASSERT_TRUE(buffer.find("uav2", "solution", 1).has_value());
  ASSERT_TRUE(buffer.getLastGood("uav2", "solution").has_value());

  result = buffer.add(first, 110);
  EXPECT_FALSE(result.inserted);
  EXPECT_TRUE(result.duplicate);
  EXPECT_EQ(SampleStatus::Duplicate, result.status);

  const auto fourth = makeEnvelope(2, 4);
  result = buffer.add(fourth, 200);
  EXPECT_TRUE(result.inserted);
  EXPECT_EQ(2u, result.seq_gap);
  EXPECT_EQ(2u, buffer.seqGapCount("uav2", "solution"));
  EXPECT_EQ(2u, buffer.getLastGood("uav2", "solution")->envelope.target_cycle);
}

TEST(SnapshotBuilderTest, BuildsFreshMissingAndBadStats) {
  SampleBuffer buffer;
  ASSERT_TRUE(buffer.add(makeEnvelope(10, 1), 1000010000).inserted);

  auto bad_clock = makeEnvelope(10, 1);
  bad_clock.sender_id = "uav3";
  bad_clock.clock_ok = false;
  ASSERT_TRUE(buffer.add(bad_clock, 1000010000).inserted);

  ChannelTiming timing;
  timing.payload_type = "bytes";
  timing.schema_id = "solution.v1";
  timing.receive_cutoff_ns = 20000000;
  timing.ttl_ns = 40000000;

  SnapshotBuildRequest request;
  request.session_id = "session-a";
  request.task_id = "formation";
  request.self_id = "uav1";
  request.cycle_id = 10;
  request.cycle_start_ns = 1000000000;
  request.period_ns = 50000000;
  request.build_time_ns = 1000010000;
  request.local_clock = ClockState{true, 0, 0, ClockQuality::OK, "test"};
  request.local_runtime.running = true;
  request.expected_samples = {
      ExpectedSample{"uav2", "solution", true, timing},
      ExpectedSample{"uav3", "solution", true, timing},
      ExpectedSample{"uav4", "solution", false, timing},
  };

  SnapshotBuilder builder;
  const auto snapshot = builder.build(request, buffer);
  ASSERT_EQ(3u, snapshot.samples.size());
  EXPECT_EQ(SampleStatus::Fresh, snapshot.samples[0].status);
  EXPECT_EQ(SampleStatus::SenderClockBad, snapshot.samples[1].status);
  EXPECT_EQ(SampleStatus::Missing, snapshot.samples[2].status);
  EXPECT_EQ(1u, snapshot.stats.fresh_count);
  EXPECT_EQ(1u, snapshot.stats.missing_count);
  EXPECT_EQ(1u, snapshot.stats.bad_count);
  EXPECT_FALSE(snapshot.all_required_fresh);
  EXPECT_FALSE(snapshot.all_required_usable);
}

TEST(WeaknetChannelProfileTest, EnforcesRealtimeNoAckAndDropQueue) {
  auto profile = swarm_sync::weaknet::defaultProfileFor(
      swarm_sync::weaknet::ChannelKind::RealtimeSample);
  std::string error;
  EXPECT_TRUE(swarm_sync::weaknet::validateChannelProfile(profile, &error)) << error;
  EXPECT_FALSE(profile.allow_ack);
  EXPECT_EQ(swarm_sync::weaknet::CongestionControl::Drop, profile.qos.congestion);
  EXPECT_LE(profile.max_queue_cycles, 1u);

  profile.allow_ack = true;
  EXPECT_FALSE(swarm_sync::weaknet::validateChannelProfile(profile, &error));
  EXPECT_NE(std::string::npos, error.find("ACK"));
}

TEST(WeaknetCycleWindowTest, NeverAcceptsLateOrWrongCycleAsFresh) {
  swarm_sync::weaknet::CycleWindow window({
      50000000,
      40000000,
      50000000,
      100000000,
  });
  std::string error;
  ASSERT_TRUE(window.valid(&error)) << error;

  swarm_sync::weaknet::SampleTiming fresh;
  fresh.target_cycle = 10;
  fresh.local_receive_ns = 1000000000 + 20000000;
  auto decision = window.classify(10, 1000000000, fresh);
  EXPECT_EQ(SampleStatus::Fresh, decision.status);
  EXPECT_TRUE(decision.usable_for_cycle);

  auto late = fresh;
  late.local_receive_ns = 1000000000 + 45000000;
  decision = window.classify(10, 1000000000, late);
  EXPECT_EQ(SampleStatus::Late, decision.status);
  EXPECT_FALSE(decision.usable_for_cycle);
  EXPECT_TRUE(decision.record_in_late_pocket);

  auto wrong = fresh;
  wrong.target_cycle = 9;
  decision = window.classify(10, 1000000000, wrong);
  EXPECT_EQ(SampleStatus::WrongCycle, decision.status);
  EXPECT_FALSE(decision.usable_for_cycle);
}

TEST(WeaknetPayloadAndBudgetTest, RejectsOversizedRealtimeAndComputesAirBudget) {
  swarm_sync::weaknet::PayloadGuardConfig payload_config;
  payload_config.warn_bytes = 800;
  payload_config.reject_bytes = 1200;

  auto decision = swarm_sync::weaknet::checkPayload(payload_config, 900, "bytes", "sample.v1");
  EXPECT_EQ(swarm_sync::weaknet::PayloadGuardAction::Warn, decision.action);
  decision = swarm_sync::weaknet::checkPayload(payload_config, 1300, "bytes", "sample.v1");
  EXPECT_EQ(swarm_sync::weaknet::PayloadGuardAction::Reject, decision.action);
  decision = swarm_sync::weaknet::checkPayload(payload_config, 16, "", "sample.v1");
  EXPECT_EQ(swarm_sync::weaknet::PayloadGuardAction::Reject, decision.action);

  swarm_sync::weaknet::ChannelBudgetInput input;
  input.channel = "solution";
  input.publishers = 3;
  input.effective_fanout = 2;
  input.frequency_hz = 20.0;
  input.payload_bytes_p95 = 600;
  const auto report = swarm_sync::weaknet::computeTrafficBudget({input}, 2000000.0, 1.35);
  EXPECT_TRUE(report.accepted);
  EXPECT_GT(report.total_realtime_bps, 0.0);

  const auto rejected = swarm_sync::weaknet::computeTrafficBudget({input}, 1000.0, 1.35);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_GT(rejected.utilization_ratio, 1.0);
}

TEST(WeaknetLinkHealthTest, SummarizesRatiosAndDrivesStateMachine) {
  swarm_sync::weaknet::LinkHealthWindow window(10);
  for (int i = 0; i < 8; ++i) {
    window.observe({SampleStatus::Fresh, 1000000, 0, false, false});
  }
  window.observe({SampleStatus::Late, 60000000, 0, true, false});
  window.observe({SampleStatus::Missing, 0, 0, false, false});

  auto health = window.summarize("uav2", "solution");
  EXPECT_EQ("uav2", health.peer_id);
  EXPECT_DOUBLE_EQ(0.8, health.fresh_ratio);
  EXPECT_GT(health.late_ratio, 0.0);
  EXPECT_GE(static_cast<uint8_t>(health.state),
            static_cast<uint8_t>(swarm_sync::weaknet::LinkState::Degraded));

  swarm_sync::weaknet::WeaknetStateMachine machine;
  const auto transition = machine.update(health);
  EXPECT_TRUE(transition.changed);
  EXPECT_EQ(health.state, transition.current);
  EXPECT_FALSE(transition.reason.empty());
}

TEST(WeaknetRecommendationAndStaggerTest, SuggestionsOnlyAndStableMicroSlots) {
  swarm_sync::weaknet::PeerChannelHealth health;
  health.peer_id = "uav2";
  health.channel = "solution";
  health.fresh_ratio = 0.8;
  health.late_ratio = 0.1;
  health.missing_ratio = 0.1;
  health.state = swarm_sync::weaknet::LinkState::Congested;
  health.primary_reason = "test congestion";
  swarm_sync::weaknet::RecommendationEngine engine;
  const auto recommendations = engine.evaluate(42, health, nullptr);
  ASSERT_GE(recommendations.size(), 2u);
  EXPECT_EQ(swarm_sync::weaknet::RecommendationType::ChangeFrequency,
            recommendations.front().type);
  EXPECT_EQ("change_frequency", swarm_sync::weaknet::toString(recommendations.front().type));

  swarm_sync::weaknet::DeterministicSendStagger stagger({true, 5000000, 500000});
  const auto a = stagger.decide("s1", "solution", "uav2", 42);
  const auto b = stagger.decide("s1", "solution", "uav2", 42);
  EXPECT_TRUE(a.enabled);
  EXPECT_EQ(a.slot_index, b.slot_index);
  EXPECT_EQ(a.offset_ns, b.offset_ns);
  EXPECT_LE(a.offset_ns, 5000000);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
