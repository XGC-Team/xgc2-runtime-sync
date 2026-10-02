#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include "periodic_sync/BudgetReport.h"
#include "periodic_sync/GetRuntimeStatus.h"
#include "periodic_sync/CycleSnapshot.h"
#include "periodic_sync/PeerLinkHealth.h"
#include "periodic_sync/PeerSampleStatus.h"
#include "periodic_sync/Recommendation.h"
#include "periodic_sync/RuntimeHealth.h"
#include "periodic_sync/SampleStats.h"
#include "periodic_sync/StartSession.h"
#include "periodic_sync/StopSession.h"
#include "periodic_sync/SyncedCycle.h"
#include "periodic_sync/WeakNetState.h"
#include "swarm_sync_core/clock_monitor.hpp"
#include "swarm_sync_core/sample_buffer.hpp"
#include "swarm_sync_core/snapshot_builder.hpp"
#include "swarm_sync_core/cycle_scheduler.hpp"
#include "swarm_sync_core/session.hpp"
#include "swarm_sync_core/weaknet/weaknet.hpp"
#include "swarm_sync_ros1/adapter_config.hpp"

namespace {

int64_t toNs(const ros::Time& time) {
  return static_cast<int64_t>(time.sec) * 1000000000LL + static_cast<int64_t>(time.nsec);
}

int64_t toNs(const ros::Duration& duration) {
  return static_cast<int64_t>(duration.sec) * 1000000000LL + static_cast<int64_t>(duration.nsec);
}

ros::Duration fromNs(int64_t ns) {
  return ros::Duration(static_cast<double>(ns) / 1.0e9);
}

ros::Time timeFromNs(int64_t ns) {
  if (ns <= 0) {
    return ros::Time(0);
  }
  return ros::Time().fromNSec(static_cast<uint64_t>(ns));
}

uint8_t toMsgClockQuality(swarm_sync::ClockQuality quality) {
  return static_cast<uint8_t>(quality);
}

std::string stateName(swarm_sync::SessionState state) {
  switch (state) {
    case swarm_sync::SessionState::IDLE:
      return "IDLE";
    case swarm_sync::SessionState::CONFIGURED:
      return "CONFIGURED";
    case swarm_sync::SessionState::ARMED:
      return "ARMED";
    case swarm_sync::SessionState::RUNNING:
      return "RUNNING";
    case swarm_sync::SessionState::STOPPED:
      return "STOPPED";
    case swarm_sync::SessionState::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

std::string linkKey(const std::string& peer, const std::string& channel) {
  return peer + "|" + channel;
}

std::string jsonEscape(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

std::string evidenceJson(const std::map<std::string, double>& evidence) {
  std::ostringstream os;
  os << "{";
  bool first = true;
  for (const auto& item : evidence) {
    if (!first) {
      os << ",";
    }
    first = false;
    os << "\"" << jsonEscape(item.first) << "\":" << item.second;
  }
  os << "}";
  return os.str();
}

}  // namespace

class SwarmRuntimeNode {
 public:
  bool init() {
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("team_id", config_.team_id, "alpha");
    pnh.param<std::string>("session_id", config_.session_id, "demo_session_001");
    pnh.param<std::string>("task_id", config_.task_id, "distributed_task");
    pnh.param<std::string>("self_id", config_.self_id, "uav01");
    pnh.param<double>("frequency_hz", frequency_hz_, 20.0);
    pnh.param<bool>("auto_start", auto_start_, true);
    pnh.param<double>("start_delay_s", start_delay_s_, 0.1);
    pnh.param<double>("poll_rate_hz", poll_rate_hz_, 200.0);
    pnh.param<std::string>("expected_sample_channel", expected_sample_channel_, "runtime_state");
    pnh.param<bool>("weaknet_enabled", weaknet_enabled_, true);
    pnh.param<double>("weaknet_budget_max_realtime_bps", weaknet_budget_max_realtime_bps_, 2000000.0);
    pnh.param<double>("weaknet_budget_safety_factor", weaknet_budget_safety_factor_, 1.35);
    pnh.param<int>("weaknet_payload_bytes_p95", weaknet_payload_bytes_p95_, 600);
    pnh.param<double>("weaknet_receive_cutoff_ratio", weaknet_receive_cutoff_ratio_, 0.80);
    pnh.param<double>("weaknet_late_record_window_ratio", weaknet_late_record_window_ratio_, 1.0);
    loadClockPolicy(pnh);

    if (!loadAdapterConfig(pnh)) {
      return false;
    }

    std::vector<std::string> required_participants;
    if (pnh.getParam("required_participants", required_participants) &&
        !required_participants.empty()) {
      config_.required_participants = required_participants;
    } else {
      config_.required_participants = {config_.self_id};
    }
    const auto period_ns = swarm_sync::CycleScheduler::periodNsFromFrequencyHz(frequency_hz_);
    if (!period_ns) {
      ROS_ERROR_STREAM("frequency_hz must be in ["
                       << swarm_sync::CycleScheduler::kMinFrequencyHz << ", "
                       << swarm_sync::CycleScheduler::kMaxFrequencyHz << "], got "
                       << frequency_hz_);
      return false;
    }
    config_.period_ns = *period_ns;
    config_.epoch_ns = toNs(ros::Time::now()) + static_cast<int64_t>(start_delay_s_ * 1.0e9);
    if (!configureWeaknet()) {
      return false;
    }
    configureExpectedSamples();

    if (!session_.configure(config_)) {
      ROS_ERROR_STREAM("failed to configure swarm runtime session: " << session_.reason());
      return false;
    }

    cycle_pub_ = nh_.advertise<periodic_sync::SyncedCycle>("/swarm_sync/cycle", 10, false);
    snapshot_pub_ = nh_.advertise<periodic_sync::CycleSnapshot>("cycle_snapshot", 10, false);
    sample_stats_pub_ = nh_.advertise<periodic_sync::SampleStats>("sample_stats", 10, false);
    health_pub_ = nh_.advertise<periodic_sync::RuntimeHealth>("/swarm_sync/runtime_health", 2, true);
    peer_link_health_pub_ = nh_.advertise<periodic_sync::PeerLinkHealth>("peer_link_health", 10, false);
    weaknet_state_pub_ = nh_.advertise<periodic_sync::WeakNetState>("weaknet_state", 10, false);
    recommendation_pub_ = nh_.advertise<periodic_sync::Recommendation>("recommendation", 10, false);
    budget_pub_ = nh_.advertise<periodic_sync::BudgetReport>("budget_report", 1, true);
    start_srv_ = nh_.advertiseService("/swarm_sync/start_session", &SwarmRuntimeNode::startCallback, this);
    stop_srv_ = nh_.advertiseService("/swarm_sync/stop_session", &SwarmRuntimeNode::stopCallback, this);
    status_srv_ = nh_.advertiseService("/swarm_sync/get_runtime_status", &SwarmRuntimeNode::statusCallback, this);

    timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, poll_rate_hz_)),
        &SwarmRuntimeNode::timerCallback,
        this);

    ROS_INFO_STREAM("swarm_runtime_node ready: session=" << config_.session_id
                    << " self=" << config_.self_id
                    << " frequency_hz=" << frequency_hz_
                    << " " << adapter_summary_
                    << " weaknet=" << weaknet_summary_
                    << " clock_provider=" << clock_policy_.provider
                    << " clock_phase=" << swarm_sync::ClockMonitor::phaseToString(clock_phase_));
    if (auto_start_) {
      last_clock_result_ = sampleClock();
      if (!startSession(config_.epoch_ns, config_.period_ns, last_clock_result_.state)) {
        ROS_ERROR_STREAM("auto-start blocked by clock gate: " << session_.reason());
      }
    }
    publishBudget();
    publishHealth();
    return true;
  }

 private:
  bool startSession(int64_t epoch_ns, int64_t period_ns, const swarm_sync::ClockState& clock) {
    if (!swarm_sync::CycleScheduler::isValidPeriodNs(period_ns)) {
      session_.markError("period_ns outside supported 0.5-50 Hz range");
      return false;
    }
    if (!clock.clock_ok || clock.quality == swarm_sync::ClockQuality::BAD) {
      session_.markError("clock gate rejected: " + last_clock_result_.reason);
      return false;
    }
    config_.epoch_ns = epoch_ns;
    config_.period_ns = period_ns;
    if (!session_.configure(config_)) {
      return false;
    }
    configureExpectedSamples();
    if (!session_.arm(epoch_ns, clock)) {
      return false;
    }

    swarm_sync::CycleSchedulerConfig scheduler_config;
    scheduler_config.session_id = config_.session_id;
    scheduler_config.task_id = config_.task_id;
    scheduler_config.epoch_ns = epoch_ns;
    scheduler_config.period_ns = period_ns;
    scheduler_config.start_cycle = config_.start_cycle;
    scheduler_config.max_cycle = config_.max_cycle;
    if (!scheduler_.configure(scheduler_config)) {
      session_.markError("failed to configure cycle scheduler");
      return false;
    }
    scheduler_.start();
    return true;
  }

  bool startCallback(periodic_sync::StartSession::Request& request,
                     periodic_sync::StartSession::Response& response) {
    auto next_config = config_;
    if (!request.session_id.empty()) {
      next_config.session_id = request.session_id;
    }
    if (!request.task_id.empty()) {
      next_config.task_id = request.task_id;
    }
    int64_t requested_period_ns = next_config.period_ns;
    if (request.period.toSec() > 0.0) {
      requested_period_ns = toNs(request.period);
    }
    if (!swarm_sync::CycleScheduler::isValidPeriodNs(requested_period_ns)) {
      response.accepted = false;
      response.reason = "period_ns outside supported 0.5-50 Hz range";
      return true;
    }
    next_config.period_ns = requested_period_ns;
    next_config.epoch_ns = toNs(request.epoch_time);
    config_ = next_config;

    last_clock_result_ = sampleClock();
    response.accepted = startSession(config_.epoch_ns, config_.period_ns, last_clock_result_.state);
    response.reason = response.accepted ? "accepted" : session_.reason();
    return true;
  }

  bool stopCallback(periodic_sync::StopSession::Request& request,
                    periodic_sync::StopSession::Response& response) {
    scheduler_.stop();
    session_.stop(request.reason.empty() ? "stopped by service" : request.reason);
    response.accepted = true;
    response.message = "stopped";
    publishHealth();
    return true;
  }

  bool statusCallback(periodic_sync::GetRuntimeStatus::Request&,
                      periodic_sync::GetRuntimeStatus::Response& response) {
    response.running = session_.running();
    response.state = stateName(session_.state());
    response.current_cycle = current_cycle_;
    last_clock_result_ = sampleClock();
    response.clock_ok = last_clock_result_.state.clock_ok;
    response.clock_offset_ns = last_clock_result_.state.offset_ns;
    response.clock_uncertainty_ns = last_clock_result_.state.uncertainty_ns;
    response.clock_quality = toMsgClockQuality(last_clock_result_.state.quality);
    response.clock_source = last_clock_result_.state.source;
    response.clock_phase = swarm_sync::ClockMonitor::phaseToString(clock_phase_);
    response.zenoh_connected = false;
    response.reason = runtimeReason();
    return true;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    // Only the start gate of an armed session and an emitted cycle read the
    // clock state. Sample it for those ticks only: for the chrony provider a
    // sample runs chronyc twice, and polling at poll_rate_hz (200 Hz) spawned
    // 400 processes per second whose results nothing read.
    if (session_.state() == swarm_sync::SessionState::ARMED || scheduler_.due(toNs(now))) {
      last_clock_result_ = sampleClock();
    }
    const auto& clock = last_clock_result_.state;

    session_.startIfDue(toNs(now), clock);
    const auto event = scheduler_.tick(toNs(now), clock.clock_ok);
    if (event) {
      current_cycle_ = event->cycle_id;
      last_jitter_ns_ = event->jitter_ns;
      publishCycle(*event, now);
      publishSnapshot(*event, now, clock);
      publishHealth(false);
    }
  }

  void publishCycle(const swarm_sync::CycleEvent& event, const ros::Time& stamp) {
    periodic_sync::SyncedCycle msg;
    msg.header.stamp = stamp;
    msg.team_id = config_.team_id;
    msg.session_id = config_.session_id;
    msg.task_id = config_.task_id;
    msg.self_id = config_.self_id;
    msg.cycle_id = event.cycle_id;
    msg.expected_time = ros::Time().fromNSec(static_cast<uint64_t>(event.expected_time_ns));
    msg.actual_time = ros::Time().fromNSec(static_cast<uint64_t>(event.actual_time_ns));
    msg.period = fromNs(event.period_ns);
    msg.jitter = fromNs(event.jitter_ns);
    msg.clock_ok = event.clock_ok;
    msg.clock_quality = toMsgClockQuality(last_clock_result_.state.quality);
    msg.session_running = session_.running();
    msg.runtime_degraded = !last_clock_result_.state.clock_ok;
    cycle_pub_.publish(msg);
  }

  void publishSnapshot(const swarm_sync::CycleEvent& event,
                       const ros::Time& stamp,
                       const swarm_sync::ClockState& clock) {
    swarm_sync::SnapshotBuildRequest request;
    request.session_id = config_.session_id;
    request.task_id = config_.task_id;
    request.self_id = config_.self_id;
    request.cycle_id = event.cycle_id;
    request.cycle_start_ns = event.expected_time_ns;
    request.period_ns = event.period_ns;
    request.build_time_ns = toNs(stamp);
    request.local_clock = clock;
    request.local_runtime.running = session_.running();
    request.local_runtime.zenoh_connected = false;
    request.local_runtime.current_cycle = event.cycle_id;
    request.local_runtime.jitter_ns = event.jitter_ns;
    request.local_runtime.state = stateName(session_.state());
    request.local_runtime.reason = session_.reason();
    request.expected_samples = expected_samples_;

    const auto snapshot = snapshot_builder_.build(request, sample_buffer_);
    const auto snapshot_msg = toSnapshotMsg(snapshot, stamp);
    snapshot_pub_.publish(snapshot_msg);
    sample_stats_pub_.publish(toSampleStatsMsg(snapshot, stamp));
    publishWeaknet(snapshot, stamp);
  }

  periodic_sync::CycleSnapshot toSnapshotMsg(const swarm_sync::CycleSnapshot& snapshot,
                                             const ros::Time& stamp) const {
    periodic_sync::CycleSnapshot msg;
    msg.header.stamp = stamp;
    msg.team_id = config_.team_id;
    msg.session_id = snapshot.session_id;
    msg.task_id = snapshot.task_id;
    msg.self_id = snapshot.self_id;
    msg.cycle_id = snapshot.cycle_id;
    msg.cycle_start_time = timeFromNs(snapshot.t_cycle_start_ns);
    msg.period = fromNs(snapshot.period_ns);
    msg.samples.reserve(snapshot.samples.size());
    for (const auto& sample : snapshot.samples) {
      msg.samples.push_back(toPeerSampleStatusMsg(sample));
    }
    msg.fresh_count = snapshot.stats.fresh_count;
    msg.missing_count = snapshot.stats.missing_count;
    msg.late_count = snapshot.stats.late_count;
    msg.wrong_cycle_count = snapshot.stats.wrong_cycle_count;
    msg.bad_count = snapshot.stats.bad_count;
    msg.all_required_fresh = snapshot.all_required_fresh;
    msg.all_required_usable = snapshot.all_required_usable;
    msg.local_clock_ok = snapshot.local_clock.clock_ok;
    msg.runtime_degraded = !snapshot.local_runtime.running || !snapshot.local_clock.clock_ok;
    return msg;
  }

  periodic_sync::PeerSampleStatus toPeerSampleStatusMsg(
      const swarm_sync::PeerSampleView& sample) const {
    periodic_sync::PeerSampleStatus msg;
    msg.peer_id = sample.peer_id;
    msg.channel = sample.channel;
    msg.status = static_cast<uint8_t>(sample.status);
    msg.expected_target_cycle = sample.expected_target_cycle;
    msg.received_target_cycle = sample.received_target_cycle;
    if (sample.sample) {
      msg.seq = sample.sample->seq;
      msg.schema_id = sample.sample->schema_id;
    }
    msg.local_receive_time = timeFromNs(sample.t_local_receive_ns);
    msg.receive_latency = fromNs(sample.receive_latency_ns);
    msg.fresh_for_this_cycle = sample.fresh_for_this_cycle;
    msg.late_for_this_cycle = sample.late_for_this_cycle;
    msg.duplicate = sample.duplicate;
    msg.wrong_cycle = sample.wrong_cycle;
    msg.bad_schema = sample.bad_schema;
    msg.bad_crc = sample.bad_crc;
    msg.sender_clock_bad = sample.sender_clock_bad;
    msg.drop_reason = sample.drop_reason;
    return msg;
  }

  periodic_sync::SampleStats toSampleStatsMsg(const swarm_sync::CycleSnapshot& snapshot,
                                              const ros::Time& stamp) const {
    periodic_sync::SampleStats msg;
    msg.header.stamp = stamp;
    msg.team_id = config_.team_id;
    msg.session_id = snapshot.session_id;
    msg.task_id = snapshot.task_id;
    msg.self_id = snapshot.self_id;
    msg.cycle_id = snapshot.cycle_id;
    msg.received_count = snapshot.stats.fresh_count + snapshot.stats.late_count;
    msg.fresh_count = snapshot.stats.fresh_count;
    msg.missing_count = snapshot.stats.missing_count;
    msg.late_count = snapshot.stats.late_count;
    msg.wrong_cycle_count = snapshot.stats.wrong_cycle_count;
    for (const auto& sample : snapshot.samples) {
      if (sample.duplicate) {
        ++msg.duplicate_count;
      }
      if (sample.bad_schema) {
        ++msg.bad_schema_count;
      }
      if (sample.bad_crc) {
        ++msg.bad_payload_count;
      }
      if (sample.sender_clock_bad) {
        ++msg.sender_clock_bad_count;
      }
    }
    return msg;
  }

  void configureExpectedSamples() {
    expected_samples_.clear();
    swarm_sync::ChannelTiming timing;
    timing.payload_type = "bytes";
    timing.schema_id = expected_sample_channel_ + ".v1";
    timing.receive_cutoff_ns = config_.period_ns > 0 ? config_.period_ns : 50000000;
    timing.ttl_ns = timing.receive_cutoff_ns;
    for (const auto& participant : config_.required_participants) {
      swarm_sync::ExpectedSample expected;
      expected.peer_id = participant;
      expected.channel = expected_sample_channel_;
      expected.required = true;
      expected.timing = timing;
      expected_samples_.push_back(std::move(expected));
    }
  }

  void publishHealth(bool refresh_clock = true) {
    periodic_sync::RuntimeHealth msg;
    msg.header.stamp = ros::Time::now();
    msg.team_id = config_.team_id;
    msg.session_id = config_.session_id;
    msg.self_id = config_.self_id;
    msg.alive = true;
    msg.session_running = session_.running();
    msg.zenoh_connected = false;
    if (refresh_clock) {
      last_clock_result_ = sampleClock();
    }
    msg.clock_ok = last_clock_result_.state.clock_ok;
    msg.clock_offset_ns = last_clock_result_.state.offset_ns;
    msg.clock_uncertainty_ns = last_clock_result_.state.uncertainty_ns;
    msg.clock_quality = toMsgClockQuality(last_clock_result_.state.quality);
    msg.current_cycle = current_cycle_;
    msg.cycle_jitter_p95 = fromNs(last_jitter_ns_);
    msg.cycle_jitter_p99 = fromNs(last_jitter_ns_);
    msg.state = stateName(session_.state());
    msg.reason = runtimeReason();
    health_pub_.publish(msg);
  }

  bool configureWeaknet() {
    weaknet_profile_ = swarm_sync::weaknet::defaultProfileFor(
        swarm_sync::weaknet::ChannelKind::RealtimeSample);
    std::string error;
    if (!swarm_sync::weaknet::validateChannelProfile(weaknet_profile_, &error)) {
      ROS_ERROR_STREAM("invalid weaknet realtime profile: " << error);
      return false;
    }
    weaknet_budget_ = computeBudgetReport();
    if (!weaknet_budget_.accepted) {
      ROS_WARN_STREAM("weaknet traffic budget exceeded: ratio="
                      << weaknet_budget_.utilization_ratio
                      << " reason=" << weaknet_budget_.reason);
    }
    weaknet_summary_ = "profile=" + weaknet_profile_.name +
                       " qos=" + swarm_sync::weaknet::toString(weaknet_profile_.qos.reliability) +
                       "/" + swarm_sync::weaknet::toString(weaknet_profile_.qos.congestion) +
                       " budget_ratio=" + std::to_string(weaknet_budget_.utilization_ratio);
    return true;
  }

  swarm_sync::weaknet::BudgetReport computeBudgetReport() const {
    const uint32_t participants =
        static_cast<uint32_t>(std::max<size_t>(1, config_.required_participants.size()));
    swarm_sync::weaknet::ChannelBudgetInput input;
    input.channel = expected_sample_channel_;
    input.publishers = participants;
    input.effective_fanout = participants > 0 ? participants - 1 : 0;
    input.frequency_hz = frequency_hz_;
    input.payload_bytes_p95 = static_cast<uint32_t>(std::max(0, weaknet_payload_bytes_p95_));
    return swarm_sync::weaknet::computeTrafficBudget({input},
                                                     weaknet_budget_max_realtime_bps_,
                                                     weaknet_budget_safety_factor_);
  }

  void publishBudget() {
    periodic_sync::BudgetReport msg;
    msg.header.stamp = ros::Time::now();
    msg.session_id = config_.session_id;
    msg.accepted = weaknet_budget_.accepted;
    msg.total_realtime_bps = weaknet_budget_.total_realtime_bps;
    msg.max_realtime_bps = weaknet_budget_.max_realtime_bps;
    msg.utilization_ratio = weaknet_budget_.utilization_ratio;
    msg.report_json = "{\"reason\":\"" + jsonEscape(weaknet_budget_.reason) + "\"}";
    budget_pub_.publish(msg);
  }

  void publishWeaknet(const swarm_sync::CycleSnapshot& snapshot, const ros::Time& stamp) {
    if (!weaknet_enabled_) {
      return;
    }
    swarm_sync::weaknet::LinkState worst_state = swarm_sync::weaknet::LinkState::Good;
    std::string worst_reason = "healthy";

    for (const auto& sample : snapshot.samples) {
      const auto key = linkKey(sample.peer_id, sample.channel);
      auto& window = link_windows_[key];
      swarm_sync::weaknet::LinkHealthSample health_sample;
      health_sample.status = sample.status;
      health_sample.latency_ns = sample.receive_latency_ns;
      health_sample.late_pocket = sample.late_for_this_cycle;
      window.observe(health_sample);

      auto health = window.summarize(sample.peer_id, sample.channel);
      auto& state_machine = link_states_[key];
      const auto transition = state_machine.update(health);
      health.state = state_machine.current();
      if (static_cast<uint8_t>(health.state) > static_cast<uint8_t>(worst_state)) {
        worst_state = health.state;
        worst_reason = health.primary_reason;
      }
      peer_link_health_pub_.publish(toPeerLinkHealthMsg(health, stamp));
      if (transition.changed) {
        weaknet_state_pub_.publish(toWeakNetStateMsg(transition, snapshot.cycle_id, stamp));
      }
      for (const auto& recommendation :
           recommendation_engine_.evaluate(snapshot.cycle_id, health, &weaknet_budget_)) {
        recommendation_pub_.publish(toRecommendationMsg(recommendation, stamp));
      }
    }

    weaknet_state_ = worst_state;
    weaknet_reason_ = worst_reason;
  }

  periodic_sync::PeerLinkHealth toPeerLinkHealthMsg(
      const swarm_sync::weaknet::PeerChannelHealth& health,
      const ros::Time& stamp) const {
    periodic_sync::PeerLinkHealth msg;
    msg.header.stamp = stamp;
    msg.local_node_id = config_.self_id;
    msg.peer_id = health.peer_id;
    msg.channel = health.channel;
    msg.fresh_ratio = health.fresh_ratio;
    msg.late_ratio = health.late_ratio;
    msg.missing_ratio = health.missing_ratio;
    msg.duplicate_ratio = health.duplicate_ratio;
    msg.wrong_cycle_ratio = health.wrong_cycle_ratio;
    msg.bad_payload_ratio = health.bad_payload_ratio;
    msg.rx_latency_p50_ns = health.rx_latency_p50_ns;
    msg.rx_latency_p95_ns = health.rx_latency_p95_ns;
    msg.rx_latency_p99_ns = health.rx_latency_p99_ns;
    msg.seq_gap_count = health.seq_gap_count;
    msg.late_pocket_count = health.late_pocket_count;
    msg.oversized_drop_count = health.oversized_drop_count;
    msg.state = static_cast<uint8_t>(health.state);
    msg.primary_reason = health.primary_reason;
    return msg;
  }

  periodic_sync::WeakNetState toWeakNetStateMsg(
      const swarm_sync::weaknet::StateTransition& transition,
      uint64_t cycle_id,
      const ros::Time& stamp) const {
    periodic_sync::WeakNetState msg;
    msg.header.stamp = stamp;
    msg.node_id = config_.self_id;
    msg.previous_state = static_cast<uint8_t>(transition.previous);
    msg.current_state = static_cast<uint8_t>(transition.current);
    msg.reason = transition.reason;
    msg.evidence_json = evidenceJson(transition.evidence);
    msg.cycle_id = cycle_id;
    return msg;
  }

  periodic_sync::Recommendation toRecommendationMsg(
      const swarm_sync::weaknet::RuntimeRecommendation& recommendation,
      const ros::Time& stamp) const {
    periodic_sync::Recommendation msg;
    msg.header.stamp = stamp;
    msg.node_id = config_.self_id;
    msg.cycle_id = recommendation.cycle_id;
    msg.level = static_cast<uint8_t>(recommendation.level);
    msg.type = swarm_sync::weaknet::toString(recommendation.type);
    msg.channel = recommendation.channel;
    msg.peer_id = recommendation.peer;
    msg.confidence = recommendation.confidence;
    msg.reason = recommendation.reason;
    msg.evidence_json = evidenceJson(recommendation.evidence);
    return msg;
  }

  bool loadAdapterConfig(const ros::NodeHandle& pnh) {
    std::string config_path;
    pnh.param<std::string>("ros1_adapters_config", config_path, "");

    XmlRpc::XmlRpcValue raw_config;
    if (pnh.getParam("ros1_adapters", raw_config)) {
      const auto parsed = swarm_sync_ros1::parseAdapterConfigXmlRpc(raw_config);
      if (!parsed.ok) {
        ROS_ERROR_STREAM("failed to load ros1_adapters config: " << parsed.error);
        return false;
      }
      adapter_config_ = parsed.config;
      adapter_summary_ = swarm_sync_ros1::summarizeAdapterConfig(adapter_config_);
      ROS_INFO_STREAM("loaded ros1_adapters config: " << adapter_summary_);
      return true;
    }

    if (!config_path.empty()) {
      ROS_ERROR_STREAM("ros1_adapters_config is set to " << config_path
                       << " but private parameter ros1_adapters was not loaded");
      return false;
    }

    adapter_config_ = swarm_sync_ros1::defaultAdapterConfig();
    adapter_summary_ = swarm_sync_ros1::summarizeAdapterConfig(adapter_config_) + " source=built_in_default";
    ROS_WARN_STREAM("no ros1_adapters config loaded; using " << adapter_summary_);
    return true;
  }

  std::string runtimeReason() const {
    const std::string clock_summary =
        "clock_source=" + last_clock_result_.state.source +
        " clock_phase=" + swarm_sync::ClockMonitor::phaseToString(clock_phase_) +
        " clock_reason=" + last_clock_result_.reason;
    if (session_.reason().empty()) {
      return adapter_summary_ + "; " + weaknet_summary_ + "; weaknet_state=" +
             swarm_sync::weaknet::toString(weaknet_state_) + " reason=" + weaknet_reason_ +
             "; " + clock_summary;
    }
    return session_.reason() + "; " + adapter_summary_ + "; " + weaknet_summary_ +
           "; weaknet_state=" + swarm_sync::weaknet::toString(weaknet_state_) +
           " reason=" + weaknet_reason_ + "; " + clock_summary;
  }

  void loadClockPolicy(const ros::NodeHandle& pnh) {
    pnh.param<std::string>("clock_provider", clock_policy_.provider, "chrony");
    pnh.param<std::string>("clock_authority", clock_policy_.authority, "ground_station");
    pnh.param<std::string>("ground_time_source", clock_policy_.ground_time_source, "");
    pnh.param<bool>("require_clock_ok_to_start", clock_policy_.require_clock_ok_to_start, true);
    pnh.param<bool>("in_flight_allow_step", clock_policy_.in_flight_allow_step, false);
    pnh.param<bool>("external_sources_allowed_in_flight",
                    clock_policy_.external_sources_allowed_in_flight,
                    false);
    pnh.param<std::string>("clock_phase_source", clock_policy_.phase_source, "external");
    pnh.param<std::string>("clock_phase", clock_phase_param_, "preflight");
    double max_offset_ms = 2.0;
    double max_uncertainty_ms = 2.0;
    pnh.param<double>("max_clock_offset_ms", max_offset_ms, 2.0);
    pnh.param<double>("preflight_max_offset_ms", max_offset_ms, max_offset_ms);
    pnh.param<double>("preflight_max_uncertainty_ms", max_uncertainty_ms, 2.0);
    clock_policy_.preflight_max_offset_ns =
        static_cast<int64_t>(std::max(0.0, max_offset_ms) * 1000000.0);
    clock_policy_.preflight_max_uncertainty_ns =
        static_cast<uint64_t>(std::max(0.0, max_uncertainty_ms) * 1000000.0);

    pnh.param<bool>("mock_clock_ok", mock_clock_ok_, true);
    pnh.param<int>("mock_clock_quality", mock_clock_quality_, 0);
    pnh.param<int>("mock_clock_offset_ns", mock_clock_offset_ns_, 0);
    pnh.param<int>("mock_clock_uncertainty_ns", mock_clock_uncertainty_ns_, 0);
    pnh.param<std::string>("mock_clock_source", mock_clock_source_, "mock_ground_station");
    clock_phase_ = swarm_sync::ClockMonitor::phaseFromString(clock_phase_param_);
  }

  // clock_phase is read once in loadClockPolicy; XGC2 nodes never poll the
  // parameter server after startup.
  swarm_sync::ClockMonitorResult sampleClock() {
    if (clock_policy_.provider == "mock") {
      swarm_sync::ClockMonitorResult result;
      result.phase = clock_phase_;
      result.source_is_ground = true;
      result.state.clock_ok = mock_clock_ok_;
      result.state.offset_ns = mock_clock_offset_ns_;
      result.state.uncertainty_ns = static_cast<uint64_t>(std::max(0, mock_clock_uncertainty_ns_));
      result.state.quality = static_cast<swarm_sync::ClockQuality>(mock_clock_quality_);
      result.state.source = mock_clock_source_;
      result.start_allowed = result.state.clock_ok || !clock_policy_.require_clock_ok_to_start;
      result.reason = result.state.clock_ok ? "mock clock synchronized to ground station"
                                            : "mock clock is not synchronized";
      return result;
    }
    return clock_monitor_.sample(clock_policy_, clock_phase_);
  }

  ros::NodeHandle nh_;
  swarm_sync::SessionConfig config_;
  swarm_sync_ros1::AdapterConfig adapter_config_;
  std::string adapter_summary_;
  swarm_sync::SessionManager session_;
  swarm_sync::CycleScheduler scheduler_;
  swarm_sync::SnapshotBuilder snapshot_builder_;
  swarm_sync::SampleBuffer sample_buffer_;
  swarm_sync::weaknet::ChannelProfile weaknet_profile_;
  swarm_sync::weaknet::BudgetReport weaknet_budget_;
  swarm_sync::weaknet::RecommendationEngine recommendation_engine_;
  swarm_sync::ClockPolicy clock_policy_;
  swarm_sync::ClockMonitor clock_monitor_;
  swarm_sync::ClockMonitorResult last_clock_result_;
  std::map<std::string, swarm_sync::weaknet::LinkHealthWindow> link_windows_;
  std::map<std::string, swarm_sync::weaknet::WeaknetStateMachine> link_states_;
  swarm_sync::weaknet::LinkState weaknet_state_{swarm_sync::weaknet::LinkState::Good};
  std::vector<swarm_sync::ExpectedSample> expected_samples_;
  ros::Publisher cycle_pub_;
  ros::Publisher snapshot_pub_;
  ros::Publisher sample_stats_pub_;
  ros::Publisher health_pub_;
  ros::Publisher peer_link_health_pub_;
  ros::Publisher weaknet_state_pub_;
  ros::Publisher recommendation_pub_;
  ros::Publisher budget_pub_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer status_srv_;
  ros::Timer timer_;
  uint64_t current_cycle_ = 0;
  int64_t last_jitter_ns_ = 0;
  double frequency_hz_ = 20.0;
  double poll_rate_hz_ = 200.0;
  double start_delay_s_ = 0.1;
  double weaknet_budget_max_realtime_bps_ = 2000000.0;
  double weaknet_budget_safety_factor_ = 1.35;
  double weaknet_receive_cutoff_ratio_ = 0.80;
  double weaknet_late_record_window_ratio_ = 1.0;
  std::string expected_sample_channel_ = "runtime_state";
  std::string weaknet_summary_ = "weaknet=unconfigured";
  std::string weaknet_reason_ = "healthy";
  std::string clock_phase_param_ = "preflight";
  std::string mock_clock_source_ = "mock_ground_station";
  int weaknet_payload_bytes_p95_ = 600;
  int mock_clock_offset_ns_ = 0;
  int mock_clock_uncertainty_ns_ = 0;
  int mock_clock_quality_ = 0;
  bool auto_start_ = true;
  bool weaknet_enabled_ = true;
  bool mock_clock_ok_ = true;
  swarm_sync::ClockPhase clock_phase_{swarm_sync::ClockPhase::Preflight};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_runtime_node");
  SwarmRuntimeNode node;
  if (!node.init()) {
    return 1;
  }
  ros::spin();
  return 0;
}
