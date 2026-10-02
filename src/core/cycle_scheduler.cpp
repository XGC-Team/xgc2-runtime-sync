#include "swarm_sync_core/cycle_scheduler.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace swarm_sync {

bool CycleScheduler::isValidFrequencyHz(double frequency_hz) {
  return std::isfinite(frequency_hz) &&
         frequency_hz >= kMinFrequencyHz &&
         frequency_hz <= kMaxFrequencyHz;
}

bool CycleScheduler::isValidPeriodNs(int64_t period_ns) {
  return period_ns >= kMinPeriodNs && period_ns <= kMaxPeriodNs;
}

std::optional<int64_t> CycleScheduler::periodNsFromFrequencyHz(double frequency_hz) {
  if (!isValidFrequencyHz(frequency_hz)) {
    return std::nullopt;
  }
  return static_cast<int64_t>(std::llround(1.0e9 / frequency_hz));
}

bool CycleScheduler::configure(const CycleSchedulerConfig& config) {
  if (!isValidPeriodNs(config.period_ns)) {
    return false;
  }
  if (config.session_id.empty() || config.task_id.empty()) {
    return false;
  }
  if (config.max_cycle > 0 && config.max_cycle < config.start_cycle) {
    return false;
  }

  config_ = config;
  running_ = false;
  has_emitted_ = false;
  last_emitted_cycle_ = 0;
  return true;
}

bool CycleScheduler::configure(std::string session_id,
                               std::string task_id,
                               int64_t epoch_ns,
                               int64_t period_ns) {
  CycleSchedulerConfig config;
  config.session_id = std::move(session_id);
  config.task_id = std::move(task_id);
  config.epoch_ns = epoch_ns;
  config.period_ns = period_ns;
  return configure(config);
}

void CycleScheduler::setCallback(Callback cb) {
  callback_ = std::move(cb);
}

void CycleScheduler::start() {
  if (config_.period_ns > 0) {
    running_ = true;
  }
}

void CycleScheduler::stop() {
  running_ = false;
}

uint64_t CycleScheduler::computeCycleId(int64_t now_ns) const {
  if (now_ns < config_.epoch_ns || config_.period_ns <= 0) {
    return config_.start_cycle;
  }

  const auto elapsed = static_cast<uint64_t>(now_ns - config_.epoch_ns);
  return config_.start_cycle + elapsed / static_cast<uint64_t>(config_.period_ns);
}

int64_t CycleScheduler::expectedTimeNs(uint64_t cycle_id) const {
  if (cycle_id <= config_.start_cycle) {
    return config_.epoch_ns;
  }
  const uint64_t offset_cycles = cycle_id - config_.start_cycle;
  const uint64_t period = static_cast<uint64_t>(config_.period_ns);
  if (offset_cycles > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) / period) {
    return std::numeric_limits<int64_t>::max();
  }
  return config_.epoch_ns + static_cast<int64_t>(offset_cycles * period);
}

std::optional<CycleEvent> CycleScheduler::tick(int64_t now_ns, bool clock_ok) {
  if (!running_ || config_.period_ns <= 0 || now_ns < config_.epoch_ns) {
    return std::nullopt;
  }

  const uint64_t cycle_id = computeCycleId(now_ns);
  if (config_.max_cycle > 0 && cycle_id > config_.max_cycle) {
    running_ = false;
    return std::nullopt;
  }
  if (has_emitted_ && cycle_id <= last_emitted_cycle_) {
    return std::nullopt;
  }

  CycleEvent event;
  event.session_id = config_.session_id;
  event.task_id = config_.task_id;
  event.cycle_id = cycle_id;
  event.expected_time_ns = expectedTimeNs(cycle_id);
  event.actual_time_ns = now_ns;
  event.jitter_ns = now_ns - event.expected_time_ns;
  event.period_ns = config_.period_ns;
  event.clock_ok = clock_ok;

  has_emitted_ = true;
  last_emitted_cycle_ = cycle_id;
  if (callback_) {
    callback_(event);
  }
  return event;
}

bool CycleScheduler::due(int64_t now_ns) const {
  if (!running_ || config_.period_ns <= 0 || now_ns < config_.epoch_ns) {
    return false;
  }
  const uint64_t cycle_id = computeCycleId(now_ns);
  if (config_.max_cycle > 0 && cycle_id > config_.max_cycle) {
    return false;
  }
  return !has_emitted_ || cycle_id > last_emitted_cycle_;
}

bool CycleScheduler::running() const {
  return running_;
}

const CycleSchedulerConfig& CycleScheduler::config() const {
  return config_;
}

}  // namespace swarm_sync
