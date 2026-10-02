#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace swarm_sync {

struct CycleEvent {
  std::string session_id;
  std::string task_id;
  uint64_t cycle_id{0};
  int64_t expected_time_ns{0};
  int64_t actual_time_ns{0};
  int64_t jitter_ns{0};
  int64_t period_ns{0};
  bool clock_ok{false};
};

struct CycleSchedulerConfig {
  std::string session_id;
  std::string task_id;
  int64_t epoch_ns{0};
  int64_t period_ns{0};
  uint64_t start_cycle{0};
  uint64_t max_cycle{0};
};

class CycleScheduler {
public:
  using Callback = std::function<void(const CycleEvent&)>;

  static constexpr double kMinFrequencyHz = 0.5;
  static constexpr double kMaxFrequencyHz = 50.0;
  static constexpr int64_t kMinPeriodNs = 20000000;
  static constexpr int64_t kMaxPeriodNs = 2000000000;

  static bool isValidFrequencyHz(double frequency_hz);
  static bool isValidPeriodNs(int64_t period_ns);
  static std::optional<int64_t> periodNsFromFrequencyHz(double frequency_hz);

  bool configure(const CycleSchedulerConfig& config);
  bool configure(std::string session_id,
                 std::string task_id,
                 int64_t epoch_ns,
                 int64_t period_ns);

  void setCallback(Callback cb);
  void start();
  void stop();

  uint64_t computeCycleId(int64_t now_ns) const;
  int64_t expectedTimeNs(uint64_t cycle_id) const;
  std::optional<CycleEvent> tick(int64_t now_ns, bool clock_ok = true);
  // True exactly when tick(now_ns) would emit a cycle; has no side effects.
  bool due(int64_t now_ns) const;

  bool running() const;
  const CycleSchedulerConfig& config() const;

private:
  CycleSchedulerConfig config_;
  Callback callback_;
  bool running_{false};
  bool has_emitted_{false};
  uint64_t last_emitted_cycle_{0};
};

}  // namespace swarm_sync
