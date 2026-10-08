#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <xgc2/xrpc/http.hpp>
#include <xgc2/xrpc/runtime_policy.hpp>

namespace swarm_sync {
struct RuntimeCommand {
  enum class Kind { Start, Stop } kind;
  uint64_t expected_revision = 0;
  int64_t epoch_ns = 0, period_ns = 0;
  std::string session_id, task_id, reason;
};
struct RuntimeSnapshot {
  uint64_t revision = 0, desired_revision = 0, applied_revision = 0, current_cycle = 0;
  int64_t epoch_ns = 0, period_ns = 0, clock_offset_ns = 0;
  uint64_t clock_uncertainty_ns = 0;
  bool running = false, clock_ok = false;
  uint8_t clock_quality = 0;
  std::string state, session_id, task_id, clock_source, clock_phase, reason;
};
struct RuntimeOutcome {
  int status = 200;
  std::string code, detail, operation_state = "succeeded";
  RuntimeSnapshot snapshot;
};
// One HTTP owner and 16 admitted native commands, independent of team size.
// The ROS/native owner calls process() and publish(); JSON stays on the IO owner.
class RuntimeControl {
public:
  explicit RuntimeControl(std::string socket_path, xgc2::xrpc::RuntimePolicyOptions options = {});
  ~RuntimeControl();
  RuntimeControl(const RuntimeControl&) = delete;
  RuntimeControl& operator=(const RuntimeControl&) = delete;
  void process(const std::function<RuntimeOutcome(const RuntimeCommand&)>& apply);
  void publish(RuntimeSnapshot snapshot);
  const std::string& instance_id() const { return instance_id_; }
private:
  struct Slot {
    enum State { Free, Queued, Executing, Ready } state = Free;
    RuntimeCommand command;
    xgc2::xrpc::HttpReply reply;
    RuntimeOutcome outcome;
    std::string id, fingerprint;
  };
  struct Receipt {
    std::string id, fingerprint, state;
    uint64_t desired_revision = 0;
    int status = 200;
    std::string code, detail;
  };
  struct Observer { bool active = false; uint64_t after = 0; xgc2::xrpc::HttpReply reply; };
  void handle(xgc2::xrpc::HttpRequest, xgc2::xrpc::HttpReply);
  void flush();
  xgc2::xrpc::HttpResponse response(const Receipt* receipt = nullptr) const;
  std::array<Slot, 16> slots_{};
  std::array<Observer, 16> observers_{};
  std::array<Receipt, 64> receipts_{};
  size_t next_receipt_ = 0;
  RuntimeSnapshot snapshot_;
  std::mutex mutex_;
  std::thread io_;
  std::atomic<xgc2::xrpc::HttpServer*> server_{nullptr};
  const xgc2::xrpc::RuntimePolicy policy_;
  const std::string instance_id_;
};
}
