#include "swarm_sync_core/runtime_control.hpp"
#include <boost/json.hpp>
#include <future>
#include <stdexcept>

namespace swarm_sync {
namespace {
namespace j = boost::json;
using namespace xgc2::xrpc;
RuntimePolicy product_policy(RuntimePolicyOptions options) {
  options.defaults["HOST_MAX_CONNECTIONS"] = "16";
  options.defaults["HOST_MAX_IN_FLIGHT"] = "16";
  options.defaults["MAX_REQUEST_BYTES"] = "4096";
  options.defaults["MAX_RESPONSE_BYTES"] = "16384";
  options.defaults["CALL_TIMEOUT_MS"] = "5000";
  options.defaults["IDLE_TIMEOUT_MS"] = "5000";
  options.default_source = "runtime-sync";
  options.ceilings["HOST_MAX_CONNECTIONS"] = 16;
  options.ceilings["HOST_MAX_IN_FLIGHT"] = 16;
  options.ceilings["MAX_REQUEST_BYTES"] = 4096;
  options.ceilings["MAX_RESPONSE_BYTES"] = 16384;
  return resolve_runtime_policy(options);
}
HttpResponse policy_response(const RuntimePolicy& policy) {
  j::array fields;
  for (const auto& f : policy.fields()) {
    j::object field{{"name", std::string(f.name)}, {"source", std::string(f.source)},
      {"source_detail", f.source_detail}, {"dynamic", f.dynamic}, {"unit", std::string(f.unit)}};
    std::visit([&](const auto& value) { field["value"] = value; }, f.value);
    field["ceiling"] = f.ceiling ? j::value(*f.ceiling) : j::value(nullptr);
    fields.push_back(std::move(field));
  }
  return {200, {{"Content-Type", "application/json"}}, j::serialize(j::object{{"revision", policy.revision()}, {"fields", std::move(fields)}})};
}

j::object object(const HttpRequest& req, std::initializer_list<const char*> allowed) {
  j::error_code ec;
  auto value = j::parse(req.body, ec);
  if (ec || !value.is_object()) throw std::invalid_argument("JSON object required");
  auto o = value.as_object();
  for (const auto& f : o) {
    bool known = false;
    for (const auto* name : allowed) if (f.key() == name) known = true;
    if (!known) throw std::invalid_argument("unknown field");
  }
  return o;
}
uint64_t number(const j::object& o, const char* name) {
  auto* v = o.if_contains(name);
  if (!v || !(v->is_uint64() || (v->is_int64() && v->as_int64() >= 0)))
    throw std::invalid_argument(std::string(name) + " must be a nonnegative integer");
  return v->is_uint64() ? v->as_uint64() : uint64_t(v->as_int64());
}
std::string string(const j::object& o, const char* name, bool required = true) {
  auto* v = o.if_contains(name);
  if (!v && !required) return {};
  if (!v || !v->is_string() || v->as_string().size() > 128 || (required && v->as_string().empty()))
    throw std::invalid_argument(std::string(name) + " must be a bounded string");
  return std::string(v->as_string());
}
}
RuntimeControl::RuntimeControl(std::string path, RuntimePolicyOptions options) : policy_(product_policy(std::move(options))), instance_id_(xgc2::xrpc::new_instance_id()) {
  std::promise<void> started; auto ready = started.get_future();
  io_ = std::thread([this, path = std::move(path), started = std::move(started)]() mutable {
    try {
      auto limits = http_limits(policy_);
      HttpServer server(UnixOptions{path}, [this](auto req, auto reply) { handle(std::move(req), reply); },
                        limits, HttpIdentity{instance_id_, {"/v1/runtime-sync/describe"}});
      server.set_wakeup_handler([this] { flush(); });
      server_.store(&server); started.set_value(); server.run(); server_.store(nullptr);
    } catch (...) { server_.store(nullptr); try { started.set_exception(std::current_exception()); } catch (...) {} }
  });
  try { ready.get(); } catch (...) { io_.join(); throw; }
}
RuntimeControl::~RuntimeControl() {
  if (auto* s = server_.load()) s->request_stop();
  io_.join();
}
xgc2::xrpc::HttpResponse RuntimeControl::response(const Receipt* r) const {
  const auto& s = snapshot_;
  j::object out{{"schema_version", 1}, {"instance_id", instance_id_}, {"revision", s.revision},
    {"desired_revision", s.desired_revision}, {"applied_revision", s.applied_revision},
    {"persisted_revision", nullptr}, {"state", s.state}, {"running", s.running},
    {"session_id", s.session_id}, {"task_id", s.task_id}, {"current_cycle", s.current_cycle},
    {"epoch_ns", s.epoch_ns}, {"period_ns", s.period_ns}, {"clock_ok", s.clock_ok},
    {"clock_offset_ns", s.clock_offset_ns}, {"clock_uncertainty_ns", s.clock_uncertainty_ns},
    {"clock_quality", s.clock_quality}, {"clock_source", s.clock_source},
    {"clock_phase", s.clock_phase}, {"reason", s.reason}};
  if (r) {
    std::string state = r->state;
    if (state == "accepted") {
      if (s.applied_revision == r->desired_revision && s.running) state = "succeeded";
      else if (s.desired_revision != r->desired_revision) state = "cancelled";
      else if (s.state == "ERROR") state = "failed";
    }
    out["receipt"] = j::object{{"operation_id", r->id}, {"state", state},
      {"desired_revision", r->desired_revision}, {"failure_stage", r->code.empty() ? "" : "apply"}};
    if (!r->code.empty()) out["error"] = j::object{{"code", r->code}, {"message", r->detail}};
  }
  return {r ? r->status : 200, {{"Content-Type", "application/json"}}, j::serialize(out)};
}
void RuntimeControl::handle(HttpRequest req, HttpReply reply) {
  try {
    if (req.method == "GET" && req.target == "/v1/runtime-sync/describe") {
      reply.complete({200, {{"Content-Type", "application/json"}},
        j::serialize(j::object{{"schema_version", 1}, {"instance_id", instance_id_},
          {"service", "runtime-sync"}, {"commands", 16}, {"receipts", 64},
          {"observers", 16}, {"persistence", false}})}); return;
    }
    if (req.method != "POST") { reply.complete(http_error(404, "not_found", "unknown operation")); return; }
    if (req.target == "/v1/runtime-sync/effective-policy") {
      object(req, {}); reply.complete(policy_response(policy_)); return;
    }
    if (req.target == "/v1/runtime-sync/status") {
      object(req, {}); std::lock_guard lock(mutex_); reply.complete(response()); return;
    }
    if (req.target == "/v1/runtime-sync/receipt") {
      auto o = object(req, {"operation_id"}); auto id = string(o, "operation_id");
      std::lock_guard lock(mutex_);
      for (const auto& r : receipts_) if (r.id == id) { reply.complete(response(&r)); return; }
      reply.complete(http_error(404, "not_found", "receipt expired or unknown")); return;
    }
    if (req.target == "/v1/runtime-sync/observe") {
      auto o = object(req, {"after_revision"}); auto after = number(o, "after_revision");
      std::lock_guard lock(mutex_);
      if (after > snapshot_.revision) { reply.complete(http_error(409, "conflict", "future observation revision")); return; }
      if (after < snapshot_.revision) { reply.complete(response()); return; }
      for (auto& w : observers_) if (!w.active || w.reply.cancelled()) {
        w = {true, after, reply}; return;
      }
      reply.complete(http_error(429, "resource_exhausted", "observers full")); return;
    }
    RuntimeCommand c;
    if (req.target == "/v1/runtime-sync/start") {
      auto o = object(req, {"expected_revision", "session_id", "task_id", "epoch_ns", "period_ns"});
      c.kind = RuntimeCommand::Kind::Start; c.expected_revision = number(o, "expected_revision");
      c.session_id = string(o, "session_id"); c.task_id = string(o, "task_id");
      auto epoch = number(o, "epoch_ns"), period = number(o, "period_ns");
      if (epoch > uint64_t(INT64_MAX) || period < 20000000 || period > 2000000000)
        throw std::invalid_argument("epoch_ns or period_ns out of range");
      c.epoch_ns = int64_t(epoch); c.period_ns = int64_t(period);
    } else if (req.target == "/v1/runtime-sync/stop") {
      auto o = object(req, {"expected_revision", "session_id", "reason"});
      c.kind = RuntimeCommand::Kind::Stop; c.expected_revision = number(o, "expected_revision");
      c.session_id = string(o, "session_id"); c.reason = string(o, "reason", false);
    } else { reply.complete(http_error(404, "not_found", "unknown operation")); return; }
    std::lock_guard lock(mutex_);
    for (const auto& r : receipts_) if (r.id == req.request_id) {
      reply.complete(r.fingerprint == req.target + req.body ? response(&r) : http_error(409, "conflict", "request ID reused")); return;
    }
    for (const auto& s : slots_) if (s.state != Slot::Free && s.id == req.request_id) {
      reply.complete(http_error(409, "conflict", "request ID already in flight")); return;
    }
    for (auto& s : slots_) if (s.state == Slot::Free) {
      s.command = std::move(c); s.reply = reply; s.id = req.request_id;
      s.fingerprint = req.target + req.body; s.state = Slot::Queued; return;
    }
    reply.complete(http_error(429, "resource_exhausted", "native command slots full"));
  } catch (const std::invalid_argument& e) { reply.complete(http_error(400, "invalid_argument", e.what())); }
}
void RuntimeControl::process(const std::function<RuntimeOutcome(const RuntimeCommand&)>& apply) {
  for (auto& slot : slots_) {
    RuntimeCommand command;
    {
      std::unique_lock lock(mutex_, std::try_to_lock);
      if (!lock || slot.state != Slot::Queued) continue;
      if (slot.reply.cancelled()) { slot = {}; continue; }
      slot.state = Slot::Executing; command = slot.command;
    }
    auto outcome = apply(command);
    { std::lock_guard lock(mutex_); slot.outcome = std::move(outcome); slot.state = Slot::Ready; }
    if (auto* s = server_.load()) s->wake();
  }
}
void RuntimeControl::publish(RuntimeSnapshot snapshot) {
  { std::lock_guard lock(mutex_); snapshot_ = std::move(snapshot); }
  if (auto* s = server_.load()) s->wake();
}
void RuntimeControl::flush() {
  std::lock_guard lock(mutex_);
  auto promote = [this] {
    for (auto& r : receipts_) if (r.state == "accepted") {
      if (snapshot_.applied_revision == r.desired_revision && snapshot_.running) r.state = "succeeded";
      else if (snapshot_.desired_revision != r.desired_revision) r.state = "cancelled";
      else if (snapshot_.state == "ERROR") r.state = "failed";
    }
  };
  promote();
  for (auto& s : slots_) if (s.state == Slot::Ready) {
    auto& r = receipts_[next_receipt_++ % receipts_.size()];
    r = {s.id, s.fingerprint, s.outcome.operation_state, s.outcome.snapshot.desired_revision,
         s.outcome.status, s.outcome.code, s.outcome.detail};
    // No old snapshot can overwrite a newer owner publication.
    if (s.outcome.snapshot.revision >= snapshot_.revision) snapshot_ = s.outcome.snapshot;
    promote(); s.reply.complete(response(&r)); s = {};
  }
  for (auto& w : observers_) if (w.active) {
    if (w.reply.cancelled()) w = {};
    else if (snapshot_.revision > w.after) { w.reply.complete(response()); w = {}; }
  }
}
}
