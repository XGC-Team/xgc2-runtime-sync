#include <gtest/gtest.h>

#include <ros/ros.h>

#include "periodic_sync/BudgetReport.h"
#include "periodic_sync/CycleSnapshot.h"
#include <boost/json.hpp>
#include <xgc2/xrpc/http.hpp>
#include "periodic_sync/PeerLinkHealth.h"
#include "periodic_sync/PeerSampleStatus.h"
#include "periodic_sync/Recommendation.h"
#include "periodic_sync/SyncedCycle.h"
#include "periodic_sync/WeakNetState.h"

namespace {

periodic_sync::SyncedCycle last_cycle;
periodic_sync::CycleSnapshot last_snapshot;
periodic_sync::BudgetReport last_budget;
periodic_sync::PeerLinkHealth last_link_health;
periodic_sync::Recommendation last_recommendation;
bool received_cycle = false;
bool received_snapshot = false;
bool received_budget = false;
bool received_link_health = false;
bool received_recommendation = false;

void cycleCallback(const periodic_sync::SyncedCycle::ConstPtr& msg) {
  last_cycle = *msg;
  received_cycle = true;
}

void snapshotCallback(const periodic_sync::CycleSnapshot::ConstPtr& msg) {
  last_snapshot = *msg;
  received_snapshot = true;
}

void budgetCallback(const periodic_sync::BudgetReport::ConstPtr& msg) {
  last_budget = *msg;
  received_budget = true;
}

void linkHealthCallback(const periodic_sync::PeerLinkHealth::ConstPtr& msg) {
  last_link_health = *msg;
  received_link_health = true;
}

void recommendationCallback(const periodic_sync::Recommendation::ConstPtr& msg) {
  last_recommendation = *msg;
  received_recommendation = true;
}

}  // namespace

TEST(RuntimeNodeSmokeTest, PublishesCycleAndServesStatus) {
  ros::NodeHandle nh;
  const auto cycle_sub = nh.subscribe("/swarm_sync/cycle", 4, cycleCallback);
  const auto snapshot_sub = nh.subscribe("/cycle_snapshot", 4, snapshotCallback);
  const auto budget_sub = nh.subscribe("/budget_report", 4, budgetCallback);
  const auto link_health_sub = nh.subscribe("/peer_link_health", 4, linkHealthCallback);
  const auto recommendation_sub = nh.subscribe("/recommendation", 4, recommendationCallback);

  const ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
  ros::Rate rate(100.0);
  while (ros::ok() &&
         !(received_cycle && received_snapshot && received_budget && received_link_health &&
           last_cycle.cycle_id == last_snapshot.cycle_id) &&
         ros::Time::now() < deadline) {
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_TRUE(received_cycle);
  ASSERT_TRUE(received_snapshot);
  ASSERT_TRUE(received_budget);
  ASSERT_TRUE(received_link_health);
  EXPECT_EQ(last_cycle.cycle_id, last_snapshot.cycle_id);
  EXPECT_EQ("test_team", last_cycle.team_id);
  EXPECT_EQ("smoke_session", last_cycle.session_id);
  EXPECT_EQ("smoke_task", last_cycle.task_id);
  EXPECT_EQ("uav_test", last_cycle.self_id);
  EXPECT_TRUE(last_cycle.clock_ok);
  EXPECT_TRUE(last_cycle.session_running);

  EXPECT_EQ("test_team", last_snapshot.team_id);
  EXPECT_EQ("smoke_session", last_snapshot.session_id);
  EXPECT_EQ("smoke_task", last_snapshot.task_id);
  EXPECT_EQ("uav_test", last_snapshot.self_id);
  EXPECT_TRUE(last_snapshot.local_clock_ok);
  ASSERT_EQ(1u, last_snapshot.samples.size());
  EXPECT_EQ("uav_test", last_snapshot.samples.front().peer_id);
  EXPECT_EQ("runtime_state", last_snapshot.samples.front().channel);
  EXPECT_EQ(periodic_sync::PeerSampleStatus::STATUS_MISSING, last_snapshot.samples.front().status);
  EXPECT_EQ(last_snapshot.cycle_id, last_snapshot.samples.front().expected_target_cycle);
  EXPECT_EQ(1u, last_snapshot.missing_count);
  EXPECT_FALSE(last_snapshot.all_required_fresh);
  EXPECT_FALSE(last_snapshot.all_required_usable);

  EXPECT_TRUE(last_budget.accepted);
  EXPECT_GT(last_budget.max_realtime_bps, 0.0);
  EXPECT_EQ("uav_test", last_link_health.peer_id);
  EXPECT_EQ("runtime_state", last_link_health.channel);
  EXPECT_EQ(0.0, last_link_health.fresh_ratio);
  EXPECT_GT(last_link_health.missing_ratio, 0.0);
  EXPECT_NE(periodic_sync::PeerSampleStatus::STATUS_FRESH, last_snapshot.samples.front().status);
  EXPECT_TRUE(received_recommendation);
  EXPECT_FALSE(last_recommendation.type.empty());
  EXPECT_FALSE(last_recommendation.reason.empty());

  namespace j = boost::json;
  ros::NodeHandle pnh("~"); std::string socket;
  ASSERT_TRUE(pnh.getParam("control_socket", socket));
  xgc2::xrpc::HttpClient discovery(socket);
  xgc2::xrpc::HttpRequest describe; describe.method = "GET";
  describe.target = "/v1/runtime-sync/describe";
  auto described = discovery.call(describe, xgc2::xrpc::Clock::now() + std::chrono::seconds(2));
  ASSERT_EQ(200, described.status);
  auto instance = std::string(j::parse(described.body).as_object().at("instance_id").as_string());
  xgc2::xrpc::HttpClient client(socket, {}, instance);
  uint64_t request_seq = 0;
  auto call = [&](const std::string& method, j::object payload, std::string id = "") {
    xgc2::xrpc::HttpRequest req; req.method = "POST";
    req.target = "/v1/runtime-sync/" + method; req.body = j::serialize(payload);
    req.request_id = id.empty() ? "smoke-" + std::to_string(++request_seq) : id;
    return client.call(req, xgc2::xrpc::Clock::now() + std::chrono::seconds(2));
  };
  auto policy_response = call("effective-policy", {}); ASSERT_EQ(200, policy_response.status);
  auto policy = j::parse(policy_response.body).as_object(); bool matched_policy = false;
  for (const auto& item : policy.at("fields").as_array()) {
    const auto& field = item.as_object();
    if (field.at("name").as_string() == "HOST_MAX_CONNECTIONS") {
      matched_policy = true; EXPECT_EQ(16, field.at("value").to_number<int>());
      EXPECT_EQ("deployment", field.at("source").as_string());
    }
  }
  EXPECT_TRUE(matched_policy);
  auto status_response = call("status", {}); ASSERT_EQ(200, status_response.status);
  auto status = j::parse(status_response.body).as_object();
  EXPECT_TRUE(status.at("running").as_bool());
  EXPECT_EQ("RUNNING", status.at("state").as_string());
  EXPECT_TRUE(status.at("clock_ok").as_bool());
  EXPECT_EQ(1000000, status.at("clock_offset_ns").as_int64());
  EXPECT_EQ(500000u, status.at("clock_uncertainty_ns").to_number<uint64_t>());
  EXPECT_EQ("mock_ground_station", status.at("clock_source").as_string());
  EXPECT_EQ("preflight", status.at("clock_phase").as_string());
  EXPECT_GE(status.at("current_cycle").to_number<uint64_t>(), last_cycle.cycle_id);
  auto desired = status.at("desired_revision").to_number<uint64_t>();
  EXPECT_EQ(desired, status.at("applied_revision").to_number<uint64_t>());
  // Every retired ROS control endpoint is absent in this isolated product.
  EXPECT_FALSE(ros::service::exists("/swarm_sync/start_session", false));
  EXPECT_FALSE(ros::service::exists("/swarm_sync/stop_session", false));
  EXPECT_FALSE(ros::service::exists("/swarm_sync/get_runtime_status", false));
  EXPECT_EQ(400, call("status", {{"typo", true}}).status);
  EXPECT_EQ(409, call("stop", {{"expected_revision", desired + 1},
                             {"session_id", "smoke_session"}}).status);
  EXPECT_TRUE(j::parse(call("status", {}).body).as_object().at("running").as_bool());
  auto stop_payload = j::object{{"expected_revision", desired}, {"session_id", "smoke_session"}};
  auto stopped = call("stop", stop_payload, "stop-once"); ASSERT_EQ(200, stopped.status);
  auto stop_state = j::parse(stopped.body).as_object();
  EXPECT_FALSE(stop_state.at("running").as_bool());
  EXPECT_EQ("STOPPED", stop_state.at("state").as_string());
  auto next = stop_state.at("desired_revision").to_number<uint64_t>();
  EXPECT_EQ(desired + 1, next);
  auto replay = call("stop", stop_payload, "stop-once"); EXPECT_EQ(200, replay.status);
  EXPECT_EQ(next, j::parse(replay.body).as_object().at("desired_revision").to_number<uint64_t>());
  EXPECT_EQ(409, call("stop", {{"expected_revision", next}, {"session_id", "smoke_session"}}, "stop-once").status);
  const auto epoch_ns = ros::Time::now().toNSec() + 300000000ULL;
  auto started = call("start", {{"expected_revision", next}, {"session_id", "next_session"},
    {"task_id", "next_task"}, {"epoch_ns", epoch_ns}, {"period_ns", 50000000}}, "start-once");
  ASSERT_EQ(200, started.status);
  auto armed = j::parse(started.body).as_object();
  EXPECT_EQ("ARMED", armed.at("state").as_string());
  EXPECT_EQ("accepted", armed.at("receipt").as_object().at("state").as_string());
  EXPECT_LT(armed.at("applied_revision").to_number<uint64_t>(), armed.at("desired_revision").to_number<uint64_t>());
  auto running = armed;
  const auto until = xgc2::xrpc::Clock::now() + std::chrono::seconds(2);
  while (!running.at("running").as_bool() && xgc2::xrpc::Clock::now() < until) {
    auto observed = call("observe", {{"after_revision", running.at("revision")}});
    ASSERT_EQ(200, observed.status); running = j::parse(observed.body).as_object();
  }
  EXPECT_TRUE(running.at("running").as_bool());
  auto receipt = j::parse(call("receipt", {{"operation_id", "start-once"}}).body).as_object();
  EXPECT_EQ("succeeded", receipt.at("receipt").as_object().at("state").as_string());
  auto final_stop = call("stop", {{"expected_revision", running.at("desired_revision")}, {"session_id", "next_session"}});
  ASSERT_EQ(200, final_stop.status);
  auto terminal = j::parse(call("receipt", {{"operation_id", "start-once"}}).body).as_object();
  EXPECT_EQ("succeeded", terminal.at("receipt").as_object().at("state").as_string());
  // A changed ROS parameter is not a second live configuration authority.
  ros::param::set("/swarm_runtime_node/clock_phase", std::string("in_flight"));
  EXPECT_EQ("preflight", j::parse(call("status", {}).body).as_object().at("clock_phase").as_string());

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "runtime_node_smoke_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
