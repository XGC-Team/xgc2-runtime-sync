#include <gtest/gtest.h>

#include <ros/ros.h>

#include "periodic_sync/BudgetReport.h"
#include "periodic_sync/CycleSnapshot.h"
#include "periodic_sync/GetRuntimeStatus.h"
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

  ros::ServiceClient client =
      nh.serviceClient<periodic_sync::GetRuntimeStatus>("/swarm_sync/get_runtime_status");
  ASSERT_TRUE(client.waitForExistence(ros::Duration(3.0)));

  periodic_sync::GetRuntimeStatus request;
  request.request.session_id = "smoke_session";
  ASSERT_TRUE(client.call(request));
  EXPECT_TRUE(request.response.running);
  EXPECT_EQ("RUNNING", request.response.state);
  EXPECT_TRUE(request.response.clock_ok);
  EXPECT_EQ(1000000, request.response.clock_offset_ns);
  EXPECT_EQ(500000u, request.response.clock_uncertainty_ns);
  EXPECT_EQ(0u, request.response.clock_quality);
  EXPECT_EQ("mock_ground_station", request.response.clock_source);
  EXPECT_EQ("preflight", request.response.clock_phase);
  EXPECT_GE(request.response.current_cycle, last_cycle.cycle_id);
  EXPECT_NE(std::string::npos, request.response.reason.find("adapters=1/1"));
  EXPECT_NE(std::string::npos, request.response.reason.find("weaknet_state="));
  EXPECT_NE(std::string::npos, request.response.reason.find("clock_source=mock_ground_station"));

  // Parameters are read once at startup: a later parameter-server write does
  // not reach the running node (change it and restart the node instead).
  ros::param::set("/swarm_runtime_node/clock_phase", std::string("in_flight"));
  const ros::Time settle = ros::Time::now() + ros::Duration(0.3);
  while (ros::ok() && ros::Time::now() < settle) {
    ros::spinOnce();
    rate.sleep();
  }
  periodic_sync::GetRuntimeStatus after_write;
  after_write.request.session_id = "smoke_session";
  ASSERT_TRUE(client.call(after_write));
  EXPECT_EQ("preflight", after_write.response.clock_phase);
  EXPECT_TRUE(after_write.response.running);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "runtime_node_smoke_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
