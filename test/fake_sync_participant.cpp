// Stand-in for one algorithm process (for example a DMPC agent) in the
// sync_coordinator rostest. It follows the participant side of the protocol:
// announce readiness once, run one cycle per SyncTrigger and answer with a
// SyncAck. Every instance is a separate process, as in a same-host deployment.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "periodic_sync/SyncAck.h"
#include "periodic_sync/SyncReady.h"
#include "periodic_sync/SyncTrigger.h"

class FakeSyncParticipant {
public:
    explicit FakeSyncParticipant(ros::NodeHandle& nh) {
        ros::NodeHandle private_nh("~");
        participant_id_ = private_nh.param("participant_id", 1);
        // Simulated solver time per cycle.
        computation_ms_ = private_nh.param("computation_ms", 5.0);
        // Ready is announced this long after the gate parameter turns true.
        ready_delay_ = private_nh.param("ready_delay", 0.0);
        // Triggers silent_from_sequence .. silent_from_sequence + silent_count - 1
        // are received but never acknowledged.
        silent_from_sequence_ = private_nh.param("silent_from_sequence", 0);
        silent_count_ = private_nh.param("silent_count", 0);
        // Boolean parameter that must be true before Ready is announced; lets the
        // test observer connect first so that it sees every event.
        gate_param_ = private_nh.param<std::string>("gate_param", "");

        ack_pub_ = nh.advertise<periodic_sync::SyncAck>(
            private_nh.param<std::string>("ack_topic", "/sync/ack"), 10);
        // Latched, like the TRO agents' formation ready publisher.
        ready_pub_ = nh.advertise<periodic_sync::SyncReady>(
            private_nh.param<std::string>("ready_topic", "/sync/ready"), 10, true);
        trigger_sub_ = nh.subscribe(private_nh.param<std::string>("trigger_topic", "/sync/trigger"),
                                    10,
                                    &FakeSyncParticipant::triggerCallback,
                                    this);
        ready_timer_ = nh.createTimer(ros::Duration(0.02), &FakeSyncParticipant::readyTimerCallback, this);
    }

private:
    // Ready is only announced once the coordinator is connected in both
    // directions, so that no trigger or ack can be lost to connection setup.
    void readyTimerCallback(const ros::TimerEvent&) {
        if (!gate_param_.empty()) {
            bool gate_open = false;
            if (!ros::param::get(gate_param_, gate_open) || !gate_open) {
                return;
            }
        }
        if (gate_open_time_.isZero()) {
            gate_open_time_ = ros::Time::now();
        }
        if (ros::Time::now() < gate_open_time_ + ros::Duration(ready_delay_) ||
            trigger_sub_.getNumPublishers() == 0 ||
            ack_pub_.getNumSubscribers() == 0) {
            return;
        }

        periodic_sync::SyncReady ready;
        ready.participant_id = static_cast<uint32_t>(participant_id_);
        ready.node_name = ros::this_node::getName();
        ready.ready_time = ros::Time::now();
        ready_pub_.publish(ready);
        ready_timer_.stop();
        ROS_INFO("Participant %d announced ready", participant_id_);
    }

    void triggerCallback(const periodic_sync::SyncTrigger::ConstPtr& trigger) {
        const ros::Time received_time = ros::Time::now();
        const uint32_t id = static_cast<uint32_t>(participant_id_);
        if (std::find(trigger->active_participant_ids.begin(),
                      trigger->active_participant_ids.end(),
                      id) == trigger->active_participant_ids.end()) {
            ROS_WARN("Participant %d is not active in trigger %lu", participant_id_, trigger->sequence_id);
            return;
        }

        const int64_t sequence = static_cast<int64_t>(trigger->sequence_id);
        if (silent_from_sequence_ > 0 &&
            sequence >= silent_from_sequence_ &&
            sequence < silent_from_sequence_ + silent_count_) {
            ROS_INFO("Participant %d stays silent for trigger %lu", participant_id_, trigger->sequence_id);
            return;
        }

        const auto solve_start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(computation_ms_));
        const double solve_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count();

        periodic_sync::SyncAck ack;
        ack.participant_id = id;
        ack.sequence_id = trigger->sequence_id;
        ack.received_time = received_time;
        ack.finished_time = ros::Time::now();
        ack.computation_ms = solve_ms;
        ack.success = true;
        ack_pub_.publish(ack);
    }

    int participant_id_;
    double computation_ms_;
    double ready_delay_;
    int silent_from_sequence_;
    int silent_count_;
    std::string gate_param_;
    ros::Time gate_open_time_;

    ros::Publisher ack_pub_;
    ros::Publisher ready_pub_;
    ros::Subscriber trigger_sub_;
    ros::Timer ready_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_sync_participant");
    ros::NodeHandle nh;
    FakeSyncParticipant participant(nh);
    ros::spin();
    return 0;
}
