// Same-host rostest for sync_coordinator.
//
// Three fake participant processes (fake_sync_participant) announce readiness,
// answer the sequenced triggers with acks, and one of them stops answering for
// a few triggers. This node records everything the coordinator publishes and
// then checks the Ready -> Trigger -> Ack protocol, the timeout handling and the
// statistics. The scenario (periods, timeouts, which triggers stay unanswered)
// is read from the parameter server, where sync_coordinator.test defines it.
//
// This exercises the coordination path only. It does not run any algorithm.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/function.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "periodic_sync/SyncAck.h"
#include "periodic_sync/SyncEvent.h"
#include "periodic_sync/SyncParticipantStats.h"
#include "periodic_sync/SyncReady.h"
#include "periodic_sync/SyncStatistics.h"
#include "periodic_sync/SyncTrigger.h"

using periodic_sync::SyncAck;
using periodic_sync::SyncEvent;
using periodic_sync::SyncParticipantStats;
using periodic_sync::SyncReady;
using periodic_sync::SyncStatistics;
using periodic_sync::SyncTrigger;

namespace {

// A participant id that the coordinator is not configured for.
constexpr uint32_t kUnknownParticipantId = 99;

struct ParticipantConfig {
    uint32_t id = 0;
    double computation_ms = 0.0;
    double ready_delay = 0.0;
    uint64_t silent_from = 0;  // first unanswered trigger; 0 = always answers
    uint64_t silent_count = 0;

    bool silent() const { return silent_count > 0; }
    uint64_t firstAnsweredAfterSilence() const { return silent_from + silent_count; }
    std::string nodeName() const { return "/participant_" + std::to_string(id); }
};

struct Scenario {
    double sync_period = 0.0;  // seconds
    double timeout_ms = 0.0;
    int max_consecutive_timeouts = 0;
    int recovery_good_cycles = 0;
    std::vector<ParticipantConfig> participants;

    const ParticipantConfig& silentParticipant() const {
        for (const ParticipantConfig& participant : participants) {
            if (participant.silent()) {
                return participant;
            }
        }
        throw std::runtime_error("the scenario has no silent participant");
    }

    double slowestComputationMs() const {
        double slowest = 0.0;
        for (const ParticipantConfig& participant : participants) {
            slowest = std::max(slowest, participant.computation_ms);
        }
        return slowest;
    }

    double longestReadyDelay() const {
        double longest = 0.0;
        for (const ParticipantConfig& participant : participants) {
            longest = std::max(longest, participant.ready_delay);
        }
        return longest;
    }

    // Number of the miss that makes the coordinator call a participant degraded.
    uint64_t degradedSequence(const ParticipantConfig& participant) const {
        return participant.silent_from + static_cast<uint64_t>(max_consecutive_timeouts) - 1;
    }

    // Number of the ack that completes the recovery after a silent period.
    uint64_t recoveredSequence(const ParticipantConfig& participant) const {
        return participant.firstAnsweredAfterSilence() + static_cast<uint64_t>(recovery_good_cycles) - 1;
    }
};

struct Recording {
    std::vector<SyncReady> ready;
    std::vector<SyncTrigger> triggers;
    std::vector<SyncEvent> events;
    std::vector<SyncStatistics> statistics;
};

struct Topics {
    std::string ready;
    std::string trigger;
    std::string ack;
    std::string statistics;
    std::string events;
};

class Recorder {
public:
    Recorder(ros::NodeHandle& nh, const Topics& topics) {
        ready_sub_ = subscribeInto(nh, topics.ready, &Recording::ready);
        trigger_sub_ = subscribeInto(nh, topics.trigger, &Recording::triggers);
        statistics_sub_ = subscribeInto(nh, topics.statistics, &Recording::statistics);
        events_sub_ = subscribeInto(nh, topics.events, &Recording::events);
    }

    bool connected(size_t participants) const {
        return ready_sub_.getNumPublishers() >= participants &&
               trigger_sub_.getNumPublishers() >= 1 &&
               statistics_sub_.getNumPublishers() >= 1 &&
               events_sub_.getNumPublishers() >= 1;
    }

    // Polls the recording under its lock until the predicate holds.
    template <typename Predicate>
    bool waitFor(Predicate predicate, double timeout_s) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (predicate(data_)) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    Recording copy() {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

private:
    template <typename Message>
    ros::Subscriber subscribeInto(ros::NodeHandle& nh,
                                  const std::string& topic,
                                  std::vector<Message> Recording::*member) {
        return nh.subscribe<Message>(
            topic,
            1000,
            boost::function<void(const boost::shared_ptr<const Message>&)>(
                [this, member](const boost::shared_ptr<const Message>& message) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    (data_.*member).push_back(*message);
                }));
    }

    std::mutex mutex_;
    Recording data_;
    ros::Subscriber ready_sub_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber statistics_sub_;
    ros::Subscriber events_sub_;
};

Scenario g_scenario;
Recording g_recording;
ros::Time g_gate_open_time;

template <typename T>
T requireParam(const std::string& key) {
    T value;
    if (!ros::param::get(key, value)) {
        throw std::runtime_error("missing parameter " + key);
    }
    return value;
}

Scenario loadScenario() {
    Scenario scenario;
    scenario.sync_period = requireParam<double>("/sync_coordinator/sync_period");
    scenario.timeout_ms = requireParam<double>("/sync_coordinator/timeout_threshold_ms");
    scenario.max_consecutive_timeouts = requireParam<int>("/sync_coordinator/max_consecutive_timeouts");
    scenario.recovery_good_cycles = requireParam<int>("/sync_coordinator/recovery_good_cycles");
    const int count = requireParam<int>("/sync_coordinator/participant_count");
    const int first_id = requireParam<int>("/sync_coordinator/first_participant_id");
    for (int id = first_id; id < first_id + count; ++id) {
        const std::string prefix = "/participant_" + std::to_string(id) + "/";
        ParticipantConfig participant;
        participant.id = static_cast<uint32_t>(id);
        participant.computation_ms = requireParam<double>(prefix + "computation_ms");
        participant.ready_delay = requireParam<double>(prefix + "ready_delay");
        participant.silent_from = static_cast<uint64_t>(requireParam<int>(prefix + "silent_from_sequence"));
        participant.silent_count = static_cast<uint64_t>(requireParam<int>(prefix + "silent_count"));
        scenario.participants.push_back(participant);
    }
    return scenario;
}

std::vector<SyncEvent> eventsOfType(const Recording& recording, uint8_t type) {
    std::vector<SyncEvent> matching;
    for (const SyncEvent& event : recording.events) {
        if (event.event_type == type) {
            matching.push_back(event);
        }
    }
    return matching;
}

std::vector<SyncEvent> eventsOfType(const Recording& recording, uint8_t type, uint32_t participant_id) {
    std::vector<SyncEvent> matching;
    for (const SyncEvent& event : eventsOfType(recording, type)) {
        if (event.participant_id == participant_id) {
            matching.push_back(event);
        }
    }
    return matching;
}

const SyncParticipantStats* findParticipant(const SyncStatistics& statistics, uint32_t participant_id) {
    for (const SyncParticipantStats& participant : statistics.participants) {
        if (participant.participant_id == participant_id) {
            return &participant;
        }
    }
    return nullptr;
}

// True if some published statistics message satisfies the predicate.
template <typename Predicate>
bool anyStatistics(const Recording& recording, Predicate predicate) {
    return std::any_of(recording.statistics.begin(), recording.statistics.end(), predicate);
}

}  // namespace

TEST(SyncCoordinator, TriggersOnlyAfterEveryParticipantIsReady) {
    const Recording& recording = g_recording;

    // Each participant announced itself once on the ready topic.
    for (const ParticipantConfig& participant : g_scenario.participants) {
        int announcements = 0;
        for (const SyncReady& ready : recording.ready) {
            if (ready.participant_id == participant.id) {
                ++announcements;
                EXPECT_EQ(participant.nodeName(), ready.node_name);
            }
        }
        EXPECT_EQ(1, announcements) << "participant " << participant.id;
    }

    // The coordinator reported every participant as ready before it reported "all ready".
    size_t ready_events = 0;
    bool all_ready_reported = false;
    for (const SyncEvent& event : recording.events) {
        if (event.event_type == SyncEvent::EVENT_READY) {
            EXPECT_FALSE(all_ready_reported) << "READY reported after ALL_READY";
            ++ready_events;
        } else if (event.event_type == SyncEvent::EVENT_ALL_READY) {
            all_ready_reported = true;
            EXPECT_EQ(g_scenario.participants.size(), ready_events);
        }
    }
    ASSERT_EQ(1u, eventsOfType(recording, SyncEvent::EVENT_ALL_READY).size());
    for (const ParticipantConfig& participant : g_scenario.participants) {
        EXPECT_EQ(1u, eventsOfType(recording, SyncEvent::EVENT_READY, participant.id).size())
            << "participant " << participant.id;
    }

    // No trigger was published before the last participant announced itself.
    ASSERT_FALSE(recording.triggers.empty());
    const ros::Time first_trigger_time = recording.triggers.front().trigger_time;
    const ros::Time all_ready_time = eventsOfType(recording, SyncEvent::EVENT_ALL_READY).front().stamp;
    EXPECT_GE(first_trigger_time, all_ready_time);
    for (const SyncReady& ready : recording.ready) {
        if (ready.participant_id != kUnknownParticipantId) {
            EXPECT_GE(first_trigger_time, ready.ready_time);
        }
    }
    EXPECT_GE((first_trigger_time - g_gate_open_time).toSec(), g_scenario.longestReadyDelay());
}

TEST(SyncCoordinator, PublishesSequencedPeriodicTriggers) {
    const std::vector<SyncTrigger>& triggers = g_recording.triggers;
    const uint64_t needed =
        g_scenario.recoveredSequence(g_scenario.silentParticipant()) + 1;
    ASSERT_GE(triggers.size(), needed);

    std::vector<uint32_t> expected_ids;
    for (const ParticipantConfig& participant : g_scenario.participants) {
        expected_ids.push_back(participant.id);
    }
    for (size_t i = 0; i < triggers.size(); ++i) {
        EXPECT_EQ(i + 1, triggers[i].sequence_id);
        EXPECT_EQ(expected_ids, triggers[i].active_participant_ids) << "trigger " << triggers[i].sequence_id;
        if (i > 0) {
            EXPECT_GT(triggers[i].trigger_time, triggers[i - 1].trigger_time);
        }
    }

    const double span = (triggers.back().trigger_time - triggers.front().trigger_time).toSec();
    const double mean_period = span / static_cast<double>(triggers.size() - 1);
    EXPECT_NEAR(g_scenario.sync_period, mean_period, 0.2 * g_scenario.sync_period);
}

TEST(SyncCoordinator, StatisticsReflectTheAcksOfResponsiveParticipants) {
    const Recording& recording = g_recording;
    ASSERT_FALSE(recording.statistics.empty());
    const SyncStatistics& last = recording.statistics.back();

    EXPECT_EQ(g_scenario.participants.size(), last.expected_response_count);
    EXPECT_NEAR(g_scenario.sync_period * 1000.0, last.sync_period_ms, 1e-6);
    EXPECT_GE(last.max_cycle_ms, g_scenario.slowestComputationMs() - 1.0);

    // While every participant answers, each round is complete when its timeout expires.
    uint64_t first_silent = recording.triggers.size() + 1;
    for (const ParticipantConfig& participant : g_scenario.participants) {
        if (participant.silent()) {
            first_silent = std::min(first_silent, participant.silent_from);
        }
    }
    for (uint64_t sequence = 1; sequence < first_silent; ++sequence) {
        EXPECT_TRUE(anyStatistics(recording, [&](const SyncStatistics& statistics) {
            return statistics.sequence_id == sequence &&
                   statistics.actual_response_count == g_scenario.participants.size() &&
                   statistics.missing_response_count == 0 &&
                   statistics.round_finish_ms >= g_scenario.slowestComputationMs();
        })) << "no complete round reported for sequence " << sequence;
    }

    for (const ParticipantConfig& config : g_scenario.participants) {
        if (config.silent()) {
            continue;
        }
        SCOPED_TRACE("participant " + std::to_string(config.id));
        const SyncParticipantStats* participant = findParticipant(last, config.id);
        ASSERT_NE(nullptr, participant);

        EXPECT_TRUE(participant->ready);
        EXPECT_EQ(config.nodeName(), participant->node_name);
        EXPECT_EQ(SyncParticipantStats::STATUS_OK, participant->status);
        EXPECT_EQ(0u, participant->missed_count);
        EXPECT_EQ(0u, participant->slow_count);
        EXPECT_EQ(0u, participant->failure_count);
        EXPECT_GE(participant->last_ack_sequence_id + 2, last.sequence_id);
        EXPECT_GE(participant->good_cycle_count + 2, last.sequence_id);
        EXPECT_FALSE(participant->last_response_time.isZero());

        // The participant sleeps for computation_ms, so that is the lower bound for
        // both the reported computation time and the received -> finished cycle time.
        EXPECT_GE(participant->computation_mean_ms, config.computation_ms);
        EXPECT_LT(participant->computation_mean_ms, config.computation_ms + 50.0);
        EXPECT_GE(participant->cycle_mean_ms, participant->computation_mean_ms - 1.0);
        EXPECT_LT(participant->cycle_mean_ms, config.computation_ms + 50.0);
        EXPECT_GE(participant->computation_max_ms, participant->computation_mean_ms);
        EXPECT_GE(participant->cycle_max_ms, participant->cycle_mean_ms);
        EXPECT_GT(participant->computation_ewma_ms, 0.0);
        EXPECT_GT(participant->cycle_ewma_ms, 0.0);

        // A responsive participant is never penalised at any point of the run.
        EXPECT_TRUE(std::none_of(
            recording.statistics.begin(), recording.statistics.end(), [&](const SyncStatistics& statistics) {
                const SyncParticipantStats* seen = findParticipant(statistics, config.id);
                return seen == nullptr || seen->status != SyncParticipantStats::STATUS_OK ||
                       seen->missed_count != 0;
            }));
        for (const SyncEvent& event : recording.events) {
            if (event.participant_id == config.id) {
                EXPECT_EQ(SyncEvent::EVENT_READY, event.event_type)
                    << "unexpected event " << static_cast<int>(event.event_type) << " at sequence "
                    << event.sequence_id;
            }
        }
    }
}

TEST(SyncCoordinator, SilentParticipantTimesOutAndIsDegraded) {
    const Recording& recording = g_recording;
    const ParticipantConfig& silent = g_scenario.silentParticipant();

    // Every unanswered trigger is reported once, in order, with the configured threshold.
    const std::vector<SyncEvent> missed = eventsOfType(recording, SyncEvent::EVENT_MISSED);
    ASSERT_EQ(silent.silent_count, missed.size());
    for (size_t i = 0; i < missed.size(); ++i) {
        EXPECT_EQ(silent.id, missed[i].participant_id);
        EXPECT_EQ(silent.silent_from + i, missed[i].sequence_id);
        EXPECT_DOUBLE_EQ(g_scenario.timeout_ms, missed[i].threshold_ms);
    }

    // Degraded exactly when the number of consecutive misses reaches the limit.
    ASSERT_GE(silent.silent_count, static_cast<uint64_t>(g_scenario.max_consecutive_timeouts));
    const std::vector<SyncEvent> degraded = eventsOfType(recording, SyncEvent::EVENT_DEGRADED);
    ASSERT_EQ(1u, degraded.size());
    EXPECT_EQ(silent.id, degraded.front().participant_id);
    EXPECT_EQ(g_scenario.degradedSequence(silent), degraded.front().sequence_id);

    // The statistics follow the misses one by one.
    for (uint64_t miss = 1; miss <= silent.silent_count; ++miss) {
        const uint8_t expected_status = miss >= static_cast<uint64_t>(g_scenario.max_consecutive_timeouts)
                                            ? SyncParticipantStats::STATUS_DEGRADED
                                            : SyncParticipantStats::STATUS_MISSING;
        EXPECT_TRUE(anyStatistics(recording, [&](const SyncStatistics& statistics) {
            const SyncParticipantStats* participant = findParticipant(statistics, silent.id);
            return participant != nullptr && participant->missed_count == miss &&
                   participant->consecutive_missed_count == miss && participant->status == expected_status;
        })) << "no statistics with " << miss << " consecutive misses and status "
            << static_cast<int>(expected_status);
    }

    // For each unanswered round, the round summary names the missing response.
    for (uint64_t sequence = silent.silent_from; sequence < silent.firstAnsweredAfterSilence(); ++sequence) {
        EXPECT_TRUE(anyStatistics(recording, [&](const SyncStatistics& statistics) {
            return statistics.sequence_id == sequence &&
                   statistics.actual_response_count == g_scenario.participants.size() - 1 &&
                   statistics.missing_response_count == 1;
        })) << "no incomplete round reported for sequence " << sequence;
    }
}

TEST(SyncCoordinator, SilentParticipantRecoversAfterConsecutiveAcks) {
    const Recording& recording = g_recording;
    const ParticipantConfig& silent = g_scenario.silentParticipant();
    const uint64_t recovered_sequence = g_scenario.recoveredSequence(silent);

    const std::vector<SyncEvent> recovered = eventsOfType(recording, SyncEvent::EVENT_RECOVERED);
    ASSERT_EQ(1u, recovered.size());
    EXPECT_EQ(silent.id, recovered.front().participant_id);
    EXPECT_EQ(recovered_sequence, recovered.front().sequence_id);

    // Until enough consecutive healthy acks arrived, the participant stays degraded.
    for (uint64_t sequence = silent.firstAnsweredAfterSilence(); sequence < recovered_sequence; ++sequence) {
        EXPECT_TRUE(anyStatistics(recording, [&](const SyncStatistics& statistics) {
            const SyncParticipantStats* participant = findParticipant(statistics, silent.id);
            return participant != nullptr && participant->last_ack_sequence_id == sequence &&
                   participant->status == SyncParticipantStats::STATUS_DEGRADED;
        })) << "participant left the degraded state before sequence " << recovered_sequence;
    }

    const SyncParticipantStats* participant = findParticipant(recording.statistics.back(), silent.id);
    ASSERT_NE(nullptr, participant);
    EXPECT_EQ(SyncParticipantStats::STATUS_OK, participant->status);
    EXPECT_EQ(0u, participant->consecutive_missed_count);
    EXPECT_EQ(silent.silent_count, participant->missed_count);
    EXPECT_GE(participant->last_ack_sequence_id, recovered_sequence);
    EXPECT_GE(participant->good_cycle_count, static_cast<uint32_t>(g_scenario.recovery_good_cycles));
}

TEST(SyncCoordinator, ReportsParticipantsItIsNotConfiguredFor) {
    const Recording& recording = g_recording;

    const std::vector<SyncEvent> unknown =
        eventsOfType(recording, SyncEvent::EVENT_UNKNOWN_PARTICIPANT, kUnknownParticipantId);
    ASSERT_EQ(2u, unknown.size());  // one for the Ready message, one for the Ack
    std::set<std::string> messages;
    for (const SyncEvent& event : unknown) {
        messages.insert(event.message);
    }
    EXPECT_EQ(2u, messages.size());

    for (const SyncStatistics& statistics : recording.statistics) {
        EXPECT_EQ(g_scenario.participants.size(), statistics.participants.size());
        EXPECT_EQ(nullptr, findParticipant(statistics, kUnknownParticipantId));
    }
    for (const SyncTrigger& trigger : recording.triggers) {
        EXPECT_EQ(0, std::count(trigger.active_participant_ids.begin(),
                                trigger.active_participant_ids.end(),
                                kUnknownParticipantId));
    }
}

// The message definitions are shared with the periodic_sync copy frozen with the
// TRO DMPC submission. A different MD5 sum would keep nodes built against the two
// copies from connecting, so these sums must not change.
TEST(MessageContract, Md5SumsAreUnchanged) {
    EXPECT_STREQ("7ad8f559f6966e96a04cd2fef9d76f17", ros::message_traits::md5sum<SyncReady>());
    EXPECT_STREQ("6553376adec5f310164bbefb1219fbfe", ros::message_traits::md5sum<SyncTrigger>());
    EXPECT_STREQ("7a44caac901b9f38526955377eacceba", ros::message_traits::md5sum<SyncAck>());
    EXPECT_STREQ("870ca1e0e37fee17dc4d8b328345de24", ros::message_traits::md5sum<SyncParticipantStats>());
    EXPECT_STREQ("870cdbd575a77d5c163fe35fd26d9130", ros::message_traits::md5sum<SyncStatistics>());
    EXPECT_STREQ("33e320bfcd98324f7b163ba28323a8ef", ros::message_traits::md5sum<SyncEvent>());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "sync_coordinator_rostest");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    try {
        g_scenario = loadScenario();
        const ParticipantConfig& silent = g_scenario.silentParticipant();

        Topics topics;
        topics.ready = requireParam<std::string>("~ready_topic");
        topics.trigger = requireParam<std::string>("~trigger_topic");
        topics.ack = requireParam<std::string>("~ack_topic");
        topics.statistics = requireParam<std::string>("~statistics_topic");
        topics.events = requireParam<std::string>("~events_topic");
        const std::string gate_param = requireParam<std::string>("~gate_param");

        Recorder recorder(nh, topics);
        const size_t participants = g_scenario.participants.size();
        auto wait_for_connections = [&] {
            for (int i = 0; i < 2000 && !recorder.connected(participants); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            return recorder.connected(participants);
        };
        if (!wait_for_connections()) {
            ROS_ERROR("The coordinator and the participants did not show up");
            return 1;
        }

        // Let the participants announce themselves now that every event will be seen.
        g_gate_open_time = ros::Time::now();
        ros::param::set(gate_param, true);

        // Run until the silent participant has been through silence and recovery.
        const uint64_t recovered_sequence = g_scenario.recoveredSequence(silent);
        const double scenario_timeout =
            g_scenario.longestReadyDelay() + 20.0 +
            g_scenario.sync_period * static_cast<double>(recovered_sequence + 5);
        const bool completed = recorder.waitFor(
            [&](const Recording& recording) {
                return recording.triggers.size() >= recovered_sequence + 3 &&
                       std::any_of(recording.events.begin(), recording.events.end(), [&](const SyncEvent& event) {
                           return event.event_type == SyncEvent::EVENT_RECOVERED;
                       });
            },
            scenario_timeout);
        if (!completed) {
            ROS_ERROR("The scenario did not complete; the checks below show how far it got");
        }

        // A participant the coordinator does not know must be reported and ignored.
        // The publishers latch, so the coordinator gets each message exactly once
        // as soon as it is connected.
        ros::Publisher unknown_ready_pub = nh.advertise<SyncReady>(topics.ready, 1, true);
        ros::Publisher unknown_ack_pub = nh.advertise<SyncAck>(topics.ack, 1, true);
        SyncReady unknown_ready;
        unknown_ready.participant_id = kUnknownParticipantId;
        unknown_ready.node_name = "/unknown_participant";
        unknown_ready.ready_time = ros::Time::now();
        unknown_ready_pub.publish(unknown_ready);
        SyncAck unknown_ack;
        unknown_ack.participant_id = kUnknownParticipantId;
        unknown_ack.sequence_id = 1;
        unknown_ack.received_time = ros::Time::now();
        unknown_ack.finished_time = unknown_ack.received_time;
        unknown_ack.success = true;
        unknown_ack_pub.publish(unknown_ack);
        recorder.waitFor(
            [](const Recording& recording) {
                return std::count_if(recording.events.begin(), recording.events.end(), [](const SyncEvent& event) {
                           return event.event_type == SyncEvent::EVENT_UNKNOWN_PARTICIPANT;
                       }) >= 2;
            },
            10.0);

        g_recording = recorder.copy();
    } catch (const std::exception& error) {
        ROS_ERROR("Cannot set up the scenario: %s", error.what());
        return 1;
    }

    const int result = RUN_ALL_TESTS();
    ros::shutdown();
    return result;
}
