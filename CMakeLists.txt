cmake_minimum_required(VERSION 3.16)
project(periodic_sync)
find_package(XgcXrpc REQUIRED CONFIG)
find_package(Boost 1.75 REQUIRED COMPONENTS json)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  BudgetReport.msg
  ChannelStats.msg
  CycleSnapshot.msg
  PeerLinkHealth.msg
  PeerSampleStatus.msg
  Recommendation.msg
  RuntimeHealth.msg
  SampleStats.msg
  SyncReady.msg
  SyncTrigger.msg
  SyncAck.msg
  SyncParticipantStats.msg
  SyncStatistics.msg
  SyncEvent.msg
  SyncedCycle.msg
  WeakNetState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swarm_sync_core swarm_sync_ros1_adapter
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(swarm_sync_core
  src/core/clock_monitor.cpp
  src/core/cycle_scheduler.cpp
  src/core/deadline_checker.cpp
  src/core/envelope_codec.cpp
  src/core/sample_buffer.cpp
  src/core/session.cpp
  src/core/snapshot_builder.cpp
  src/core/transport.cpp
  src/core/transport_keys.cpp
  src/core/weaknet/budget_guard.cpp
  src/core/weaknet/channel_profile.cpp
  src/core/weaknet/cycle_window.cpp
  src/core/weaknet/link_health.cpp
  src/core/weaknet/payload_guard.cpp
  src/core/weaknet/recommendation_engine.cpp
  src/core/weaknet/send_stagger.cpp
  src/core/weaknet/state_machine.cpp
)
target_compile_features(swarm_sync_core PUBLIC cxx_std_17)

add_library(swarm_sync_ros1_adapter
  src/ros1/adapter_config.cpp
)
target_compile_features(swarm_sync_ros1_adapter PUBLIC cxx_std_17)
target_link_libraries(swarm_sync_ros1_adapter
  swarm_sync_core
  ${catkin_LIBRARIES}
)

add_executable(sync_coordinator
  src/sync_coordinator_node.cpp
)
add_dependencies(sync_coordinator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sync_coordinator
  ${catkin_LIBRARIES}
)

add_library(swarm_sync_control src/core/runtime_control.cpp)
target_compile_features(swarm_sync_control PUBLIC cxx_std_20)
target_link_libraries(swarm_sync_control PUBLIC XgcXrpc::http Boost::json Threads::Threads)

add_executable(swarm_runtime_node
  src/ros1/runtime_node.cpp
)
add_dependencies(swarm_runtime_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(swarm_runtime_node PUBLIC cxx_std_20)
target_link_libraries(swarm_runtime_node
  swarm_sync_control
  swarm_sync_ros1_adapter
  swarm_sync_core
  ${catkin_LIBRARIES}
)
set_target_properties(swarm_runtime_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

add_executable(swarm_ground_station_node
  src/ros1/ground_station_node.cpp
)
add_dependencies(swarm_ground_station_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(swarm_ground_station_node
  swarm_sync_core
  ${catkin_LIBRARIES}
)
set_target_properties(swarm_ground_station_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS swarm_sync_control swarm_sync_core swarm_sync_ros1_adapter sync_coordinator swarm_runtime_node swarm_ground_station_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY config docs launch scripts specs tests tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  FILES_MATCHING PATTERN "*.msg"
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(swarm_sync_core_tests test/core_tests.cpp)
  if(TARGET swarm_sync_core_tests)
    target_compile_features(swarm_sync_core_tests PUBLIC cxx_std_17)
    target_link_libraries(swarm_sync_core_tests swarm_sync_ros1_adapter swarm_sync_core ${catkin_LIBRARIES})
  endif()

  add_executable(runtime_node_smoke_test EXCLUDE_FROM_ALL test/runtime_node_smoke.cpp)
  if(TARGET runtime_node_smoke_test)
    add_dependencies(runtime_node_smoke_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_compile_features(runtime_node_smoke_test PUBLIC cxx_std_20)
    target_link_libraries(runtime_node_smoke_test gtest XgcXrpc::http Boost::json ${catkin_LIBRARIES})
  endif()

  add_rostest(test/runtime_node_smoke.test DEPENDENCIES swarm_runtime_node runtime_node_smoke_test)
  add_test(NAME check_chrony_script_test
           COMMAND bash
                   ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_chrony_script_test.sh
                   ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_chrony.sh)
endif()
