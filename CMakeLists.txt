cmake_minimum_required(VERSION 3.16)
project(periodic_sync)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)

add_message_files(
  FILES
  SyncReady.msg
  SyncTrigger.msg
  SyncAck.msg
  SyncParticipantStats.msg
  SyncStatistics.msg
  SyncEvent.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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

install(TARGETS sync_coordinator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
