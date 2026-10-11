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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_executable(fake_sync_participant EXCLUDE_FROM_ALL
    test/fake_sync_participant.cpp
  )
  add_dependencies(fake_sync_participant
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(fake_sync_participant
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(sync_coordinator_rostest
    test/sync_coordinator.test
    test/sync_coordinator_test.cpp
  )
  if(TARGET sync_coordinator_rostest)
    add_dependencies(sync_coordinator_rostest
      sync_coordinator
      fake_sync_participant
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(sync_coordinator_rostest
      ${catkin_LIBRARIES}
    )
  endif()
endif()
