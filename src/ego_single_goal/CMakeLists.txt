cmake_minimum_required(VERSION 3.0.2)
project(ego_single_goal)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  across_gate_module
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    across_gate_module
    geometry_msgs
    mavros_msgs
    nav_msgs
    roscpp
    rospy
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(competition_mission_manager
  src/competition_mission_manager.cpp
)

add_dependencies(competition_mission_manager
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(competition_mission_manager
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/livox_cloud_to_ego_world.py
  src/task_a_gap_detector.py
  src/task_a_waypoint_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS competition_mission_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
