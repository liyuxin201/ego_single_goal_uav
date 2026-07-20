cmake_minimum_required(VERSION 3.0.2)
project(ego_single_goal)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/livox_cloud_to_ego_world.py
  src/task_a_gap_detector.py
  src/task_a_waypoint_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
