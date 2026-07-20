# ego_single_goal_uav

Standalone catkin workspace for the Task A single-goal PX4 launch.

This workspace contains `ego_single_goal` plus the local packages needed by `autonomous_flight` and EGO-Planner. It intentionally excludes PX4-Autopilot, PX4-SITL_gazebo-classic, Mid360 Gazebo simulation, and the original `competition_scene` package.

## Build

```bash
cd ~/ego_single_goal_uav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch ego_single_goal ego_single_goal_px4.launch
```

Without RViz:

```bash
roslaunch ego_single_goal ego_single_goal_px4.launch rviz:=false
```

## NUC migration

Copy the whole `ego_single_goal_uav` directory to the NUC, install ROS Noetic and system dependencies, then run the build commands above.
