#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ego_single_goal ego_single_goal_px4.launch "$@"
