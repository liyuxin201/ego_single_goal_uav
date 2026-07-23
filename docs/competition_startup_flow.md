# 完整比赛启动流程

本文说明完整比赛运行时各工作空间、各 launch 文件和上层调度节点的启动顺序。

适用范围：

- 这是比赛现场启动文档，重点回答“完整比赛要按什么顺序启动、每一步检查什么”。
- 上层调度设计说明见：`docs/competition_mission_manager_design.md`。
- 建议先分终端启动并检查 topic 稳定，再启动最后的 `competition_mission_manager`。

当前系统由多个工作空间组成：

```text
~/uav_ws                    Livox / FAST-LIVO / livox_to_world
~/Fast-Drone-250             PX4 控制
~/ego_single_goal_uav        A区模块、EGO、上层比赛调度
/home/uavx/gate_module_ws    D区穿框模块
```

推荐采用“分终端启动 + 上层调度统一管理任务流程”的方式。

## 1. 启动前检查

### 1.1 修改比赛点位参数

先修改：

```bash
~/ego_single_goal_uav/src/ego_single_goal/config/competition_mission.yaml
```

正式比赛前至少要确认这些参数：

```yaml
task_b_enabled: true
task_b_waypoints:
  - [B入口x, B入口y, 1.0]
  - [B出口x, B出口y, 1.0]

task_c_enabled: true
task_c_waypoints:
  - [C入口x, C入口y, 1.0]
  - [C出口x, C出口y, 1.0]

task_d_prepare_waypoints:
  - [D区前准备点x, D区前准备点y, 1.0]
```

说明：

- B 区暂时没有独立识别模块，但正式流程建议 `task_b_enabled: true`，用预设点占位，保证 A→B→C→D 顺序。
- C 区依靠 EGO 避障，`task_c_waypoints` 至少应包含入口和出口。
- D 区准备点应该在 gate module 能看到门框前的位置。

### 1.2 编译上层调度节点

如果改过 C++ 节点或第一次使用，需要编译：

```bash
source /opt/ros/noetic/setup.bash
source /home/uavx/gate_module_ws/devel/setup.bash

cd ~/ego_single_goal_uav
catkin_make --pkg ego_single_goal
source devel/setup.bash
```

必须先 source `gate_module_ws`，因为 C++ 调度节点依赖：

```text
across_gate_module/GatePlanAction
```

### 1.3 检查关键文件

上层调度节点：

```text
~/ego_single_goal_uav/src/ego_single_goal/src/competition_mission_manager.cpp
```

上层调度参数：

```text
~/ego_single_goal_uav/src/ego_single_goal/config/competition_mission.yaml
```

上层调度 launch：

```text
~/ego_single_goal_uav/src/ego_single_goal/launch/competition_mission_manager.launch
```

## 2. 推荐启动顺序

### 终端 1：启动 MAVROS

```bash
sudo chmod 666 /dev/ttyUSB1
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB1:921600
```

检查：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
```

期望：

- `/mavros/state` 有输出。
- `/mavros/local_position/pose` 有稳定位姿。

### 终端 2：启动 USB 相机

```bash
source /opt/ros/noetic/setup.bash
roslaunch usb_cam usb_cam-test.launch
```

检查：

```bash
rostopic hz /usb_cam/image_raw
```

D 区 gate module 需要相机图像，必须确认该 topic 有数据。

### 终端 3：启动 Livox MID360

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash

roslaunch livox_ros_driver2 msg_MID360.launch
```

检查：

```bash
rostopic hz /livox/lidar
```

### 终端 4：启动 FAST-LIVO

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash

roslaunch fast_livo mapping_mid360.launch
```

检查：

```bash
rostopic list | grep -E "aft|cloud|mapped|odom"
```

如果 gate module 使用 `/aft_mapped_to_init`，还要检查：

```bash
rostopic echo /aft_mapped_to_init
```

### 终端 5：启动点云转换

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash

roslaunch test livox_to_world.launch
```

检查：

```bash
rostopic hz /world_cloud
```

如果 EGO launch 中 `cloud_topic` 是 `/world_cloud`，这里必须有数据。

### 终端 6：启动 PX4 控制

```bash
source /opt/ros/noetic/setup.bash
cd ~/Fast-Drone-250
source devel_uavx/setup.bash

roslaunch px4 px4_control.launch
```

这个节点负责底层控制链。启动后确认模式切换、控制输出和安全开关正常。

### 终端 7：启动 EGO + A 区模块

```bash
source /opt/ros/noetic/setup.bash
source /home/uavx/gate_module_ws/devel/setup.bash
cd ~/ego_single_goal_uav
source devel/setup.bash

roslaunch ego_single_goal ego_single_goal_px4_interactive.launch \
  auto_detect:=true \
  target_ready:=false \
  auto_task_a_waypoints:=false
```

关键参数：

```bash
auto_task_a_waypoints:=false
```

原因：

- 旧的 `task_a_waypoint_manager.py` 也会发布 `/move_base_simple/goal`。
- 新的 C++ 上层调度节点也会发布 `/move_base_simple/goal`。
- 两者不能同时运行，否则会抢目标点。

`target_ready:=false` 可以保留，因为上层调度节点会向：

```text
/task_a/final_goal
```

发布 A 区最终目标，触发 A 区模块输出正式路径。

### 终端 8：启动 D 区穿框模块

推荐使用部署脚本：

```bash
cd /home/uavx/gate_module_ws
./deployment/run_full_module.sh
```

或者手动启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/uavx/gate_module_ws/devel/setup.bash

roslaunch across_gate_module onboard.launch
```

检查：

```bash
rostopic list | grep gate_module
rostopic echo /gate_module/state
```

正常启动后，gate module 应该处于：

```text
IDLE
```

### 终端 9：启动上层比赛调度

```bash
source /opt/ros/noetic/setup.bash
source /home/uavx/gate_module_ws/devel/setup.bash
cd ~/ego_single_goal_uav
source devel/setup.bash

roslaunch ego_single_goal competition_mission_manager.launch
```

如果要使用自定义参数文件：

```bash
roslaunch ego_single_goal competition_mission_manager.launch \
  config_file:=/path/to/your_competition_mission.yaml
```

## 3. 启动后完整比赛流程

上层调度节点启动后会自动执行：

```text
WAIT_READY
  等待里程计和 OFFBOARD

TAKEOFF
  发布起飞高度目标

TASK_A_WAIT_PATH
  向 /task_a/final_goal 发布 A 区目标
  等待 /task_a/guide_path

TASK_A_FLY
  发布 A 区路径到 /competition/guide_path
  发布 A 区终点到 /move_base_simple/goal

TASK_B_SETUP / TASK_B_FLY
  用 YAML 里的 B 区预设点通过

TASK_C_SETUP / TASK_C_FLY
  用 YAML 里的 C 区入口/出口点
  EGO 根据点云自主避障

TASK_D_PREPARE
  飞到 D 区前准备点

TASK_D_START_ACTION
  调用 /gate_module/plan

TASK_D_FLY_OBSERVATION
  飞到 gate module feedback 返回的 observation_pose

TASK_D_TRAVERSE
  飞 gate module result 返回的 traversal_path

MISSION_DONE
  悬停在最终点
```

## 4. 运行中监控命令

### 4.1 检查基础输入

```bash
rostopic hz /mavros/local_position/odom
rostopic echo /mavros/state
rostopic hz /world_cloud
rostopic hz /usb_cam/image_raw
```

### 4.2 检查 A 区输出

```bash
rostopic echo /task_a/guide_path
rostopic echo /task_a/final_goal
```

### 4.3 检查 EGO 接口

```bash
rostopic echo /competition/guide_path
rostopic echo /move_base_simple/goal
```

### 4.4 检查 D 区模块

```bash
rostopic echo /gate_module/state
rostopic echo /gate_module/observation_pose
rostopic echo /gate_module/traversal_path
```

### 4.5 检查 TF

如果 D 区返回的坐标系不是 `map`，必须确认 TF 存在：

```bash
rosrun tf tf_echo map camera_init
```

或者按实际坐标系替换。

## 5. 常见问题

### 5.1 调度节点一直停在 WAIT_READY

检查：

```bash
rostopic echo /mavros/local_position/odom
rostopic echo /mavros/state
```

如果还没有切 OFFBOARD，且参数：

```yaml
wait_for_offboard: true
```

调度节点会一直等待。

### 5.2 A 区一直等不到路径

检查：

```bash
rostopic echo /task_a/final_goal
rostopic echo /task_a/guide_path
```

确认：

- A 区点云正常。
- `auto_detect:=true`。
- 上层调度已向 `/task_a/final_goal` 发目标。
- A 区识别到了墙/圆柱结构。

如果只是调试，可以临时打开：

```yaml
task_a_use_candidate_path: true
```

允许调度节点使用 `/task_a/candidate_guide_path`。

### 5.3 C 区被跳过

检查：

```yaml
task_c_enabled: true
task_c_waypoints: []
```

如果 `task_c_waypoints` 是空，调度节点会跳过 C，直接进入 D。

正式比赛前必须填写 C 区入口/出口点。

### 5.4 D 区 Action 一直等待

检查：

```bash
rostopic echo /gate_module/state
rostopic echo /gate_module/observation_pose
```

可能原因：

- gate module 没有启动。
- 相机没有图像。
- 点云或 TF 不对。
- 飞机没有到达 `observation_pose`。
- D 区准备点离门框太远或角度不对。

### 5.5 飞机往错误方向飞

优先检查坐标系：

```bash
rostopic echo /competition/guide_path/header/frame_id
rostopic echo /move_base_simple/goal/header/frame_id
rosrun tf tf_echo map camera_init
```

如果 gate module 输出 `camera_init`，而 EGO 使用 `map`，必须保证有 TF 转换。

### 5.6 目标点被两个节点抢发

确认启动 EGO/A 模块时使用：

```bash
auto_task_a_waypoints:=false
```

否则旧的 `task_a_waypoint_manager.py` 和新的 `competition_mission_manager` 会同时发布 `/move_base_simple/goal`。

## 6. 推荐一键启动方向

现场调试稳定前，建议保持多终端启动。

稳定后可以写一个 `tmux` 或 shell 脚本，按顺序启动：

```text
1. MAVROS
2. usb_cam
3. Livox
4. FAST-LIVO
5. livox_to_world
6. px4_control
7. EGO + A module
8. gate module
9. competition_mission_manager
```

不要一开始写成一个巨大 launch。多终端更方便定位是哪一层出问题。
