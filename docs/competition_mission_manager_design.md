# 上层比赛任务调度节点设计

本文说明 `competition_mission_manager` 的设计思路、职责边界、状态机流程，以及它和现有 A 区、C 区、D 区代码模块的对接方式。

适用范围：

- 这是上层调度节点的设计文档，重点回答“调度节点做什么、放哪里、怎么和现有模块对接”。
- 当前实现对应节点：`src/ego_single_goal/src/competition_mission_manager.cpp`。
- 配套启动文档见：`docs/competition_startup_flow.md`。

## 1. 节点应该放在哪里

上层调度节点建议放在 `ego_single_goal` 包中：

```text
src/ego_single_goal/src/competition_mission_manager.cpp
```

原因：

- `ego_single_goal` 已经包含 A 区检测、EGO 规划启动和 `/move_base_simple/goal`、`/competition/guide_path` 等飞行接口。
- 上层调度只负责“比赛流程管理”，不应该搬动 `gate_module_ws` 或 `Fast-Drone-250` 的源码。
- D 区穿框模块已经通过 ROS Action 暴露接口，调度节点只需要作为 Action client 调用 `/gate_module/plan`。
- 多工作空间可以通过 `source` overlay 方式软整合，不需要强行合并成一个大工作空间。

## 2. 总体架构

上层任务调度节点的定位是“比赛流程大脑”：

```text
传感器/里程计层
  MAVROS + usb_cam + Livox + FAST-LIVO

控制/规划层
  Fast-Drone-250 px4_control
  ego_single_goal_uav / EGO planner

任务模块层
  A区模块：task_a_gap_detector.py
  B区模块：暂时空，用预设点占位
  C区模块：EGO 直接规划
  D区模块：gate_module_ws /gate_module/plan Action

上层调度层
  competition_mission_manager.cpp
```

上层调度节点不做以下事情：

- 不直接控制电机。
- 不直接发布 MAVROS setpoint。
- 不替代 EGO 局部规划。
- 不把 A/D 感知代码硬合到一起。

它只做以下事情：

- 判断当前比赛阶段。
- 等待当前阶段所需输入。
- 调用对应任务模块。
- 发布当前阶段目标点和引导路径。
- 判断到达后切换到下一阶段。

## 3. 模块职责划分

| 模块 | 主要职责 | 上层调度如何对接 |
| --- | --- | --- |
| MAVROS | 飞控通信、状态、局部位姿 | 订阅 `/mavros/state`、`/mavros/local_position/odom` |
| FAST-LIVO / livox_to_world | 里程计和世界点云 | EGO/A/D 模块使用，上层只依赖位姿 |
| EGO / autonomous_flight | 局部避障、轨迹规划、控制目标生成 | 上层发布 `/move_base_simple/goal` 和 `/competition/guide_path` |
| A 区模块 | 识别墙/圆柱，生成 A 区穿越路径 | 上层订阅 `/task_a/guide_path`，必要时向 `/task_a/final_goal` 发布目标 |
| B 区模块 | 暂未实现 | 上层先用 YAML 预设点通过 |
| C 区 | 森林区避障 | 上层给 C 区入口/出口点，EGO 自主避障 |
| D 区 gate module | 门框识别、观察点生成、七航点穿框路径 | 上层调用 `/gate_module/plan` Action |

## 4. 对外接口

### 4.1 订阅

```text
/mavros/local_position/odom
/mavros/state
/task_a/guide_path
/task_a/candidate_guide_path
```

含义：

- `/mavros/local_position/odom`：用于判断是否到达当前目标点。
- `/mavros/state`：用于等待 OFFBOARD。
- `/task_a/guide_path`：A 区正式路径。
- `/task_a/candidate_guide_path`：A 区候选路径，默认不使用，调试时可打开。

### 4.2 发布

```text
/move_base_simple/goal
/competition/guide_path
/task_a/final_goal
```

含义：

- `/move_base_simple/goal`：当前阶段终点。
- `/competition/guide_path`：当前阶段推荐路径，给 EGO 作为外部引导路径。
- `/task_a/final_goal`：当 A 区模块以 `target_ready:=false` 启动时，上层调度向 A 模块发送最终目标，触发 A 模块发布正式 `/task_a/guide_path`。

### 4.3 Action Client

```text
/gate_module/plan
```

Action 类型：

```cpp
across_gate_module/GatePlanAction
```

上层调度发送：

```cpp
GatePlanGoal goal;
goal.request_id = ++gate_request_id;
```

上层调度使用：

- `feedback.observation_pose`
- `result.success`
- `result.traversal_path`
- `result.error_code`
- `result.message`

## 5. 状态机设计

当前状态机：

```text
WAIT_READY
  ↓
TAKEOFF
  ↓
TASK_A_WAIT_PATH
  ↓
TASK_A_FLY
  ↓
TASK_B_SETUP
  ↓
TASK_B_FLY
  ↓
TASK_C_SETUP
  ↓
TASK_C_FLY
  ↓
TASK_D_PREPARE
  ↓
TASK_D_START_ACTION
  ↓
TASK_D_FLY_OBSERVATION
  ↓
TASK_D_TRAVERSE
  ↓
MISSION_DONE
```

异常时进入：

```text
MISSION_ABORTED
```

## 6. 各阶段逻辑

### WAIT_READY

等待：

- 收到 `/mavros/local_position/odom`
- 如果 `wait_for_offboard: true`，等待 `/mavros/state.mode == OFFBOARD`

满足后进入 `TAKEOFF`。

### TAKEOFF

发布从当前位置到起飞高度的路径：

```text
current_position → [current_x, current_y, takeoff_height]
```

到达 `takeoff_height` 后进入 `TASK_A_WAIT_PATH`。

### TASK_A_WAIT_PATH

上层调度向 A 模块发布：

```text
/task_a/final_goal
```

然后等待：

```text
/task_a/guide_path
```

收到 A 区路径后：

- 转发到 `/competition/guide_path`
- 将路径最后一个点发布到 `/move_base_simple/goal`
- 进入 `TASK_A_FLY`

### TASK_A_FLY

持续重发当前目标点。

当当前里程计距离 A 区终点小于 `task_a_reach_radius`，进入 `TASK_B_SETUP`。

### TASK_B_SETUP / TASK_B_FLY

B 区暂时没有独立模块，先用预设点占位。

如果：

```yaml
task_b_enabled: false
```

则直接跳到 `TASK_C_SETUP`。

如果开启：

```yaml
task_b_enabled: true
task_b_waypoints:
  - [x1, y1, z1]
  - [x2, y2, z2]
```

调度节点会发布：

```text
current_position → task_b_waypoints...
```

到达最后一个 B 点后进入 `TASK_C_SETUP`。

### TASK_C_SETUP / TASK_C_FLY

C 区由 EGO 负责避障，上层只提供入口/出口或中间引导点：

```yaml
task_c_waypoints:
  - [C入口x, C入口y, z]
  - [C出口x, C出口y, z]
```

上层发布：

```text
/competition/guide_path = current_position → C waypoints
/move_base_simple/goal = C 最后一个点
```

到达后进入 `TASK_D_PREPARE`。

### TASK_D_PREPARE

先飞到 D 区前方准备点：

```yaml
task_d_prepare_waypoints:
  - [D准备点x, D准备点y, z]
```

到达后进入 `TASK_D_START_ACTION`。

### TASK_D_START_ACTION

调用 D 区 Action：

```text
/gate_module/plan
```

发送 `GatePlanGoal` 后进入 `TASK_D_FLY_OBSERVATION`。

### TASK_D_FLY_OBSERVATION

D 区 gate module 的 Action 是两阶段流程：

```text
sendGoal
  ↓
第一次门框锁定
  ↓
feedback 返回 observation_pose
  ↓
上层让飞机飞到 observation_pose
  ↓
gate module 检测飞机已到达
  ↓
二次门框锁定
  ↓
result 返回 traversal_path
```

因此上层调度不能简单 `waitForResult()` 阻塞等待，必须在 feedback 中取 `observation_pose`，发布给 EGO。

收到 `observation_pose` 后：

```text
/competition/guide_path = current_position → observation_pose
/move_base_simple/goal = observation_pose
```

到达观察点后，继续等待 gate module 返回最终 `traversal_path`。

### TASK_D_TRAVERSE

收到 `result.traversal_path` 后：

```text
/competition/guide_path = result.traversal_path
/move_base_simple/goal = traversal_path 最后一个点
```

到达最后一个点后进入 `MISSION_DONE`。

### MISSION_DONE

保持最后一个目标点，飞机悬停。

后续可以扩展：

- 自动降落。
- 手动接管提示。
- Apriltag 降落。

## 7. 参数文件

默认参数位于：

```text
src/ego_single_goal/config/competition_mission.yaml
```

关键参数：

```yaml
frame_id: map
goal_topic: /move_base_simple/goal
guide_path_topic: /competition/guide_path

takeoff_enabled: true
takeoff_height: 1.0

task_a_goal_x: 3.120975971221924
task_a_goal_y: -5.956602573394775
task_a_goal_z: 1.0

task_b_enabled: false
task_b_waypoints: []

task_c_enabled: true
task_c_waypoints: []

task_d_enabled: true
gate_action_name: /gate_module/plan
task_d_prepare_waypoints:
  - [10.361700057983398, -5.367546081542969, 1.0]
```

正式比赛前必须现场更新：

- `task_b_waypoints`
- `task_c_waypoints`
- `task_d_prepare_waypoints`
- `task_a_goal_x/y/z`

## 8. 坐标系要求

所有发给 EGO 的 `PoseStamped` 和 `Path` 最终必须统一到：

```yaml
frame_id: map
```

如果某个模块输出 `camera_init`，调度节点会尝试通过 TF 转到 `map`。

如果没有 TF，调度节点会拒绝该路径并进入 `MISSION_ABORTED`。

正式比赛前必须检查：

```bash
rosrun tf tf_echo map camera_init
```

或者根据实际系统检查对应 TF。

## 9. 与旧节点的关系

旧节点：

```text
task_a_waypoint_manager.py
```

也会发布：

```text
/move_base_simple/goal
/competition/guide_path
```

使用新的 C++ 上层调度节点时，必须关闭旧 waypoint manager：

```bash
roslaunch ego_single_goal ego_single_goal_px4_interactive.launch \
  auto_detect:=true \
  target_ready:=false \
  auto_task_a_waypoints:=false
```

否则两个节点会同时发目标点，导致任务切换混乱。

## 10. 后续扩展

建议按优先级扩展：

1. B 区从预设点占位升级为感知路径模块。
2. C 区加入多段中间点或动态目标更新。
3. D 区失败时增加重试机制。
4. 增加 `/mission/state` 状态发布，方便地面站显示。
5. 增加自动降落状态：`LAND_PREPARE → LANDING → DISARMED`。
6. 增加安全超时，每个任务区超时后悬停或进入下一任务。
