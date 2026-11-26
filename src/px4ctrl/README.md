# Fast-Drone-250 姿态与推力接口

控制器来自 [Fast-Drone-250 `8ff6427`](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/8ff64274b161bb829653faf5946656658e7e2c9f/src/realflight_modules/px4ctrl)，保留上游 GPLv3 许可。MyWorks 沿用现有 EGO 的 `PositionCommand` 格式。上游控制律本身不使用 jerk，输入适配将该项设为零。

先启动 FAST-LIO。在工作区根目录构建并加载 ROS 环境。如果 MAVROS 安装在工作区的本地依赖目录，再加载其中的环境脚本。

```bash
./build_ros1.sh
source devel/setup.bash
source .build_deps/mavros-root/setup-local.bash
roslaunch px4ctrl run_with_fastlio.launch fcu_url:=/dev/ttyACM0:57600 port:=/dev/ttyUSB0
```

串口路径和波特率按实际设备填写。已有 MAVROS 时可以省略 `fcu_url`，已有 ToF 测距话题时可以传 `start_serial:=false`。这个 launch 同时启动 EGO、ToF 测距和 `px4ctrl`。姿态模式使用 `/mavros/setpoint_raw/attitude`，同一架飞控在运行此 launch 时应只使用这一套 Offboard 指令来源。

`px4ctrl` 订阅 EGO 的 `/position_cmd`、FAST-LIO 的 `/Odometry`、四路测距以及 MAVROS 的 IMU 和飞控状态。轨迹跟踪状态下，ToF 将位置反馈与速度前馈合成的水平速度限制在当前可用区间，再限制反馈与加速度前馈合成的水平加速度。前方受阻而仍向前移动时，反向制动仍可用。竖直和航向由 `px4ctrl` 的上游控制律处理。

每个方向的速度上限以 S 形曲线从 `stop_distance` 处的零过渡到 `slow_distance` 处的全速。曲线在两个端点斜率为零。测距无效或进入停止区时，该方向的上限直接为零。

每个方向还根据刹停模型计算速度上限。可用距离是 ToF 测距减去 `stop_distance`，刹停距离按 `v × 延迟 + v² / (2 × 制动减速度)` 计算。延迟包含 `braking_delay` 和走到控制环时该测距消息的年龄。控制器取 S 形上限和刹停上限中的较小值。实测接近速度达到刹停上限时，该方向目标速度归零，剩余水平加速度预算优先用于制动。

`braking_deceleration` 和 `braking_delay` 默认分别为 `0.3 m/s²` 和 `0.3 s`，是回放和台架的初始参数。部署时使用实测可用减速度和响应延迟。模型使用的每轴减速度最多为水平加速度上限的 `1/√2`，给两个方向同时制动留出预算。

联合 launch 禁用上游的自动起飞、自动降落和自动解锁，遥控器状态机保留。EGO 目标点在控制器进入 `AUTO_HOVER` 后发送。默认参数文件来自上游机型，实体测试前通过 `ctrl_config` 加载适合当前机架的参数，尤其是悬停推力。ToF 默认限速 `0.3 m/s`、限加速度 `0.6 m/s²`。可在 `/debugPx4ctrl` 查看跟踪时的限幅速度和加速度。

当前 MID360 回放中的 FAST-LIO 里程计约为 10 Hz，低于上游 `px4ctrl` 的 100 Hz 频率提示阈值。回放验证覆盖 ROS 指令链路，实体机架的跟踪稳定性需要单独验证。
