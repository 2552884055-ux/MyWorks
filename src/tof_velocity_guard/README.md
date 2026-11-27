# ToF 四路测距与速度限幅

STM32 采集前、后、左、右四个 VL53L0X 的距离，通过串口发给本机。ROS1 C++ 节点 `tof` 把距离转换成米，发布四路 `sensor_msgs/Range`。限幅节点也使用 C++，两个节点可分别启动。

## STM32 串口格式

- 默认波特率 `115200`，8 个数据位、无校验、1 个停止位，无流控。
- 每组数据占一行，四路顺序为前、后、左、右。每路包含一个毫米整数和一个状态，状态 `0` 表示有效，`1` 至 `3` 表示无效并发布为 `NaN`。
- 用换行字节 `\n` 或 `\r\n` 结束一行。序号和 STM32 采样完成时间（开机后毫秒数）是 `uint32`，ROS 消息使用本机接收时间。

```text
$TOF1,<seq>,<t_ms>,<front_mm>,<front_status>,<rear_mm>,<rear_status>,<left_mm>,<left_status>,<right_mm>,<right_status>*<CRC16>
$TOF1,123,456789,420,0,710,0,0,3,980,0*8CA2
```

CRC-16/CCITT-FALSE 对 `$` 和 `*` 之间的字节计算，初值 `0xFFFF`、多项式 `0x1021`。格式错误或校验失败时丢弃整帧。示例输出前 `0.420 m`、后 `0.710 m`、左 `NaN`、右 `0.980 m`。

旧固件若仍发送 `420,710,-1,980\r\n` 形式的四整数 CSV，启动时指定 `protocol:=legacy_csv`。

## 启动

在工作区根目录运行，按实际设备修改串口路径。如果 USB 设备提供 `/dev/serial/by-id/` 链接，可以用该路径来避免插拔后编号改变。

```bash
source /opt/ros/noetic/setup.bash
./build_ros1.sh --pkg tof_velocity_guard
source devel/setup.bash
roslaunch tof_velocity_guard serial_range.launch port:=/dev/ttyUSB0
```

| 启动参数 | 默认值 | 用途 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 本机串口设备 |
| `baud` | `115200` | 需与 STM32 设置一致 |
| `protocol` | `tof1` | `tof1` 带 CRC，`legacy_csv` 是旧四整数格式 |
| `min_range` | `0.03` | 最小有效距离，单位 m |
| `max_range` | `2.0` | 本机接受的最大有效距离，单位 m |
| `field_of_view` | `0.44` | 消息中的视场角，单位 rad |

量程和视场角参数请按实测设置。若节点一直报串口打开失败，先检查设备路径和当前用户的读写权限。

## 输出与录包

| 方向 | ROS 话题 | 默认 `header.frame_id` |
| --- | --- | --- |
| 前 | `/tof/front` | `tof_front` |
| 后 | `/tof/rear` | `tof_rear` |
| 左 | `/tof/left` | `tof_left` |
| 右 | `/tof/right` | `tof_right` |

同一串口帧的四条消息使用同一个本机接收时间 `header.stamp`。消费者应检查 `range` 是有限数值、位于 `min_range` 和 `max_range` 之间，并按时间戳判断是否超时。格式错误的串口行会被丢弃，串口断开时节点会尝试重连。

```bash
rostopic echo -n 1 /tof/front
rostopic hz /tof/front
rosbag record -O tof.bag /tof/front /tof/rear /tof/left /tof/right
```

做隧道联采时，把这四个话题加进同一条 `rosbag record` 命令，和激光点云、IMU、毫米波数据录在一起。

## 联合启动 FAST-LIO、EGO 与 ToF

在同一个 `MyWorks` 工作区编译后，先启动 FAST-LIO，再从工作区根目录运行下面的命令。它启动 EGO、串口测距和速度限幅。如果已有节点发布四路 `/tof/*` 测距话题，传入 `start_serial:=false`。

```bash
source devel/setup.bash
roslaunch tof_velocity_guard run_with_ego.launch port:=/dev/ttyUSB0
```

EGO 将 `camera_init` 坐标系下的跟踪指令发到 `/position_cmd`。限幅节点结合 `/Odometry` 和 TF，将 EGO 的速度前馈与水平位置反馈合并，再按前、后、左、右的测距限制机体系速度。加速度由限幅后速度的跟踪误差与 EGO 的加速度前馈共同计算，接近障碍时压低朝它加速的上限，并保留制动。

- `/tof/limited_velocity` 使用 `geometry_msgs/TwistStamped`。
- `/tof/limited_acceleration` 使用 `geometry_msgs/AccelStamped`，只提供水平分量。
- 两条输出都使用 `body` 坐标系，同一轮计算共享时间戳。里程计、规划指令或所需 TF 失效时，两个输出归零。
- 水平速度和加速度上限分别通过 `max_horizontal_speed` 和 `max_horizontal_acceleration` 调整，默认分别为 `0.3 m/s` 和 `0.6 m/s²`。位置和速度反馈增益分别通过 `position_gain` 和 `velocity_gain` 调整。

飞控桥接接收限幅结果后，限幅才会作用到执行端。速度模式可使用限幅后的速度目标。若下游接收加速度指令，可使用限幅后的加速度。该值已经包含 ROS 侧的速度反馈，接入时需明确速度环由哪一级闭合。EGO 原始位置已用于 ROS 侧的位置反馈。

## 与 PX4 交换数据

在 ROS Noetic 环境准备 MAVROS 和其 GeographicLib 数据集。按实物连接填写 MAVROS 的 `fcu_url`，串口连接使用设备路径和飞控端配置的波特率，例如：

```bash
source devel/setup.bash
roslaunch tof_velocity_guard px4_velocity.launch fcu_url:=/dev/ttyACM0:57600
```

该 launch 启动 MAVROS。速度插件订阅 `/tof/limited_velocity` 并以 `BODY_NED` 向 PX4 发送目标速度。MAVROS 负责把 ROS 的机体系分量转换为 MAVLink 约定。PX4 回传的连接状态和本地里程计可从 `/mavros/state` 和 `/mavros/local_position/odom` 查看。该接口仅启动通信，解锁和飞行模式由飞控操作端管理。

PX4 速度环使用限幅后的速度目标，`/tof/limited_acceleration` 可用于记录和分析。当前竖直速度默认是零。监听两路数据可以检查发送和接收：

```bash
rostopic echo -n 1 /tof/limited_velocity
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/odom
```

## 按四路距离限制速度指令

### 台架复现

见[项目测试目录中的 ToF 台架说明](../../test/tof/README.md)，包含一键启动命令和通过标准。

### 单独启动与话题

限幅节点的私有话题 `~directional_speed_limits` 发布四个方向各自允许的速度，单位为 m/s，顺序为前、后、左、右。有测距数据即可观察这些上限，实际速度指令仍由规划输入决定。桌面上可用以下命令打开实时曲线：

```bash
rosrun tof_velocity_guard plot_speed_limits.py
```

如果限幅节点在命名空间内，使用 `--topic` 指定完整的 `directional_speed_limits` 话题。

用 `rosrun tof_velocity_guard plot_speed_limits.py --against-distance` 可改用测距作横轴。此模式订阅 `~directional_range_speed`，每组发布前、后、左、右四对 `[距离(m), 上限(m/s)]`；无效距离为 `NaN`，不会绘成有效测距点。虚线是按限幅参数画的参考，实点是最近 15 秒的实测样本。

先启动测距节点，再启动限幅节点。EGO-Planner 的 `PositionCommand` 需要在编译时找到 `quadrotor_msgs`。如果它在另一个工作区，先编译该工作区，再用下面的命令构建本包。把示例中的路径改为实际包含 `quadrotor_msgsConfig.cmake` 的目录。

```bash
./build_ros1.sh --pkg tof_velocity_guard --force-cmake \
  -Dquadrotor_msgs_DIR=/path/to/ego_ws/devel/share/quadrotor_msgs/cmake
source devel/setup.bash
```

没有该消息包时，测距节点仍可构建，限幅节点可用 `input_type:=twist_stamped` 启动。

```bash
roslaunch tof_velocity_guard velocity_limiter.launch
```

- 默认订阅 `/position_cmd`（`quadrotor_msgs/PositionCommand`），读取就绪轨迹的 `velocity` 字段。可用 `input_topic:=...` 更换话题。已有 `geometry_msgs/TwistStamped` 速度指令时，可设置 `input_type:=twist_stamped`。
- 指令有坐标系和时间戳才会使用。节点用最新 TF 将速度旋转到 `body_frame`（默认 `body`）；如果指令已经在该坐标系下，则直接使用。EGO-Planner 默认用 `world`，需提供实际的 `world` 到机体系的 TF。
- 机体系 `+X` 为前、`-X` 为后、`+Y` 为左、`-Y` 为右，四个测距探头应与这些方向对应。从 `stop_distance` 到 `slow_distance` 线性增加各方向允许速度。小于等于停止距离，或该方向测距缺失、超时、读取失败或超量程时，朝该方向的速度为零。
- `/tof/limited_velocity` 以 30 Hz 发布机体系 `geometry_msgs/TwistStamped`。输出只包含线速度。指令超时或所需 TF 不可用、过期时，输出零速度。竖直速度默认限制为零，因为四个探头没有覆盖上下。

| 启动参数 | 默认值 |
| --- | --- |
| `stop_distance` | `0.35 m` |
| `slow_distance` | `0.9 m` |
| `max_horizontal_speed` | `0.3 m/s`，水平速度向量的大小上限 |
| `max_vertical_speed` | `0.0 m/s`，竖直速度绝对值的上限 |
| `range_timeout`、`command_timeout` | 各 `0.2 s` |
| `tf_timeout` | `0.15 s`，动态 TF 的最大年龄 |

先在台架用 `rostopic echo /tof/limited_velocity` 检查各方向，再按探头安装位置、实测量程和飞控制动表现设置阈值。这个节点只发布速度指令，没有接入飞控。如果现有控制器继续直接跟踪原始 `PositionCommand` 里的位置和加速度，这个速度输出无法限制它。

```bash
rosbag record -O limiter.bag /position_cmd /tof/limited_velocity \
  /tof/front /tof/rear /tof/left /tof/right /tf /tf_static
```
