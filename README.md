# MyWorks · 毫米波雷达速度辅助 FAST-LIO

这是一个基于 ROS Noetic 的毕业设计实验工程。研究目标是将毫米波雷达的多普勒自速度作为观测引入 FAST-LIO，评估它能否减小地下管廊、长直走廊等几何退化场景中的沿轴漂移。计划使用 TI IWR6843AOP 测速，以 Livox MID360 的激光点云和 IMU 运行 FAST-LIO。

## 工程结构

| 目录 | 用途 |
| --- | --- |
| [`src/FAST_LIO_GLOBAL`](src/FAST_LIO_GLOBAL) | 激光惯性里程计、点云建图以及可选的雷达速度更新 |
| [`src/reve`](src/reve) | 从带径向多普勒速度的点云估计雷达自身的三轴速度 |
| [`src/livox_ros_driver2`](src/livox_ros_driver2) | MID360 的 ROS 驱动和点云消息定义 |
| [`src/tof_velocity_guard`](src/tof_velocity_guard/README.md) | 接收 STM32 四路 ToF 测距，并根据距离限制速度指令 |
| `src/octomap_mapping`、`src/pointcloud_to_laserscan`、`src/pcd2pgm_package` | 地图转换与二维导航相关的辅助模块 |

## 构建

在 Ubuntu 20.04 和 ROS Noetic 环境中，准备 Catkin、C++ 和 Fortran 编译器、PCL、Eigen、Livox SDK2 与 `catkin_simple`。编译整个工作空间还需要 `tf2_sensor_msgs`、`octomap`、`octomap_msgs`、`octomap_ros` 等 ROS 依赖，具体依赖见各包的 `package.xml` 和 `CMakeLists.txt`。SDK2 和 Catkin 扩展需能被 CMake 找到。

```bash
git clone https://github.com/2552884055-ux/MyWorks.git
cd MyWorks
source /opt/ros/noetic/setup.bash
test -e src/CMakeLists.txt || catkin_init_workspace src
./build_ros1.sh
source devel/setup.bash
```

[`build_ros1.sh`](build_ros1.sh) 会在需要时从 ROS1 模板生成 Livox 的 `package.xml`，并调用 `catkin_make`。本地依赖目录、构建产物、运行日志和示例 bag 不随仓库提供。

## 分阶段运行

要查找或复现现有测试，从[项目测试索引](test/README.md)进入。

### 读取 STM32 四路测距

串口协议、限幅参数与两个节点的启动说明见[测距与速度限幅 README](src/tof_velocity_guard/README.md)。

STM32 串口默认使用 115200、8N1、无流控。节点校验带 CRC 的 `TOF1` 帧，再发布前、后、左、右四路距离。单路无效读数发布为 `NaN`，校验失败的帧整组丢弃。旧四整数固件可指定 `protocol:=legacy_csv`。

```bash
source devel/setup.bash
roslaunch tof_velocity_guard serial_range.launch port:=/dev/ttyUSB0
```

默认发布 `/tof/front`、`/tof/rear`、`/tof/left` 和 `/tof/right`，类型均为 `sensor_msgs/Range`。同组四路使用本机接收时间，默认坐标系依次为 `tof_front`、`tof_rear`、`tof_left`、`tof_right`。按实测设置 launch 的 `min_range`、`max_range` 和 `field_of_view`。若 USB 串口设备编号会变，可使用 `/dev/serial/by-id/` 下的设备路径。

```bash
rostopic echo /tof/front
rosbag record -O tof.bag /tof/front /tof/rear /tof/left /tof/right
```

### 单独验证 REVE 测速

从 [REVE 上游](https://github.com/christopherdoer/reve) 获取 `demo_ti_mmwave_rospkg_format.bag`，放入 `src/reve/demo_datasets/`。在已加载 `devel/setup.bash` 的终端启动 REVE，并在其他终端启动监听和回放。

```bash
# 终端 1
roslaunch radar_ego_velocity_estimator demo_ti_mmwave_ego_velocity.launch \
  mode:=ros do_plot:=false topic_twist:=/radar_velocity

# 终端 2
rostopic echo /radar_velocity

# 终端 3（项目根目录）
rosbag play src/reve/demo_datasets/demo_ti_mmwave_rospkg_format.bag
```

示例输入为 `/ti_mmwave/radar_scan_pcl` 和 `/sensor_platform/radar_right/trigger`。TI 点云适配要求 `PointCloud2` 包含 `x`、`y`、`z`、`intensity`、`velocity` 字段。实机需另行准备与固件兼容的采集驱动。如果驱动只发布点云，可设置 `run_without_trigger:=true`，并检查点云时间戳。

### 单独验证 FAST-LIO

在独立的 ROS Master 中，从项目根目录加载 MID360 参数，关闭毫米波速度融合，再直接运行主节点。这个启动方式绕开了旧建图 launch 对 `all_project` 的依赖。

```bash
# 终端 1
roscore

# 终端 2（项目根目录，已加载 devel/setup.bash）
mkdir -p src/FAST_LIO_GLOBAL/Log
rosparam load src/FAST_LIO_GLOBAL/config/mid360.yaml
rosparam set /radar/radar_en false
rosparam set /pcd_save/pcd_save_en false
rosparam set /publish/path_en true
rosrun fast_lio_global fastlio_mapping_global
```

在另一终端启动配置好网络参数的 MID360 驱动，或回放同步录制的 MID360 点云与 IMU 数据。默认输入是 `/livox/lidar`（`livox_ros_driver2/CustomMsg`）和 `/livox/imu`（`sensor_msgs/Imu`），可用 `rostopic hz /Odometry` 检查输出，用 RViz 查看 `/path` 和 `/cloud_registered`。

## 验证状态与接入限制

| 验证项 | 结果 |
| --- | --- |
| ROS Noetic 工作空间构建 | 整体编译通过 |
| REVE 的 TI 格式示例回放 | 412 帧点云得到 412 条速度，约 10.24 Hz。数据包无速度真值 |
| 纯 FAST-LIO | 合成房间数据中输出 117 帧里程计与配准点云 |
| REVE 到 FAST-LIO 的软件通信 | 临时消息转换下，FAST-LIO 收到全部 120 条合成雷达速度。该测试统计 ROS 接收量，未统计滤波器实际采纳的观测数 |

REVE 发布 `geometry_msgs/TwistWithCovarianceStamped`，当前 FAST-LIO 订阅 `geometry_msgs/TwistStamped`，同名话题也无法直连。通信测试使用仓库外的临时转换，转换时丢弃了协方差。正式接入还需统一消息类型和坐标系，核对多普勒正负号、雷达与 IMU 外参及观测时间戳。REVE 的 TI 适配会旋转点云坐标轴，当前速度消息仍沿用输入的 `frame_id`，标定时需特别核对。

IWR6843AOP 实机采集与真实管廊定位效果尚待验证。抗退化实验应在同一份同步采集数据上对比关闭和开启雷达融合时的轨迹误差与管廊轴向漂移。

## 上游项目与许可

- FAST-LIO2、IKFoM 和 ikd-Tree 的上游说明见 [FAST-LIO 文档](src/FAST_LIO_GLOBAL/README.fastlio-upstream.md)与 [许可证](src/FAST_LIO_GLOBAL/LICENSE)。
- REVE 由 Christopher Doer 开发，引用信息见 [REVE 文档](src/reve/README.md)与 [GPLv3 许可证](src/reve/LICENSE)。
- Livox 驱动的许可见 [Livox ROS Driver 2 许可证](src/livox_ros_driver2/LICENSE.txt)。
