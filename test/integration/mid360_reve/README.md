# Mid-360 与 REVE 同步采集

[返回测试索引](../../README.md)

此测试同时启动 Mid-360 激光和内置 IMU、IWR6843AOP 驱动、REVE、纯 FAST-LIO 以及 rosbag 录制。里程计来自雷达融合关闭的 FAST-LIO，用于离线检查外参和时差。

前置条件为已编译的 MyWorks 工作区、已连接且网络配置匹配的 Mid-360，以及可访问 TI 驱动配置中两个串口的 IWR6843AOP。当前 Mid-360 配置要求主机网口地址为 `192.168.1.50`，设备地址为 `192.168.1.144`。启动前关闭占用同一设备的驱动。

```bash
cd /home/orangepi/MyWorks && source devel/setup.bash && mkdir -p logs/reve && \
roslaunch --sigint-timeout 45 test/integration/mid360_reve/record_alignment.launch \
  bag_path:="$PWD/logs/reve/alignment_$(date +%Y%m%d_%H%M%S).bag"
```

默认关闭图形窗口。可追加 `plot:=true` 打开 REVE 速度图，或用 `radar_rviz:=true` 显示毫米波点云。

装在同一刚性支架上的设备先静止约 10 秒，再做前后、左右往返和加减速，最后转动。按 Ctrl+C 后等待 rosbag 完成写盘。采集结果在 `logs/reve/`。

| 录制话题 | 内容 |
| --- | --- |
| `/livox/lidar` | Mid-360 激光点云及点内时间 |
| `/livox/imu` | Mid-360 内置 IMU |
| `/ti_mmwave/radar_scan_pcl` | 毫米波检测点云 |
| `/reve/velocity` | REVE 速度和协方差 |
| `/Odometry` | 纯 FAST-LIO 里程计 |

通过标准是 bag 正常收尾，`rosbag info` 显示五个话题都有消息，采集覆盖静止与运动段。时间偏差和外参需要用录制结果离线验证。
