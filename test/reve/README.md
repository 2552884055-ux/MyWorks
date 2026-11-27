# REVE 实时测速台架测试

[返回测试索引](../README.md)

验证 IWR6843AOP 点云进入 REVE 后，能输出三轴自身速度和线速度协方差。测试入口复用包内的实时 launch 和绘图脚本。

前置条件为已编译 MyWorks 的 ROS Noetic 工作区、通过 USB 接入且当前用户可访问串口的 IWR6843AOP，以及支持检测点和 SNR 输出的 TI TLV 固件。开启 RViz 和速度图时需要桌面环境。

在 MyWorks 根目录运行：

```bash
source devel/setup.bash
roslaunch test/reve/iwr6843aop_live.launch
```

无桌面时加 `rviz:=false plot:=false`。已有驱动发布点云时加 `start_driver:=false`，避免重复打开串口。按 Ctrl+C 结束测试。

| 话题 | 用途 |
| --- | --- |
| `/ti_mmwave/radar_scan_pcl` | TI 驱动输出的检测点云 |
| `/reve/velocity` | `geometry_msgs/TwistWithCovarianceStamped`，坐标系为 `radar_reve` |

可用 `rostopic hz /ti_mmwave/radar_scan_pcl`、`rostopic hz /reve/velocity` 和 `rostopic echo -n 1 /reve/velocity` 检查输入输出。

通过标准：

- 点云持续到达，场景有足够静态反射时能收到时间戳前进的速度消息，线速度及左上角 3×3 协方差为有限值。
- 雷达静止时速度接近零。保持姿态并沿雷达前后移动时，Y 轴响应为主，反向移动时符号翻转。
- 默认 CFG 的点云目标帧率为 10 Hz。REVE 解算失败时会跳过速度发布，因此速度频率可低于点云频率。本测试验证数据链路及速度方向，精度和协方差标定需另做参考实验。
