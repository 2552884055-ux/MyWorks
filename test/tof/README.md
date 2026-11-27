# ToF 测距与速度限幅

[返回测试索引](../README.md)

### 台架复现

[台架启动文件](bench_speed_limits.launch) 一起启动 STM32 采集、独立限幅节点和两张实时图。第一张的横轴是时间，第二张是实测距离。在项目根目录运行：

```bash
source devel/setup.bash
roslaunch test/tof/bench_speed_limits.launch port:=/dev/ttyUSB0
```

已有 `/tof/front` 等四路测距话题时，传入 `start_serial:=false`，避免重复打开串口。没有桌面显示时传入 `show_plots:=false`。可用 `stop_distance`、`slow_distance`、`max_horizontal_speed` 和 `range_timeout` 调整台架参数。已有同名台架时可用 `bench_namespace:=tof_bench_2` 单独启动另一组。

两张图只需要实时测距，不需要发测试指令。需要检查实际限幅输出时，在另一终端用以下命令请求前向 `0.3 m/s`，结束测试按 `Ctrl+C`。测试输入和输出都在 `/tof_bench` 下，启动文件不启动 EGO-Planner 或飞控接口。

```bash
rostopic pub -s -r 20 /tof_bench/test_velocity geometry_msgs/TwistStamped \
  '{header: {stamp: now, frame_id: body}, twist: {linear: {x: 0.3, y: 0.0, z: 0.0}}}'
```

可用 `rostopic echo /tof_bench/limited_velocity` 查看限幅后的速度。

### 通过标准

- 串口接通后，两张图都能收到前、后、左、右四路数据。
- 移动单个方向的挡板，对应速度上限在默认 0.35 m 以下为零，0.35–0.9 m 之间逐渐增加，达到 0.9 m 时为 0.3 m/s。
- 测距无效或超时后，对应速度上限归零。使用测试指令时，在 `/tof_bench/limited_velocity` 检查实际输出。
