#!/usr/bin/env bash
set -eo pipefail
source /tmp/reve-demo.qZpcve/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11342
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export ROS_HOME=/home/orangepi/reve/build/live-demo/ros
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
demo_children=()
cleanup() {
    trap - EXIT INT TERM
    for demo_pid in "${demo_children[@]}"; do kill -INT "$demo_pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT INT TERM
roslaunch -p 11342 radar_ego_velocity_estimator demo_rio_body_velocity.launch mode:=ros do_plot:=False > /home/orangepi/reve/build/live-demo/estimator.log 2>&1 &
demo_children+=("$!")
for attempt in {1..30}; do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep 0.3
done
rviz -d /home/orangepi/reve/build/live-demo/radar.rviz > /home/orangepi/reve/build/live-demo/rviz.log 2>&1 &
demo_children+=("$!")
python3 /home/orangepi/reve/build/live-demo/live_velocity.py > /home/orangepi/reve/build/live-demo/plot.log 2>&1 &
demo_children+=("$!")
sleep 4
rosbag play --loop --clock /home/orangepi/reve/demo_datasets/demo_rio_format.bag > /home/orangepi/reve/build/live-demo/playback.log 2>&1 &
demo_children+=("$!")
wait
