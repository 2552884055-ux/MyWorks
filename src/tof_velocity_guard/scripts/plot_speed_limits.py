#!/usr/bin/env python3
"""Display the directional speed limits reported by the ROS limiter."""

import argparse
from collections import deque
from math import isfinite
from pathlib import Path
from threading import Lock
import time

import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray


DIRECTIONS = ("Front (+X)", "Rear (-X)", "Left (+Y)", "Right (-Y)")
COLORS = ("#127562", "#b45718", "#306aba", "#9256a4")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", help="Override the default ROS diagnostic topic.")
    parser.add_argument("--against-distance", action="store_true", help="Plot limits against measured distance.")
    parser.add_argument("--snapshot", type=Path, help="Save a PNG after five seconds of live data.")
    args = parser.parse_args(rospy.myargv()[1:])
    distance_mode = args.against_distance
    topic = args.topic or ("/tof_velocity_limiter/directional_range_speed" if distance_mode
                           else "/tof_velocity_limiter/directional_speed_limits")
    expected_fields = len(DIRECTIONS) * (2 if distance_mode else 1)
    rospy.init_node("tof_speed_limit_plot", anonymous=True)

    history = deque(maxlen=600)
    lock = Lock()

    def receive(message):
        if len(message.data) == expected_fields:
            with lock:
                history.append((time.monotonic(), *message.data))

    subscriber = rospy.Subscriber(topic, Float64MultiArray, receive, queue_size=1)
    parent = topic.rsplit("/", 1)[0]
    max_speed = rospy.get_param(parent + "/max_horizontal_speed", 0.3)
    if distance_mode:
        stop = rospy.get_param(parent + "/stop_distance", .35)
        slow = rospy.get_param(parent + "/slow_distance", .9)
        max_distance = max(rospy.get_param("/tof/max_range", 2.0), slow)

    figure, axes = plt.subplots(2, 2, sharex=True, sharey=True, figsize=(10, 5.8))
    title = "ToF · Speed limit vs distance" if distance_mode else "ToF · Live directional speed limits"
    figure.canvas.manager.set_window_title(title)
    figure.suptitle(title, fontsize=17, fontweight="bold", y=.97)
    status = figure.text(.5, .905, "Waiting for ROS data", ha="center", color="#465160")
    figure.subplots_adjust(left=.10, right=.97, bottom=.13, top=.82, hspace=.36, wspace=.16)
    figure.text(.5, .045, "Measured distance (m)" if distance_mode else "Time relative to now (s)", ha="center")
    figure.text(.045, .49, "Speed limit (m/s)", rotation=90, va="center")
    window = 15.0
    lines, readouts, latest_markers = [], [], []
    for axis, direction, color in zip(axes.flat, DIRECTIONS, COLORS):
        axis.set_title(direction, loc="left", fontsize=12, fontweight="bold")
        axis.set_xlim((0, max_distance) if distance_mode else (-window, 0))
        axis.set_ylim(-.02 * max(.1, max_speed), 1.12 * max(.1, max_speed))
        axis.grid(alpha=.23)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        if distance_mode:
            axis.plot([0, stop, slow, max_distance], [0, 0, max_speed, max_speed],
                      color="#778495", linestyle="--", linewidth=1.2, alpha=.65)
            line, = axis.plot([], [], ".", color=color, markersize=4, alpha=.35)
            latest, = axis.plot([], [], "o", color=color, markersize=7, zorder=4)
            latest_markers.append(latest)
        else:
            line, = axis.plot([], [], color=color, linewidth=2)
        lines.append(line)
        readouts.append(axis.text(.97, .83, "—", transform=axis.transAxes, ha="right", color=color))

    first_sample = None
    snapshot = args.snapshot

    def update():
        nonlocal first_sample, snapshot
        if rospy.is_shutdown():
            plt.close(figure)
            return
        now = time.monotonic()
        with lock:
            points = [sample for sample in history if now - sample[0] <= window]
        times = [sample[0] - now for sample in points]
        fresh = bool(points) and now - points[-1][0] < .5
        if fresh and first_sample is None:
            first_sample = points[0][0]
        live_label = "● Measured samples   ·   Dashed: configured limit" if distance_mode else "● Live ROS data"
        status.set_text(live_label if fresh else "Waiting for fresh ROS data")
        for i, (line, readout) in enumerate(zip(lines, readouts)):
            if distance_mode:
                column = 1 + 2 * i
                valid = [(s[column], s[column + 1]) for s in points if isfinite(s[column])]
                line.set_data(*zip(*valid) if valid else ([], []))
                latest = points[-1][column:column + 2] if fresh else (float("nan"), 0.0)
                distance, cap = latest
                latest_markers[i].set_data(*(([distance], [cap]) if isfinite(distance) else ([], [])))
                readout.set_text(f"{distance:.3f} m · {cap:.3f} m/s" if isfinite(distance)
                                 else ("Invalid range" if fresh else "—"))
            else:
                line.set_data(times, [sample[i + 1] for sample in points])
                readout.set_text(f"{points[-1][i + 1]:.3f} m/s" if fresh else "—")
        figure.canvas.draw_idle()
        if snapshot is not None and fresh and now - first_sample >= 5:
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(snapshot, dpi=155)
            print(f"Saved {snapshot}", flush=True)
            snapshot = None

    timer = figure.canvas.new_timer(interval=100)
    timer.add_callback(update)
    timer.start()
    try:
        plt.show()
    finally:
        timer.stop()
        subscriber.unregister()
        rospy.signal_shutdown("Plot closed")


if __name__ == "__main__":
    main()
