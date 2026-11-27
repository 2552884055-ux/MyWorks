#!/usr/bin/env python3
"""Display live REVE velocity estimates in a rolling 20-second plot."""
from collections import deque
import math
import threading
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped


def main():
    rospy.init_node("live_velocity_plot", disable_signals=True)
    samples = deque(maxlen=2000)
    lock = threading.Lock()
    start_stamp = None
    last_received = None
    frame_id = ""

    def receive(message):
        nonlocal start_stamp, last_received, frame_id
        velocity = message.twist.twist.linear
        stamp = message.header.stamp.to_sec()
        values = (velocity.x, velocity.y, velocity.z)
        if not all(math.isfinite(value) for value in (stamp, *values)):
            return
        with lock:
            if not samples or stamp < samples[-1][0]:
                samples.clear()
                start_stamp = stamp
            if samples and stamp - samples[-1][0] > .25:
                samples.append((samples[-1][0] + 1e-6, math.nan, math.nan, math.nan))
            samples.append((stamp, *values))
            last_received = time.monotonic()
            frame_id = message.header.frame_id

    topic = rospy.get_param("~topic", "/reve/velocity")
    subscriber = rospy.Subscriber(topic, TwistWithCovarianceStamped, receive, queue_size=100)
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    figure.canvas.manager.set_window_title("REVE live velocity")
    lines = []
    value_labels = []
    for axis, component, direction, color in zip(
        axes, "xyz", ("right", "forward", "up"),
        ("tab:blue", "tab:orange", "tab:green")
    ):
        line, = axis.plot([], [], color=color, linewidth=1.5)
        lines.append(line)
        value_labels.append(axis.text(
            .98, .88, "Waiting", transform=axis.transAxes,
            ha="right", va="top", color=color, fontsize=13
        ))
        axis.set_ylabel("v_{} {} [m/s]".format(component, direction))
        axis.set_ylim(-.2, .2)
        axis.grid(True, alpha=.3)
    axes[-1].set_xlim(0, 20)
    axes[-1].set_xlabel("Time since first estimate [s]")
    heading = figure.suptitle("Waiting for " + topic)
    figure.tight_layout(rect=(0, 0, 1, .95))

    def update(_):
        if rospy.is_shutdown():
            plt.close(figure)
            return
        with lock:
            data = list(samples)
            origin = start_stamp
            received = last_received
            frame = frame_id
        if not data:
            return
        elapsed = data[-1][0] - origin
        left = max(0, elapsed - 20)
        visible = [row for row in data if row[0] - origin >= left]
        times = [row[0] - origin for row in visible]
        for index, (axis, line, label) in enumerate(zip(axes, lines, value_labels)):
            values = [row[index + 1] for row in visible]
            line.set_data(times, values)
            limit = max(.2, max(abs(value) for value in values if math.isfinite(value)) * 1.15)
            axis.set_ylim(-limit, limit)
            label.set_text("{:+.3f} m/s".format(data[-1][index + 1]))
        axes[-1].set_xlim(left, max(20, elapsed))
        age = time.monotonic() - received
        state = "LIVE" if age < 1 else "No new estimate for {:.1f} s".format(age)
        heading.set_text("REVE | {} | frame {} | {}".format(state, frame, topic))
        figure.canvas.manager.set_window_title(
            "REVE | {} | vx={:+.3f} vy={:+.3f} vz={:+.3f} m/s".format(
                state, *data[-1][1:]
            )
        )

    animation = FuncAnimation(figure, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        subscriber.unregister()
        rospy.signal_shutdown("Velocity plot closed")


if __name__ == "__main__":
    main()
