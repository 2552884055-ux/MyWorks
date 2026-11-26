#!/usr/bin/env python3
from collections import deque
import threading

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped

samples = deque(maxlen=1200)
lock = threading.Lock()

def receive(message):
    velocity = message.twist.twist.linear
    sample = (message.header.stamp.to_sec(), velocity.x, velocity.y, velocity.z)
    with lock:
        if samples and sample[0] < samples[-1][0]:
            samples.clear()
        samples.append(sample)

rospy.init_node("live_velocity_plot", disable_signals=True)
subscriber = rospy.Subscriber(
    "/radar_body_velocity_estimation_node/twist_body",
    TwistWithCovarianceStamped, receive, queue_size=100,
)
figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
figure.canvas.manager.set_window_title("Live radar body velocity")
lines = []
for axis, component, color in zip(axes, "xyz", ("tab:blue", "tab:orange", "tab:green")):
    line, = axis.plot([], [], color=color, linewidth=1.5)
    lines.append(line)
    axis.set_ylabel("v_" + component + " [m/s]")
    axis.grid(True)
    axis.set_ylim(-1, 1)
axes[-1].set_xlabel("Time since start of replay [s]")
figure.suptitle("Live radar body velocity — waiting for data")
figure.tight_layout()

def update(_):
    with lock:
        data = list(samples)
    if not data:
        return lines
    times = [row[0] - data[0][0] for row in data]
    for index, (axis, line) in enumerate(zip(axes, lines)):
        values = [row[index + 1] for row in data]
        line.set_data(times, values)
        limit = max(0.2, max(abs(value) for value in values) * 1.15)
        axis.set_ylim(-limit, limit)
    axes[-1].set_xlim(max(0, times[-1] - 20), max(20, times[-1]))
    figure.suptitle("Live radar body velocity | {:.1f} s | {:.3f}, {:.3f}, {:.3f} m/s".format(
        times[-1], *data[-1][1:]))
    return lines

animation = FuncAnimation(figure, update, interval=100, cache_frame_data=False)
plt.show()
rospy.signal_shutdown("Plot closed")
