#!/usr/bin/env python3
"""
sensor_health.py
=================
Live health monitor for one robot's raw sensor streams. Every 2 s prints,
per topic, the message rate and the largest gap between messages, plus the
encoder movement, gyro yaw rate and how full the LiDAR scans are.

Usage:
  python3 scripts/sensor_health.py            # robot2
  python3 scripts/sensor_health.py robot1

What healthy looks like (firmware publishes IMU/encoder at 50 Hz, scan at 10 Hz):
  - encoder / imu_raw well above 10 Hz, scan_raw near 10 Hz, max gap < 300 ms.
    All three dropping to ~1 Hz together means the ESP32 loop is stalling
    (Wi-Fi / reliable publish), not a sensor fault.
  - gyro z ~0.000 rad/s while the robot is still. A steady offset there
    rotates the EKF heading and ghosts the SLAM map.
  - encoder dL/dR both moving whenever the robot drives; one stuck at 0/1
    means that wheel's encoder channel is not counting.
  - scan finite readings well over 250 / 360.
"""

import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32MultiArray

WINDOW_S = 2.0


class SensorHealth(Node):

    def __init__(self, robot):
        super().__init__(f'sensor_health_{robot}')
        self.robot = robot
        self.times = {'encoder': [], 'imu_raw': [], 'scan_raw': []}
        self.encoders = []
        self.gyro_z = []
        self.scan_finite = []

        self.create_subscription(
            Int32MultiArray, f'/{robot}/encoder', self.encoder_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, f'/{robot}/imu_raw', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, f'/{robot}/scan_raw', self.scan_cb, qos_profile_sensor_data)
        self.create_timer(WINDOW_S, self.report)

    def encoder_cb(self, msg):
        self.times['encoder'].append(time.time())
        self.encoders.append((msg.data[0], msg.data[1]))

    def imu_cb(self, msg):
        self.times['imu_raw'].append(time.time())
        self.gyro_z.append(msg.angular_velocity.z)

    def scan_cb(self, msg):
        self.times['scan_raw'].append(time.time())
        self.scan_finite.append(sum(1 for r in msg.ranges if math.isfinite(r)))

    def report(self):
        parts = []
        for name, ts in self.times.items():
            gaps = [b - a for a, b in zip(ts, ts[1:])]
            max_gap = f'{max(gaps) * 1000:4.0f}ms' if gaps else '  -  '
            parts.append(f'{name} {len(ts) / WINDOW_S:5.1f}Hz gap {max_gap}')
        line = ' | '.join(parts)

        if len(self.encoders) >= 2:
            (l0, r0), (l1, r1) = self.encoders[0], self.encoders[-1]
            line += f' || enc L={l1} R={r1} dL={l1 - l0:+d} dR={r1 - r0:+d}'
        else:
            line += ' || enc -'

        if self.gyro_z:
            line += f' | gyro z {sum(self.gyro_z) / len(self.gyro_z):+.4f} rad/s'

        if self.scan_finite:
            line += f' | scan finite {min(self.scan_finite)}-{max(self.scan_finite)}/360'

        print(line, flush=True)

        for ts in self.times.values():
            ts.clear()
        self.encoders.clear()
        self.gyro_z.clear()
        self.scan_finite.clear()


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else 'robot2'
    rclpy.init()
    node = SensorHealth(robot)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
