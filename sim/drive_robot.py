#!/usr/bin/env python3
"""Drive the robot along a scripted pattern to generate rich sensor data.

Two modes (ROS 2 parameter ``mode``):

* ``pattern`` (default): straight -> turn left -> straight -> turn right, ...
* ``straight``:         strict straight-line test — angular velocity is
                        HARD-constrained to 0.0 for the whole run, so the
                        commanded trajectory is provably a straight line.
                        Use this for the pure straight-line KF validation.

Example (strict straight line, 0.4 m/s for 30 s):
    ros2 run ... / python3 drive_robot.py --ros-args \
        -p mode:=straight -p straight_speed:=0.4 -p straight_duration:=30.0
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

SEGMENTS = [
    # (linear m/s, angular rad/s, duration s)
    (0.4, 0.0, 5.0),
    (0.25, 0.6, 5.0),
    (0.4, 0.0, 5.0),
    (0.25, -0.6, 5.0),
    (0.0, 0.9, 3.0),
    (0.45, 0.0, 6.0),
    (0.3, -0.5, 4.0),
    (0.45, 0.0, 6.0),
]


class Driver(Node):

    def __init__(self):
        super().__init__('scripted_driver')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ---- Motion constraint parameters ----
        # mode: 'pattern' (mixed straight/turn) or 'straight' (strict line)
        self.declare_parameter('mode', 'pattern')
        # Strict straight-line test parameters
        self.declare_parameter('straight_speed', 0.4)      # m/s, constant
        self.declare_parameter('straight_duration', 30.0)  # seconds

    def _publish_cmd(self, v, w, enforce_straight=False):
        """Publish one cmd_vel message.

        When ``enforce_straight`` is True the angular rate is hard-clamped to
        exactly 0.0 — this is the software-level motion constraint that
        guarantees the straight-line test never commands a turn.
        """
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = 0.0 if enforce_straight else float(w)
        self.pub.publish(msg)

    def run(self):
        mode = self.get_parameter('mode').value

        if mode == 'straight':
            # ---- Strict straight-line constraint ----
            speed = float(self.get_parameter('straight_speed').value)
            duration = float(self.get_parameter('straight_duration').value)
            self.get_logger().info(
                f'STRAIGHT-LINE TEST: v={speed} m/s, w=0.0 (hard constrained), '
                f'duration={duration}s')
            t0 = time.time()
            while time.time() - t0 < duration:
                # enforce_straight=True -> angular velocity can never be non-zero
                self._publish_cmd(speed, 0.0, enforce_straight=True)
                time.sleep(0.05)
        else:
            # ---- Original mixed pattern ----
            for v, w, dur in SEGMENTS:
                self.get_logger().info(f'segment v={v} w={w} for {dur}s')
                t0 = time.time()
                while time.time() - t0 < dur:
                    self._publish_cmd(v, w, enforce_straight=False)
                    time.sleep(0.05)

        # stop
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.05)
        self.get_logger().info('drive pattern finished')


def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
