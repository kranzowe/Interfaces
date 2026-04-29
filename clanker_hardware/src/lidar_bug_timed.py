#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from lidar_bug import LidarBugNode


class TimedLidarBugNode(LidarBugNode):
    def __init__(self):
        super().__init__()

        self.declare_parameter("run_duration", 10.0)
        self.declare_parameter("stop_speed", 1500.0)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("start_delay", 0.0)
        self.declare_parameter("odom_start_delay", 0.0)
        self.declare_parameter("require_odom", False)

        self.node_start_time = self.get_ros_time_as_double()
        self.drive_start_time = None
        self.start_odom_xy = None
        self.latest_odom_xy = None
        self.stop_logged = False
        self.waiting_for_odom_logged = False
        self.run_complete = False
        self._load_timed_params()
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

    def _load_timed_params(self):
        self.run_duration = max(
            0.0,
            float(self.get_parameter("run_duration").value),
        )
        self.stop_speed = float(self.get_parameter("stop_speed").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        start_delay = float(self.get_parameter("start_delay").value)
        legacy_odom_start_delay = float(
            self.get_parameter("odom_start_delay").value
        )
        if start_delay == 0.0 and legacy_odom_start_delay > 0.0:
            start_delay = legacy_odom_start_delay
        self.start_delay = max(0.0, start_delay)
        self.require_odom = bool(self.get_parameter("require_odom").value)

    def odom_cb(self, msg):
        self.latest_odom_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def update_param(self):
        super().update_param()
        self._load_timed_params()

    def pub_cb(self):
        if self.previous_time == 0.0:
            self._publish_stop_command()
            return

        now = self.get_ros_time_as_double()
        if self.drive_start_time is None:
            if now - self.node_start_time < self.start_delay:
                self._publish_stop_command()
                return

            if self.require_odom and self.latest_odom_xy is None:
                self._publish_stop_command()
                if self.waiting_for_odom_logged:
                    return

                self.get_logger().info(
                    "Waiting for first odom message before starting timed run..."
                )
                self.waiting_for_odom_logged = True
                return

            self.drive_start_time = now
            self.start_odom_xy = self.latest_odom_xy
            self.get_logger().info(
                f"Timed lidar drive started for {self.run_duration:.2f} seconds "
                f"at ol_speed {self.ol_speed:.1f}; stop_speed {self.stop_speed:.1f}."
            )

        elapsed = now - self.drive_start_time
        if not self.run_complete and elapsed < self.run_duration:
            super().pub_cb()
            return

        self.run_complete = True
        self._publish_stop(elapsed)

    def _publish_stop_command(self):
        msg = Twist()
        msg.linear.x = self.stop_speed
        msg.angular.z = self.neutral_steer
        msg.angular.z = max(1010.0, min(1990.0, msg.angular.z))
        self.init_vel_pub.publish(msg)

    def _publish_stop(self, elapsed):
        self._publish_stop_command()

        if not self.stop_logged:
            odom_text = "odom displacement unavailable"
            if self.start_odom_xy is not None and self.latest_odom_xy is not None:
                dx = self.latest_odom_xy[0] - self.start_odom_xy[0]
                dy = self.latest_odom_xy[1] - self.start_odom_xy[1]
                odom_dist = (dx * dx + dy * dy) ** 0.5
                odom_text = f"odom displacement {odom_dist:.3f} m"

            self.get_logger().info(
                f"Timed lidar drive stopped after {elapsed:.2f} seconds; {odom_text}."
            )
            self.stop_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = TimedLidarBugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
