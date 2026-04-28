#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from math import floor, isinf, pi
import numpy as np


class WASDNode(Node):

    optimal_angle = 0.0
    target_angle = 0.0
    instant_angular_rate = 0.0
    previous_angle = 0.0
    previous_time = 0.0
    current_time = 0.0
    wall_ahead = float('inf')

    def __init__(self):
        super().__init__("wasd_node")

        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        self.init_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.integral_scan_pub = self.create_publisher(LaserScan, "integral_scan", 10)
        self.optimal_angle_pub = self.create_publisher(Float32, "optimal_angle", 10)

        self.declare_parameter("ol_speed", 1500.0)
        # Tune neutral_steer until the rover goes straight with no gap error
        self.declare_parameter("neutral_steer", 1470.0)
        # Set steer_sign to -1.0 if the rover steers the wrong direction
        self.declare_parameter("steer_sign", 1.0)
        # Proportional gain: PWM units per degree of gap error
        self.declare_parameter("steer_p", 5.0)
        # Derivative gain: PWM units per degree/s of angular rate
        self.declare_parameter("steer_d", 0.3)
        # Normal saturation angle — errors beyond this are clamped in straight sections
        self.declare_parameter("steer_phi", 45.0)
        # Maximum steering correction in PWM units
        self.declare_parameter("steer_max", 400.0)
        # Bias target angle (degrees) to compensate for mechanical drift — negative = bias right
        self.declare_parameter("target_angle", 0.0)
        # Distance (m) at which wall-ahead detection starts forcing a sharper turn
        self.declare_parameter("wall_ahead_distance", 1.5)
        self.declare_parameter("lidar_res", 720)
        self.declare_parameter("integration_range", 10)   # degrees
        self.declare_parameter("exclusion_width", 150)    # degrees either side of rear to ignore
        self.declare_parameter("range_threshold", 0.6)

        self._load_params()

        self.create_timer(0.1, self.pub_cb)
        self.create_timer(1.0, self.update_param)

    def _load_params(self):
        self.ol_speed = self.get_parameter("ol_speed").value
        self.neutral_steer = self.get_parameter("neutral_steer").value
        self.steer_sign = self.get_parameter("steer_sign").value
        self.steer_p = self.get_parameter("steer_p").value
        self.steer_d = self.get_parameter("steer_d").value
        self.steer_phi = self.get_parameter("steer_phi").value
        self.steer_max = self.get_parameter("steer_max").value
        self.target_angle = self.get_parameter("target_angle").value
        self.wall_ahead_distance = self.get_parameter("wall_ahead_distance").value
        self.lidar_resolution = self.get_parameter("lidar_res").value
        self.range_threshold = self.get_parameter("range_threshold").value
        deg_to_samples = self.lidar_resolution / 360.0
        self.integration_range = floor(
            self.get_parameter("integration_range").value * deg_to_samples
        )
        self.exclusion_width = floor(
            self.get_parameter("exclusion_width").value * deg_to_samples
        )

    def scan_cb(self, msg):
        self.current_time = self.get_ros_time_as_double()

        scan_ranges = np.array(msg.ranges, dtype=float)
        actual_res = len(scan_ranges)

        # Adapt if the lidar outputs a different resolution than the parameter
        if actual_res != self.lidar_resolution:
            self.get_logger().warn(
                f"Scan length {actual_res} != lidar_res {self.lidar_resolution}, adapting."
            )
            self.lidar_resolution = actual_res
            deg_to_samples = actual_res / 360.0
            self.integration_range = floor(
                self.get_parameter("integration_range").value * deg_to_samples
            )
            self.exclusion_width = floor(
                self.get_parameter("exclusion_width").value * deg_to_samples
            )

        scan_ranges = self._fill_gaps(scan_ranges)

        # Measure closest obstacle in the forward ±20° arc to detect approaching walls
        fwd = self.lidar_resolution // 2
        fwd_width = round(20 * self.lidar_resolution / 360)
        fwd_slice = scan_ranges[max(0, fwd - fwd_width): fwd + fwd_width]
        finite_vals = fwd_slice[np.isfinite(fwd_slice)]
        self.wall_ahead = float(np.min(finite_vals)) if len(finite_vals) > 0 else float('inf')

        # Sliding window sum: higher = wider open space in that direction
        width_integral = np.zeros(self.lidar_resolution + self.integration_range)
        for i in range(self.integration_range):
            width_integral[i:i + self.lidar_resolution] += scan_ranges

        trim = width_integral[:self.lidar_resolution].copy()
        trim[:self.integration_range] = width_integral[self.lidar_resolution:]

        # Symmetric rear exclusion zone.
        # Index 0 = -180° (directly behind), index lidar_res/2 = 0° (forward).
        # Zero out exclusion_width/2 samples from each end (both sides of rear).
        excl_half = self.exclusion_width // 2
        trim[:excl_half] = 0
        trim[self.lidar_resolution - excl_half:] = 0

        max_val = np.max(trim)
        if max_val > 0:
            threshold_points = (trim > max_val * self.range_threshold).astype(float)
        else:
            threshold_points = np.zeros(self.lidar_resolution)

        raw_idx = self._find_widest_gap_center(threshold_points)
        self.optimal_angle = (
            (raw_idx - self.integration_range / 2.0)
            / round(self.lidar_resolution / 360)
            - 180.0
        )

        scan_out = LaserScan()
        scan_out.angle_min = -pi + pi / self.lidar_resolution
        scan_out.angle_max = pi - pi / self.lidar_resolution
        scan_out.angle_increment = 2 * pi / self.lidar_resolution
        scan_out.ranges = list(trim / max(self.integration_range, 1))
        scan_out.intensities = list(threshold_points)
        self.integral_scan_pub.publish(scan_out)

        angle_msg = Float32()
        angle_msg.data = float(self.optimal_angle)
        self.optimal_angle_pub.publish(angle_msg)

        if self.previous_time != 0.0:
            dt = self.current_time - self.previous_time
            if dt > 0.0:
                self.instant_angular_rate = (
                    (self.optimal_angle - self.previous_angle) / dt
                )

        self.previous_angle = self.optimal_angle
        self.previous_time = self.current_time

    def _fill_gaps(self, scan_ranges):
        """Linearly interpolate across infinite (no-return) scan gaps."""
        n = len(scan_ranges)
        found_first_valid = False
        idx = 0
        while idx < n:
            r = scan_ranges[idx]
            if isinf(r) and found_first_valid:
                # Find the next finite reading (with a hard limit to avoid infinite loop)
                end_offset = None
                for k in range(1, n):
                    ni = (idx + k) % n
                    if not isinf(scan_ranges[ni]):
                        end_offset = k
                        break
                if end_offset is None:
                    break  # all remaining readings are inf
                end_idx = (idx + end_offset) % n
                start_val = scan_ranges[idx - 1]
                end_val = scan_ranges[end_idx]
                for j in range(end_offset):
                    scan_ranges[(idx + j) % n] = (
                        start_val + (end_val - start_val) * (j + 1) / (end_offset + 1)
                    )
                idx += end_offset
            else:
                if not isinf(r):
                    found_first_valid = True
                idx += 1
        return scan_ranges

    def _find_widest_gap_center(self, threshold_points):
        """Dilate threshold map and return the midpoint index of the widest open region."""
        original = threshold_points.copy()
        for idx in range(2, self.lidar_resolution - 3):
            if original[idx] > 0:
                threshold_points[idx - 2:idx + 3] = 1.0

        best_start, best_len = -1, 0
        cur_start, cur_len = -1, 0

        for idx in range(self.lidar_resolution):
            if threshold_points[idx] > 0.0:
                if cur_start == -1:
                    cur_start = idx
                cur_len += 1
            else:
                if cur_len > best_len:
                    best_len = cur_len
                    best_start = cur_start
                cur_start, cur_len = -1, 0

        if cur_len > best_len:
            best_start = cur_start
            best_len = cur_len

        if best_start == -1:
            return 0
        return best_start + best_len / 2.0

    def pub_cb(self):
        if self.previous_time == 0.0:
            return

        # error > 0 means gap is to the left; error < 0 means gap is to the right
        error = self.optimal_angle - self.target_angle

        # When a wall is close ahead, scale steer_phi up toward 90° so the rover
        # commands a hard turn instead of a capped modest correction.
        if self.wall_ahead < self.wall_ahead_distance:
            proximity = 1.0 - (self.wall_ahead / self.wall_ahead_distance)
            effective_phi = self.steer_phi + (90.0 - self.steer_phi) * proximity
        else:
            effective_phi = self.steer_phi

        error_sat = max(-effective_phi, min(effective_phi, error))

        # PD control — smooth, no chattering
        correction = (
            self.steer_p * error_sat
            + self.steer_d * self.instant_angular_rate
        )
        correction = max(-self.steer_max, min(self.steer_max, correction))

        msg = Twist()
        msg.linear.x = self.ol_speed
        msg.angular.z = self.neutral_steer + self.steer_sign * correction
        msg.angular.z = max(1010.0, min(1990.0, msg.angular.z))

        self.init_vel_pub.publish(msg)

    def get_ros_time_as_double(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return sec + nsec * 1e-9

    def update_param(self):
        self._load_params()


def main(args=None):
    rclpy.init(args=args)
    node = WASDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
