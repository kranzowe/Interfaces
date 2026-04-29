#!/usr/bin/env python3

import numpy as np
import rclpy

from lidar_bug import WASDNode


class ReverseWASDNode(WASDNode):
    node_name = "lidar_bug_reverse_node"
    ol_speed_default = 1610.0
    steer_sign_default = -1.0

    def _prepare_scan_ranges(self, scan_ranges):
        # Treat the physical rear of the rover as the travel direction.
        return np.roll(scan_ranges, len(scan_ranges) // 2)


def main(args=None):
    rclpy.init(args=args)
    node = ReverseWASDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
