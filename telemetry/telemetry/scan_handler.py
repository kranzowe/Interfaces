import base64
from io import BytesIO
from flask import Flask
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import os
import json
from rclpy.qos import qos_profile_sensor_data
from ament_index_python import get_package_share_directory

class ScanHandler:
    def __init__(self, node, debug):
        self.node = node

        self.scan_x = []
        self.scan_y = []
        self.integral_scan_x = []
        self.integral_scan_y = []
        self.integral_scan_int = []
        self.control_angle = None

        self.lidar_range = 12.0
        self.lidar_scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.integral_scan_sub = self.node.create_subscription(
            LaserScan, '/integral_scan', self.integral_scan_callback, qos_profile_sensor_data)
        self.control_angle_sub = self.node.create_subscription(
            Float32, '/optimal_angle', self.control_angle_callback, qos_profile_sensor_data)

        self.scan_fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_scan = self.scan_fig.subplots(1)
        self.ax_scan.set_title("Lidar Integration")
        self.ax_scan.set_xlim([-self.lidar_range-1.0,self.lidar_range+1.0])
        self.ax_scan.set_ylim([-self.lidar_range-1.0,self.lidar_range+1.0])
        self.ax_scan.scatter([0], [0], c='c') # Origin
    
        self.scatter_scan = None
        self.scatter_integral_scan = None
        self.control_angle_arrow = None

        self.last_scan = None
        self.last_integral_scan = None
        self.last_control_angle = None

        if debug:
            self.debug_lidar_stream = self.node.create_timer(0.1, self.dummy_lidar_callback)

    def dummy_lidar_callback(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        self.control_angle = -np.pi/12 + 0.1 * np.sin(now)
        scan_path = os.path.join(get_package_share_directory('telemetry'), 'data/laserscan.json')
        with open(scan_path, 'r') as f:
            lidar_scan_load = json.load(f)
            dummy_scan = LaserScan()
            dummy_scan.ranges = [float(r) if r is not None else float('inf') for r in lidar_scan_load['ranges']]
            dummy_scan.intensities = [float(r) if r is not None else float('inf') for r in lidar_scan_load['intensities']]
        self.integral_scan_callback(dummy_scan)
        self.last_scan = now
        self.last_integral_scan = now
        self.last_control_angle = now

    def scan_callback(self, msg):
        pts = np.linspace(np.pi, -np.pi, len(msg.ranges))
        self.scan_x = [np.sin(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.scan_y = [np.cos(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.last_scan = self.node.get_clock().now().nanoseconds / 1e9

    def integral_scan_callback(self, msg):
        pts = np.linspace(np.pi, -np.pi, len(msg.ranges))
        self.integral_scan_x = [np.sin(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.integral_scan_y = [np.cos(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.integral_scan_int = [msg.intensities[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.last_integral_scan = self.node.get_clock().now().nanoseconds / 1e9

    def control_angle_callback(self, msg):
        self.control_angle = np.deg2rad(msg.data)
        self.last_control_angle = self.node.get_clock().now().nanoseconds / 1e9

    def update_scan_plot(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        if self.scan_x:
            if self.scatter_scan is None:
                self.scatter_scan = self.ax_scan.scatter(self.scan_x, self.scan_y, c='r', s=2)
            elif now - self.last_scan < self.node.gc_rate:
                pts = list(zip(self.scan_x, self.scan_y))
                self.scatter_scan.set_offsets(pts)
            else:
                self.scatter_scan.set_offsets([])
        if self.integral_scan_x:
            if self.scatter_integral_scan is None:
                self.scatter_integral_scan = self.ax_scan.scatter(self.integral_scan_x, self.integral_scan_y, c=self.integral_scan_int, cmap='summer', s=2)
            elif now - self.last_integral_scan < self.node.gc_rate:
                pts = list(zip(self.integral_scan_x, self.integral_scan_y))
                self.scatter_integral_scan.set_offsets(pts)
                self.scatter_integral_scan.set_array(self.integral_scan_int)
            else:
                self.scatter_integral_scan.set_offsets([])
        if self.control_angle is not None:
            arrow_len = 3
            if self.control_angle_arrow is None:
                self.control_angle_arrow = self.ax_scan.arrow(0, 0, arrow_len * np.sin(-self.control_angle), arrow_len * np.cos(-self.control_angle), color='c')
            elif now - self.last_control_angle < self.node.gc_rate:
                self.control_angle_arrow.set_data(dx=arrow_len * np.sin(-self.control_angle), dy=arrow_len * np.cos(-self.control_angle))
            else:
                self.control_angle_arrow.set_data(dx=0, dy=0)
        return self.scan_fig