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

class GyroHandler:
    def __init__(self, node, debug):
        self.node = node

        self.imu_time_window_s = 10.0
        self.imu_time_buffer = []
        self.imu_x_buffer = []
        self.imu_y_buffer = []
        self.imu_z_buffer = []
        self.imu_sub = self.node.create_subscription(
            Vector3, '/imu/gyro', self.imu_callback, qos_profile_sensor_data)
        
        self.imu_fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_x, self.ax_y, self.ax_z = self.imu_fig.subplots(3,1)
        self.ax_x.set_title("Gyro X Angular Velocity")
        self.ax_y.set_title("Gyro Y Angular Velocity")
        self.ax_z.set_title("Gyro Z Angular Velocity")
        self.line_x, self.line_y, self.line_z = None, None, None

        self.t0 = self.node.get_clock().now().nanoseconds / 1e9
        self.last_imu = None

        if debug:
            self.debug_imu_stream = self.node.create_timer(0.01, self.dummy_imu_callback)

    def dummy_imu_callback(self):
        time = self.node.get_clock().now().nanoseconds / 1e9 - self.t0
        if time % 40 > 15:
            self.imu_time_buffer.append(time)
            self.imu_x_buffer.append(np.sin(time*10))
            self.imu_y_buffer.append(np.cos(time*10))
            self.imu_z_buffer.append(-9.8)
            while (self.imu_time_buffer[-1] - self.imu_time_buffer[0]) > self.imu_time_window_s:
                self.imu_time_buffer.pop(0)
                self.imu_x_buffer.pop(0)
                self.imu_y_buffer.pop(0)
                self.imu_z_buffer.pop(0)
            
            self.last_imu = time

    def imu_callback(self, msg):
        time = self.node.get_clock().now().nanoseconds / 1e9 - self.t0
        self.imu_time_buffer.append(time)
        self.imu_x_buffer.append(msg.x)
        self.imu_y_buffer.append(msg.y)
        self.imu_z_buffer.append(msg.z)
        while (self.imu_time_buffer[-1] - self.imu_time_buffer[0]) > self.imu_time_window_s:
            self.imu_time_buffer.pop(0)
            self.imu_x_buffer.pop(0)
            self.imu_y_buffer.pop(0)
            self.imu_z_buffer.pop(0)
        
        self.last_imu = time

    def update_imu_plot(self):
        now = self.node.get_clock().now().nanoseconds / 1e9 - self.t0
        if self.imu_time_buffer:
            if self.line_x is None:
                self.line_x, = self.ax_x.plot(self.imu_time_buffer, self.imu_x_buffer, c='g')
                self.line_y, = self.ax_y.plot(self.imu_time_buffer, self.imu_y_buffer, c='g')
                self.line_z, = self.ax_z.plot(self.imu_time_buffer, self.imu_z_buffer, c='g')
            elif now - self.last_imu < self.node.gc_rate:
                self.ax_x.set_xlim([np.min(self.imu_time_buffer), np.max(self.imu_time_buffer)])
                self.ax_y.set_xlim([np.min(self.imu_time_buffer), np.max(self.imu_time_buffer)])
                self.ax_z.set_xlim([np.min(self.imu_time_buffer), np.max(self.imu_time_buffer)])
                self.ax_x.set_ylim([np.min(self.imu_x_buffer)-1e-3, np.max(self.imu_x_buffer)+1e-3])
                self.ax_y.set_ylim([np.min(self.imu_y_buffer)-1e-3, np.max(self.imu_y_buffer)+1e-3])
                self.ax_z.set_ylim([np.min(self.imu_z_buffer)-1e-3, np.max(self.imu_z_buffer)+1e-3])
                self.line_x.set_data(self.imu_time_buffer, self.imu_x_buffer)
                self.line_y.set_data(self.imu_time_buffer, self.imu_y_buffer)
                self.line_z.set_data(self.imu_time_buffer, self.imu_z_buffer)
            else:
                self.line_x.set_data([], [])
                self.line_y.set_data([], [])
                self.line_z.set_data([], [])
        return self.imu_fig