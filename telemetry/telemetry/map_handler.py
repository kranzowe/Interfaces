import base64
from io import BytesIO
from flask import Flask
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from PIL import Image
import numpy as np
import os
import yaml
from rclpy.qos import qos_profile_sensor_data
from ament_index_python import get_package_share_directory

class MapHandler:
    def __init__(self, node, debug):
        self.node = node

        self.pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile_sensor_data)
        
        self.map_fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_map = self.map_fig.subplots(1,1)
        self.ax_map.set_axis_off()
        self.scatter_pose = None

        self.node.declare_parameter("map.yaml", 'data/oh_my.yaml')
        map_config_path = os.path.join(get_package_share_directory('telemetry'), self.node.get_parameter("map.yaml").value)
        with open(map_config_path, "r") as f:
            self.map_config = yaml.safe_load(f)

        self.node.declare_parameter("map.img", 'data/oh_my.pgm')
        map_path = os.path.join(get_package_share_directory('telemetry'), self.node.get_parameter("map.img").value)
        img = Image.open(map_path)
        self.image_array = np.flipud(np.array(img))
        self.ax_map.set_title("Map")
        self.ax_map.imshow(self.image_array, cmap='gray')

        self.node.declare_parameter("map.waypoint_csv", 'data/ohmy_big_path_SMOOTH.csv')
        waypoints_path = os.path.join(get_package_share_directory('telemetry'), self.node.get_parameter("map.waypoint_csv").value)
        self.waypoints = np.genfromtxt(waypoints_path, delimiter=',', skip_header=1)
        world_waypoints_x, world_waypoints_y = self.world_pose_to_img_pose(self.waypoints[:,0], self.waypoints[:,1])
        self.ax_map.plot(world_waypoints_x, world_waypoints_y, c='r', zorder=1)

        self.t0 = self.node.get_clock().now().nanoseconds / 1e9
        self.pose_x = None
        self.pose_y = None
        self.last_pose = None

        self.node.declare_parameter("map.window_size", 10.0)
        self.window_size = self.node.get_parameter("map.window_size").value / self.map_config["resolution"]

        self.param_timer = self.node.create_timer(0.1, self.param_cb)

        if debug:
            self.debug_pose_stream = self.node.create_timer(0.1, self.dummy_pose_callback)

    def dummy_pose_callback(self):
        time = self.node.get_clock().now().nanoseconds / 1e9 - self.t0
        if time % 40 > 15:
            self.pose_x = np.sin(time)
            self.pose_y = np.cos(time)
            self.last_pose = time

    def pose_callback(self, msg):
        time = self.node.get_clock().now().nanoseconds / 1e9 - self.t0
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.last_pose = time

    def world_pose_to_img_pose(self, x, y):
        if x is None or y is None:
            return None, None
        map_x = (x - self.map_config["origin"][0]) / self.map_config["resolution"]
        map_y = (y - self.map_config["origin"][1]) / self.map_config["resolution"]
        return (map_x, map_y)

    def update_map_plot(self):
        now = self.node.get_clock().now().nanoseconds / 1e9 - self.t0

        map_x, map_y = self.world_pose_to_img_pose(self.pose_x, self.pose_y)
        if map_x is None:
            self.ax_map.set_xlim([0,self.image_array.shape[1]])
            self.ax_map.set_ylim([0,self.image_array.shape[0]])
        elif self.scatter_pose is None:
            self.scatter_pose = self.ax_map.scatter([map_x], [map_y], c='b', s=50, zorder=2)
            self.ax_map.set_xlim([map_x-self.window_size,map_x+self.window_size])
            self.ax_map.set_ylim([map_y-self.window_size,map_y+self.window_size])
        elif now - self.last_pose < self.node.gc_rate:
            self.scatter_pose.set_offsets([[map_x, map_y]])
            self.ax_map.set_xlim([map_x-self.window_size,map_x+self.window_size])
            self.ax_map.set_ylim([map_y-self.window_size,map_y+self.window_size])
        else:
            self.scatter_pose.set_offsets(np.empty((0, 2)))
            self.ax_map.set_xlim([0,self.image_array.shape[1]])
            self.ax_map.set_ylim([0,self.image_array.shape[0]])
        return self.map_fig

    def param_cb(self):
        self.window_size = self.node.get_parameter("map.window_size").value / self.map_config["resolution"]