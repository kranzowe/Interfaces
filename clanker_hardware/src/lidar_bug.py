#!/usr/bin/env python3

from pynput import keyboard

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from threading import Thread, Lock

from math import floor, isinf
import numpy as np

MIN_SPEED = .2
MAX_SPEED = 1.0
INCREMENT_SPEED = 0.1
MIN_TURN = .1
MAX_TURN = 2.0
INCREMENT_TURN = 0.2

STALE_TIME = 0.25

class WASDNode(Node):

    direction = 0
    speed = MIN_SPEED
    direction_stale = 0

    steer = 0
    steer_rate = MIN_TURN
    steer_stale = 0

    listener_lock = Lock()

    optimal_angle = 0

    def __init__(self):
        super().__init__("wasd_node")

        #subscribe to the scan data
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)


        #initialize the velocity publisher
        self.init_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        #declare parameters
        self.declare_parameter("ol_speed", 1500.0)
        self.declare_parameter("tune_mode", True)
        self.declare_parameter("pwm_mode", True)
        self.declare_parameter("netrual_steer", 1470.0)
        self.declare_parameter("lidar_res", 720)
        self.declare_parameter("integration_range", 10) #units are degrees
        self.declare_parameter("exclusion_width", 150)
        self.declare_parameter("steer_p", 10)
        
        self.neutral_steer = self.get_parameter("netrual_steer").value
        self.ol_speed = self.get_parameter("ol_speed").value
        self.pwm_mode = self.get_parameter("pwm_mode").value
        self.tune_mode = self.get_parameter("tune_mode").value
        self.lidar_resolution = self.get_parameter("lidar_res").value
        self.steer_p = self.get_parameter("steer_p").value
        self.integration_range = floor(self.get_parameter("integration_range").value / 360 * self.lidar_resolution)
        self.exclusion_width = floor(self.get_parameter("exclusion_width").value / 360 * self.lidar_resolution)

        #start a timer to handle consistent message pub
        self.create_timer(0.1, self.pub_cb)

        self.create_timer(1, self.update_param)

      

    def scan_cb(self, msg):

        scan_ranges = np.array(msg.ranges)
        self.get_logger().info(f"{scan_ranges.shape}")

        for idx, meas_range in enumerate(scan_ranges):
            if(isinf(meas_range)):
                scan_ranges[idx] = 12.0

            if(meas_range > 12):
                scan_ranges[idx] = 12.0

        width_integral = np.zeros((self.lidar_resolution + self.integration_range))

        for i in range(self.integration_range):
            width_integral[i:i+self.lidar_resolution] += scan_ranges
        
        trim_1_integral = width_integral[:self.lidar_resolution]
        trim_1_integral[:self.integration_range] = width_integral[self.lidar_resolution:]

        #trim the back out
        trim_1_integral[:floor((self.exclusion_width + self.integration_range)/ 2)] = 0
        trim_1_integral[floor(self.lidar_resolution - (self.exclusion_width - self.integration_range / 2)):] = 0

        #get the optimal angle
        self.optimal_angle = (np.argmax(trim_1_integral) + self.integration_range / 2) / round(self.lidar_resolution / 360) - 180


    def pub_cb(self):

        msg = Twist()

        msg.linear.x = self.ol_speed 
        msg.angular.z = self.optimal_angle * self.steer_p + 1500.0

        if(msg.angular.z > 1990):
            msg.angular.z = 1990.0
        elif(msg.angular.z < 1010):
            msg.angular.z = 1010.0

        self.init_vel_pub.publish(msg)



    def get_ros_time_as_double(self):
        #return the ros2 time as float
        return self.get_clock().now().seconds_nanoseconds()[1] * 1e-9 + self.get_clock().now().seconds_nanoseconds()[0]
    
    def update_param(self):

        self.ol_speed = self.get_parameter("ol_speed").value
        self.tune_mode = self.get_parameter("tune_mode").value
        self.pwm_mode = self.get_parameter("pwm_mode").value
        self.neutral_steer = self.get_parameter("netrual_steer").value
        self.lidar_resolution = self.get_parameter("lidar_res").value
        self.integration_range = floor(self.get_parameter("integration_range").value / 360 * self.lidar_resolution)
        self.exclusion_width = floor(self.get_parameter("exclusion_width").value / 360 * self.lidar_resolution)
        self.steer_p = self.get_parameter("steer_p").value






def main(args=None):
    rclpy.init(args=args)

    wasd_node = WASDNode()

    rclpy.spin(wasd_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wasd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()