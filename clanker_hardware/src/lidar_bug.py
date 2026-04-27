#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from threading import Thread, Lock

from math import floor, isinf, pi
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
    target_angle = 0
    
    previous_angle = 0
    previous_time = 0 
    current_time = 0

    def __init__(self):
        super().__init__("wasd_node")

        #subscribe to the scan data
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)


        #initialize the velocity publisher
        self.init_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.integral_scan_pub = self.create_publisher(LaserScan, "integral_scan", 10)
        self.optimal_angle_pub = self.create_publisher(Float32, "optimal_angle", 10)

        #declare parameters
        self.declare_parameter("ol_speed", 1500.0)
        self.declare_parameter("tune_mode", True)
        self.declare_parameter("pwm_mode", True)
        self.declare_parameter("netrual_steer", 1470.0)
        self.declare_parameter("lidar_res", 720)
        self.declare_parameter("integration_range", 10) #units are degrees
        self.declare_parameter("exclusion_width", 150)
        self.declare_parameter("steer_p", 50.0)
        self.declare_parameter("steer_lamda", 0.3)
        self.declare_parameter("steer_phi", 3.0)
        self.declare_parameter("steer_b", 0.0)
        self.declare_parameter("steer_b0", 90.0)
        self.declare_parameter("nuetral_steer", 3)
        self.declare_parameter("range_threshold", 0.6)
        
        self.neutral_steer = self.get_parameter("netrual_steer").value
        self.ol_speed = self.get_parameter("ol_speed").value
        self.pwm_mode = self.get_parameter("pwm_mode").value
        self.tune_mode = self.get_parameter("tune_mode").value
        self.lidar_resolution = self.get_parameter("lidar_res").value
        self.steer_p = self.get_parameter("steer_p").value
        self.steer_lamda = self.get_parameter("steer_lamda").value
        self.steer_phi = self.get_parameter("steer_phi").value
        self.steer_b = self.get_parameter("steer_b").value
        self.steer_b0 = self.get_parameter("steer_b0").value
        self.range_threshold = self.get_parameter("range_threshold").value
        self.neutral_steer = self.get_parameter("nuetral_steer").value
        self.integration_range = floor(self.get_parameter("integration_range").value / 360 * self.lidar_resolution)
        self.exclusion_width = floor(self.get_parameter("exclusion_width").value / 360 * self.lidar_resolution)

        #start a timer to handle consistent message pub
        self.create_timer(0.1, self.pub_cb)

        self.create_timer(1, self.update_param)

    def scan_cb(self, msg):

        self.current_time = self.get_ros_time_as_double()


        scan_ranges = np.array(msg.ranges)

        found_first_valid = False
        for idx, meas_range in enumerate(scan_ranges):
            if(isinf(meas_range) and found_first_valid):

                gap_end_found = False
                counter = 1
                gap_end_val = 0
                while(not gap_end_found):
                    inner_idx = (counter + idx) % self.lidar_resolution

                    if not (isinf(scan_ranges[inner_idx])):
                        gap_end_found = True
                        gap_end_val = scan_ranges[inner_idx]

                    counter += 1

                increment = (gap_end_val - scan_ranges[idx - 1]) / counter

                for idx2 in range(0, counter):
                    
                    scan_ranges[(idx + idx2) % self.lidar_resolution] = idx2 * increment + scan_ranges[idx - 1]

            elif(not isinf(meas_range)):
                found_first_valid = True


        width_integral = np.zeros((self.lidar_resolution + self.integration_range))

        for i in range(self.integration_range):
            width_integral[i:i+self.lidar_resolution] += scan_ranges
        
        trim_1_integral = width_integral[:self.lidar_resolution]
        trim_1_integral[:self.integration_range] = width_integral[self.lidar_resolution:]

        #trim the back out
        trim_1_integral[:floor((self.exclusion_width + self.integration_range)/ 2)] = 0
        trim_1_integral[floor(self.lidar_resolution - (self.exclusion_width - self.integration_range / 2)):] = 0

        #get the optimal angle - v1
        #self.optimal_angle = (np.argmax(trim_1_integral) - self.integration_range / 2) / round(self.lidar_resolution / 360) - 180

        #get the optimal angle - v2
        max_range = np.max(trim_1_integral)
        threshold_range = max_range * self.range_threshold
        max_indicies = np.where(trim_1_integral > threshold_range)

        threshold_points = np.zeros((self.lidar_resolution))
        threshold_points[max_indicies] = 1.0

        #take the average of the threshold points
        self.optimal_angle = (self.determine_optimal_angle(threshold_points) - self.integration_range / 2) / round(self.lidar_resolution / 360) - 180

        msg = LaserScan()
        msg.angle_min = -pi + pi / self.lidar_resolution
        msg.angle_max = pi - pi / self.lidar_resolution
        msg.angle_increment = 2 * pi / self.lidar_resolution
        msg.ranges = list(trim_1_integral / self.integration_range)
        msg.intensities = list(threshold_points)

        self.get_logger().info(f"{threshold_points}")

        self.integral_scan_pub.publish(msg)

        msg = Float32()
        msg.data = self.optimal_angle
        self.optimal_angle_pub.publish(msg)

        if(self.previous_time == 0):
            self.instant_angular_rate = (self.optimal_angle - self.previous_angle) / (self.current_time - self.previous_time)


        self.previous_time = self.current_time


    def pub_cb(self):

        #make sure the derivate term is saturated
        if(self.previous_time == 0):
            return

        msg = Twist()

        #slide on that mode
        s = (self.target_angle - self.optimal_angle) * self.steer_lamda + self.instant_angular_rate
        beta_steer = self.steer_p * self.steer_lamda * abs(s) + self.steer_b * self.instant_angular_rate**2 + self.steer_b0
        control_input = -self.sign(s) * beta_steer - self.instant_angular_rate

        msg.linear.x = self.ol_speed 
        msg.angular.z = 1500.0 + control_input

        if(msg.angular.z > 1990):
            msg.angular.z = 1990.0
        elif(msg.angular.z < 1010):
            msg.angular.z = 1010.0

        self.init_vel_pub.publish(msg)

    def determine_optimal_angle(self, threshold_points):

        #run a dilation
        for idx in range(2, self.lidar_resolution - 3):

            threshold_points[(idx - 2):(idx +3)] = threshold_points[idx] * np.array([1,1,1,1,1])

        middle_ind = 0 
        largest_middle_ind = 0
        for idx in range(0, self.lidar_resolution):
            
            if(threshold_points[idx] > 0.0):
                if(middle_ind == 0):
                    middle_ind = idx
                else:
                    middle_ind += .5

                largest_middle_ind = middle_ind
            else:
                middle_ind = 0

        return largest_middle_ind
            


    def sign(self, val):

        if(abs(val) < self.steer_phi * self.steer_lamda):
            return 0 
        
        return val / abs(val) 

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
        self.neutral_steer = self.get_parameter("nuetral_steer").value
        self.steer_lamda = self.get_parameter("steer_lamda").value
        self.steer_phi = self.get_parameter("steer_phi").value
        self.steer_b = self.get_parameter("steer_b").value
        self.steer_b0 = self.get_parameter("steer_b0").value
        self.range_threshold = self.get_parameter("range_threshold").value

        






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