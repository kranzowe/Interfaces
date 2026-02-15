#!/usr/bin/env python3

from pynput import keyboard

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from threading import Thread, Lock

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

    def __init__(self):
        super().__init__("wasd_node")

        #initialize the velocity publisher
        self.init_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        #start a timer to handle consistent message pub
        self.create_timer(0.1, self.pub_cb)

        def start_listener():
            #initialize the keyboard listener
            with keyboard.Listener(on_press=self.on_press) as listener:
                listener.join()

        listener_thread = Thread(target= start_listener)
        listener_thread.start()

    def on_press(self, key):


        try:

            with self.listener_lock:
                if(key.char == "w"):
                    self.direction = 1
                    self.direction_stale = self.get_ros_time_as_double() + STALE_TIME
                elif(key.char == "s"):
                    self.direction = -1
                    self.direction_stale = self.get_ros_time_as_double() + STALE_TIME
                elif(key.char == "a"):
                    self.steer = 1
                    self.steer_stale = self.get_ros_time_as_double() + STALE_TIME
                elif(key.char == "d"):
                    self.steer = -1
                    self.steer_stale = self.get_ros_time_as_double() + STALE_TIME
                elif(key.char == "i"):
                    self.speed += INCREMENT_SPEED

                    if(self.speed > MAX_SPEED):
                        self.speed = MAX_SPEED
                elif(key.char == "k"):
                    self.speed -= INCREMENT_SPEED

                    if(self.speed < MIN_SPEED):
                        self.speed = MIN_SPEED
                
                elif(key.char == "l"):
                    self.steer_rate -= INCREMENT_TURN

                    if(self.steer_rate < MIN_TURN):
                        self.steer_rate = MIN_TURN

                elif(key.char == "j"):
                    self.steer_rate += INCREMENT_TURN

                    if(self.steer_rate > MAX_TURN):
                        self.steer_rate = MAX_TURN

 
        except AttributeError:
            pass

    def pub_cb(self):

        msg = Twist()
        time = self.get_ros_time_as_double()

        with self.listener_lock:
            if(time < self.steer_stale):
                msg.angular.z = self.steer * self.steer_rate

            if(time < self.direction_stale):
                msg.linear.x = self.direction * self.speed

        self.init_vel_pub.publish(msg)


    def get_ros_time_as_double(self):
        #return the ros2 time as float
        return self.get_clock().now().seconds_nanoseconds()[1] * 1e-9 + self.get_clock().now().seconds_nanoseconds()[0]


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