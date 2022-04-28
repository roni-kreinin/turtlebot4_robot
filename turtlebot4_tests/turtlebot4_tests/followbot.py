#!/usr/bin/env python3

# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

# Description:
#   This script exposes a set of useful tests at the ROS-level,
#   via ROS topics, to verify the functionality of core features.
#   In addition, these tests together serve as a useful robot-level
#   diagnostic tool, be identifying the root cause of problems,
#   or at the very least, narrowing down on where the root cause(s) may be.
#
# Usage:
#   ros2 run turtlebot4_tests ros_tests

import os
from os.path import expanduser
from re import L

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from vision_msgs.msg import Detection2D, Detection2DArray

from turtlebot4_msgs.msg import UserLed

from geometry_msgs.msg import Twist


class FollowBot(Node):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2
    CENTER = 3
    FORWARD_LEFT = 4
    FORWARD_RIGHT = 5
    STOP = 6

    direction = UNKNOWN
    previous_direction = RIGHT
    image_width = 300
    image_height = 300
    fwd_margin = 25
    turn_margin = 50

    def __init__(self):
        super().__init__('followbot')

        mobilenet_sub = self.create_subscription(Detection2DArray,
                                                     '/color/mobilenet_detections',
                                                     self.mobilenetCallback,
                                                     qos_profile_sensor_data)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)
        self.user_led_pub = self.create_publisher(UserLed, '/hmi/led', qos_profile_sensor_data)

    def mobilenetCallback(self, msg: Detection2DArray):
        if len(msg.detections) > 0:
            for detection in msg.detections:
                # Person detected
                if detection.id == '15':
                    position_x = detection.bbox.center.x
                    position_y = detection.bbox.center.y

                    left_boundary = position_x - detection.bbox.size_x / 2
                    right_boundary = position_x + detection.bbox.size_x / 2

                    bbox_size = detection.bbox.size_x * detection.bbox.size_y

                   # print("{0}:{1}".format(position_x, bbox_size))

                    if left_boundary > self.image_width / 2 + self.turn_margin:
                        self.direction = self.RIGHT
                    elif left_boundary > self.image_width / 2 + self.fwd_margin:
                        self.direction = self.FORWARD_RIGHT
                    elif right_boundary < self.image_width / 2 - self.turn_margin:
                        self.direction = self.LEFT
                    elif right_boundary < self.image_width / 2 - self.fwd_margin:
                        self.direction = self.FORWARD_LEFT
                    else:
                        if bbox_size < 40000.0:
                            self.direction = self.CENTER
                        else:
                            self.direction = self.STOP
                    return
        else:
            self.direction = self.UNKNOWN

    def led(self, led, color, period, duty):
        msg = UserLed()
        msg.led = led
        msg.color = color
        msg.blink_period = period
        msg.duty_cycle = duty
        self.user_led_pub.publish(msg)

    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x

        self.cmd_vel_pub.publish(msg)

    def run(self):
        msg = UserLed()
        while True:
            if self.direction == self.STOP:
                self.drive(0.0, 0.0)
                self.led(0, 0, 1000, 0.0)
                self.led(1, 2, 1000, 1.0)
            elif self.direction == self.CENTER:
                self.drive(0.3, 0.0)
                self.led(0, 1, 1000, 0.5)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.LEFT:
                self.drive(0.0, 0.35)
                self.previous_direction = self.LEFT
                self.led(0, 0, 1000, 0.5)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.RIGHT:
                self.drive(0.0, -0.35)
                self.previous_direction = self.RIGHT
                self.led(0, 1, 1000, 0.5)
                self.led(1, 0, 1000, 0.5)
            elif self.direction == self.FORWARD_LEFT:
                self.drive(0.2, 0.25)
                self.previous_direction = self.LEFT
                self.led(0, 1, 1000, 1.0)
                self.led(1, 1, 1000, 0.5)
            elif self.direction == self.FORWARD_RIGHT:
                self.drive(0.2, -0.25)
                self.previous_direction = self.LEFT
                self.led(0, 1, 1000, 0.5)
                self.led(1, 1, 1000, 1.0)
            else:
                if self.previous_direction == self.LEFT:
                    self.drive(0.0, 0.75)
                    self.led(0, 0, 500, 0.5)
                    self.led(1, 1, 500, 0.5)
                else:
                    self.drive(0.0, -0.75)
                    self.led(0, 1, 500, 0.5)
                    self.led(1, 0, 500, 0.5)

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)

    node = FollowBot()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print('Running FollowBot...\n')

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    thread.join()


if __name__ == '__main__':
    main()
