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

import subprocess
import threading
import time
import psutil

from irobot_create_msgs.msg import InterfaceButtons

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from std_msgs.msg import String
from turtlebot4_msgs.msg import UserLed
from geometry_msgs.msg import Twist


class Turtlebot4EmcTests(Node):
    def __init__(self):
        super().__init__('turtlebot4_emc_test')

        self.tests_running = False

        self.create_subscription(InterfaceButtons,
                                 '/interface_buttons',
                                 self.createButtonCallback,
                                 qos_profile_sensor_data)

        self.led_pub = self.create_publisher(UserLed, '/hmi/led', qos_profile_sensor_data)
        self.display_pub = self.create_publisher(String, '/hmi/display/message', qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)

        self.processes = []


    def add_process(self, cmd):
        self.processes.append(subprocess.Popen('exec ' + cmd, stdout=subprocess.PIPE, shell=True))

    def kill_process(self, pid):
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()

    def start_tests(self):
        print("Starting tests")
        self.tests_running = True

        # Blink User 1 Green
        led_msg = UserLed()
        led_msg.led = UserLed.USER_LED_1
        led_msg.color = UserLed.COLOR_GREEN
        led_msg.blink_period = 1000
        led_msg.duty_cycle = 0.5
        self.led_pub.publish(led_msg)

        # Write to display
        display_msg = String()
        display_msg.data = "EMC Test Running"
        self.display_pub.publish(display_msg)

        # Multiple cameras
        self.add_process('ros2 run turtlebot4_tests multi_cam')
        # TurtleBot 4 nodes
        self.add_process('ros2 launch turtlebot4_bringup emc_tasks.launch.py')
        # Stress test
        self.add_process('stress -c 4')

    def stop_tests(self):
        print("Stopping tests")
        self.tests_running = False

        for p in self.processes:
            self.kill_process(p.pid)

        # Turn off User 1 LED
        led_msg = UserLed()
        led_msg.led = UserLed.USER_LED_1
        led_msg.color = UserLed.COLOR_OFF
        self.led_pub.publish(led_msg)

        # Write to display
        display_msg = String()
        display_msg.data = "EMC Test Stopped"
        self.display_pub.publish(display_msg)

    def createButtonCallback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed and not self.tests_running:
            self.start_tests()

        if msg.button_2.is_pressed and self.tests_running:
            self.stop_tests()

    def run(self):
        cmd = Twist()
        cmd.angular.z = 1.9

        prev_swap_time = time.time()
        swap_period = 2.0

        while True:
            if self.tests_running:
                if time.time() - prev_swap_time > swap_period:
                    cmd.angular.z = -cmd.angular.z
                    prev_swap_time = time.time()
                self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    tests = Turtlebot4EmcTests()

    thread = threading.Thread(target=rclpy.spin, args=(tests,), daemon=True)
    thread.start()

    print('Running Turtlebot4 EMC tests...\n')

    tests.run()

    tests.destroy_node()
    rclpy.shutdown()

    thread.join()



if __name__ == '__main__':
    main()
