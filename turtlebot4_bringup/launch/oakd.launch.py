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


import os

from ament_index_python.packages import get_package_share_directory

from launch import launch_description_sources, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
import launch_ros.descriptions

ARGUMENTS = [

]

def generate_launch_description():

    pkg_depthai_examples = get_package_share_directory('depthai_examples')

    rgb_stereo_launch_file = PathJoinSubstitution(
        [pkg_depthai_examples, 'launch', 'rgb_stereo_node.launch.py'])

    tf_prefix = LaunchConfiguration('tf_prefix', default='oakd_pro')

    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value=tf_prefix,
        description='The name of the camera. It can be different from the camera model and it will be used in naming TF.')

    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rgb_stereo_launch_file]),
        launch_arguments=[('colorResolution', '1080p'),
                          ('useVideo', False),
                          ('usePreview', True),
                          ('useDepth', True),
                          ('previewWidth', 300),
                          ('previewHeight', 300),
                          ('publish_urdf', False),
                          ('tf_prefix', tf_prefix)])

    ld = LaunchDescription()
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(oakd_launch)
    return ld
