# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    # pkg_project = get_package_share_directory('my_sos')
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the URDF file from "description" package
    # urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'robot.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    velor = Node(
        package='my_sos',
        executable='velor',
        name='velor'
    )
    
    sos_vis = Node(
        package='my_sos',
        executable='sos_vis',
        name='sos_vis'
    )

    return LaunchDescription([
        
        velor,
        sos_vis
        # TimerAction(
        #     period=5.0,
        #     actions=[create])
    ])
