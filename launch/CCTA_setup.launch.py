#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_share_directory("nebolab_gazebo_sim"), "models")


def generate_launch_description():
    print(os.environ["GAZEBO_MODEL_PATH"])

    DeclareLaunchArgument(
        "yaml_name",
        default_value="formation4_mixed",
        description="Name of the yaml file",
    )

    converter = Node(
        package="nebolab_gazebo_sim",
        executable="sdf_convert",
        name="sdf_converter",
        parameters=[{"yaml_name": LaunchConfiguration("yaml_name")}],
    )

    environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nebolab_gazebo_sim"), "launch"
                ),
                "/totally_empty.launch.py",
            ]
        )
    )

    spawn = Node(
        package="nebolab_gazebo_sim",
        executable="spawn_turtlebot",
        name="entity_spawner",
        parameters=[{"yaml_name": LaunchConfiguration("yaml_name")}],
    )

    vicon = Node(
        package="nebolab_gazebo_sim",
        executable="vicon_localization",
        name="vicon_localization",
    )

    return LaunchDescription([converter, environment, spawn, vicon])
