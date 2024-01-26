#!/usr/bin/env python3
import os
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="nebolab_gazebo_sim",
                executable="vicon_localization",
                name="vicon_localization",
            ),
        ]
    )
