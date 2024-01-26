#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
<<<<<<< Updated upstream


os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_share_directory("nebolab_gazebo_sim"), "models")


def generate_launch_description():
    DeclareLaunchArgument(
        "yaml_name",
        default_value="formation4_mixed",
        description="Name of the yaml file",
    )
=======
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_nebolab_gazebo_sim = get_package_share_directory("nebolab_gazebo_sim")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ----- EDIT THIS SECTION -----

    # Edit these pose values to change the initial pose of the robot
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # Edit this world file name to change which world file to load
    world = os.path.join(
        pkg_nebolab_gazebo_sim,
        "worlds",
        "example_world.world",
    )

    # ----- END EDIT SECTION -----
>>>>>>> Stashed changes

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_turtlebot3_gazebo,
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

<<<<<<< Updated upstream
    return LaunchDescription([converter, environment, spawn, vicon])
=======
    vicon_fake_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nebolab_gazebo_sim,"launch", "vicon_localization.launch.py")
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(vicon_fake_localization_cmd)

    return ld
>>>>>>> Stashed changes
