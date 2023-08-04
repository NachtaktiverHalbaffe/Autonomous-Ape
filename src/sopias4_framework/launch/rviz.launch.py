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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    ),
    DeclareLaunchArgument(
        "use_description",
        default_value="false",
        description="Launch turtlebot4 description",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Path to Rviz configuration file",
    ),
]


def generate_launch_description():
    pkg_path = get_package_share_directory("sopias4_framework")

    rviz2_config = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    description = LaunchConfiguration("use_description")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("map", "/map")]

    rviz = GroupAction(
        [
            PushRosNamespace(namespace),
            #  --- Use own param file ---
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz2_config],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=remappings,
                namespace=namespace,
                output="screen",
                ros_arguments=["--log-level", "FATAL"],
                condition=LaunchConfigurationNotEquals("params_file", ""),
            ),
            # --- Use Turtlebot4 param file ---
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            pkg_path,
                            "config",
                            "rviz.rviz",
                        ]
                    ),
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=remappings,
                namespace=namespace,
                ros_arguments=["--log-level", "ERROR"],
                output="screen",
                condition=LaunchConfigurationEquals("params_file", ""),
            ),
            # Delay launch of robot description to allow Rviz2 to load first.
            # Prevents visual bugs in the model.
            TimerAction(
                condition=IfCondition(description),
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("turtlebot4_description"),
                                "launch",
                                "robot_description.launch.py",
                            ]
                        ),
                        launch_arguments=[
                            ("model", "standard"),
                            ("namespace", namespace),
                        ],
                    )
                ],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)

    return ld
