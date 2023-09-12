# Copyright (c) 2018 Intel Corporation
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    sopias4_fleetbroker_path = get_package_share_directory("sopias4_fleetbroker")

    params_file = LaunchConfiguration("params_file")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            sopias4_fleetbroker_path, "maps", "default_map.yaml"
        ),
        description="Full path to map yaml file to load",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            sopias4_fleetbroker_path, "config", "map_server.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Whether to respawn if a node crashes",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [sopias4_fleetbroker_path, "launch", "map_server.launch.py"]
            ),
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_respawn": LaunchConfiguration("use_respawn"),
            "log_level": LaunchConfiguration("log_level"),
            "autostart": LaunchConfiguration("autostart"),
        }.items(),
    )

    multi_robot_coordinator = Node(
        package="sopias4_fleetbroker",
        executable="multi_robot_coordinator",
        name="multi_robot_coordinator",
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(multi_robot_coordinator)
    ld.add_action(map_server)

    return ld
