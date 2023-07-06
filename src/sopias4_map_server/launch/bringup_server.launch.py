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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("sopias4_map_server")
    lifecycle_nodes = ["map_saver"]

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "map_server.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    map_server_empty = Node(
        condition=LaunchConfigurationNotEquals("map", ""),
        namespace=namespace,
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {"yaml_filename": map_yaml_file}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    map_server_map_loaded = Node(
        condition=LaunchConfigurationEquals("map", ""),
        namespace=namespace,
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        parameters=[configured_params],
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_sopias4_map_server",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
        ],
    )
    multi_robot_coordinator = Node(
        package="sopias4_map_server",
        executable="multi_robot_coordinator",
        name="multi_robot_coordinator",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(map_server_empty)
    ld.add_action(map_server_map_loaded)
    ld.add_action(multi_robot_coordinator)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
