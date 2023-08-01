# Copyright 2022 Clearpath Robotics, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes",
    ),
    DeclareLaunchArgument("log_level", default_value="error", description="log level"),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    ),
]


def generate_launch_description():
    lifecycle_nodes = ["amcl"]

    amcl_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("sopias4_framework"),
                "config",
                "amcl.yaml",
            ]
        ),
        description="AMCL parameters",
    )

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    use_respawn = LaunchConfiguration("use_respawn")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("map", "/map")]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
        namespace=namespace,
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        output="screen",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"autostart": LaunchConfiguration("autostart")},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(amcl_params_arg)
    ld.add_action(amcl)
    ld.add_action(lifecycle_manager)
    return ld
