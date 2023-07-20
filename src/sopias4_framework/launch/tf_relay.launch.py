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
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
]


def generate_launch_description():
    pkg_sopias4_framework = get_package_share_directory("sopias4_framework")

    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    tf_relay = Node(
        package="sopias4_framework",
        executable="tf_relay.py",
        name="tf_relay",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
        namespace=namespace,
    )
    tf_relay_static = Node(
        package="sopias4_framework",
        executable="tf_static_relay.py",
        name="tf_static_relay",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
        namespace=namespace,
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(tf_relay)
    ld.add_action(tf_relay_static)
    return ld
