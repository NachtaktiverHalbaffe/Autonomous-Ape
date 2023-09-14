import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

SOPIAS4_FLEETBROKER_PATH = get_package_share_directory("sopias4_fleetbroker")
ARGUMENTS = [
    DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            SOPIAS4_FLEETBROKER_PATH,
            "maps",
            "default_map.yaml",
        ),
        description="Full path to map yaml file to load",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            SOPIAS4_FLEETBROKER_PATH, "config", "map_server.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    ),
    DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Whether to respawn if a node crashes",
    ),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    ),
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    DeclareLaunchArgument(
        "use_domain_bridge",
        default_value="true",
        description="If Sopias4 DomainBridge should also be launches",
    ),
]


def generate_launch_description():
    # Get the launch directory

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [SOPIAS4_FLEETBROKER_PATH, "launch", "map_server.launch.py"]
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

    domain_bridges = Node(
        package="sopias4_fleetbroker",
        executable="sopias4_domain_bridge",
        name="sopias4_domain_bridge",
        condition=IfCondition(LaunchConfiguration("use_domain_bridge")),
    )

    multi_robot_coordinator = Node(
        package="sopias4_fleetbroker",
        executable="multi_robot_coordinator",
        name="multi_robot_coordinator",
    )

    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(multi_robot_coordinator)
    ld.add_action(map_server)
    ld.add_action(domain_bridges)

    return ld
