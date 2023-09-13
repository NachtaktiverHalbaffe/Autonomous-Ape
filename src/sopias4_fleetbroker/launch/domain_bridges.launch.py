import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from nav2_common.launch import ReplaceString, RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("sopias4_fleetbroker"),
            "config",
            "domain_bridge_namespaceless.yaml",
        ),
        description="Domain bridge parameters",
    ),
]


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    params_file = LaunchConfiguration("params_file")
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    # Make shure that the keyword doesnt have a leading "/"
    # params_file_turtlebot1 = ReplaceString(
    #     source_file=param_file,
    #     replacements={"<robot_namespace>": ("turtle1")},
    # )
    # params_file_turtlebot1 = RewrittenYaml(
    #     source_file=params_file_turtlebot1, param_rewrites={}
    # )

    domain_bridge_turtle1 = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="sopias4_domain_bridge",
        arguments=[params_file],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(domain_bridge_turtle1)
    return ld
