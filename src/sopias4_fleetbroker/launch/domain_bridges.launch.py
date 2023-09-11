import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from nav2_common.launch import ReplaceString

ARGUMENTS = [
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
]


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    param_file = LaunchConfiguration("params_file")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("sopias4_map_server"),
            "config",
            "domain_bridge_namespaceless.yaml",
        ),
        description="Domain bridge parameters",
    )

    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    # Make shure that the keyword doesnt have a leading "/"
    params_file_turtlebot1 = ReplaceString(
        source_file=param_file,
        replacements={"<robot_namespace>": ("turtle1")},
    )

    domain_bridge_turtle1 = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("domain_bridge"),
                "launch",
                "domain_bridge.launch.xml",
            )
        ),
        launch_arguments={
            "bidirectional": "True",
            "config": params_file_turtlebot1,
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(params_arg)
    ld.add_action(domain_bridge_turtle1)
    return ld
