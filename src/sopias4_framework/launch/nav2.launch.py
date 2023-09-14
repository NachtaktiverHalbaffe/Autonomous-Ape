import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import ReplaceString

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    ),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    ),
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of conatiner that nodes will load in if use composition",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("sopias4_application"), "config", "nav2.yaml"
        ),
        description="Nav2 parameters",
    ),
]


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")
    autostart = LaunchConfiguration("autostart")
    container_name = LaunchConfiguration("container_name")

    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    # Make shure that the keyword doesnt have a leading "/"
    params_file = ReplaceString(
        source_file=LaunchConfiguration("params_file"),
        replacements={"<robot_namespace>": ("/", namespace)},
    )
    # Replace double "/" in case no namespace was passed
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"//": ("/")},
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("map", "/map")]

    nav2 = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(use_composition),
                name=container_name,
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[{"autostart": autostart}],
                # arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                namespace=namespace,
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_nav2_bringup, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_composition": use_composition,
                    "params_file": params_file,
                    "autostart": autostart,
                    "container_name": container_name,
                    "namespace": namespace,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    return ld
