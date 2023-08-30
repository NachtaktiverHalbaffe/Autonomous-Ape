import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("sopias4_application"), "config", "nav2.yaml"
        ),
        description="Nav2 parameters",
    )

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

    nav2 = GroupAction(
        [
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_nav2_bringup, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_composition": "False",
                    "params_file": params_file,
                }.items(),
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_params_arg)
    ld.add_action(nav2)
    return ld
