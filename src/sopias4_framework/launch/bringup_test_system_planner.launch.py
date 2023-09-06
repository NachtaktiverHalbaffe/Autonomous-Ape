import os

from ament_index_python import get_package_share_directory
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
    DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file to load",
    ),
]


def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    map_yaml_file = LaunchConfiguration("map")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    test_system = Node(
        package="sopias4_framework",
        executable="global_planner_testserver.py",
        name="test_node",
        output="screen",
        parameters=[
            {
                "test_data_path": os.path.join(
                    get_package_share_directory("sopias4_framework"),
                    "assets",
                    "global_costmaps",
                    "global_costmap.json",
                ),
            }
        ],
    )

    map_server = GroupAction(
        condition=LaunchConfigurationNotEquals("map", ""),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=True,
                parameters=[
                    {"yaml_filename": map_yaml_file},
                    {"topic_name": "map"},
                    {"frame_id": "map"},
                ],
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": "true"}, {"node_names": ["map_server"]}],
            ),
        ],
    )

    rviz = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sopias4_framework"),
                            "launch",
                            "rviz.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "params_file": os.path.join(
                        get_package_share_directory("sopias4_framework"),
                        "config",
                        "rviz.rviz",
                    )
                }.items(),
            ),
        ]
    )
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    ld.add_action(map_server)
    ld.add_action(test_system)

    return ld
