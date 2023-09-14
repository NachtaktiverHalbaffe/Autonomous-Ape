import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
    DeclareLaunchArgument(
        "plugin_name",
        default_value="abstract_plugin",
        description="The name of the planner plugin which should be tested",
    ),
    DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            get_package_share_directory("sopias4_framework"),
            "assets",
            "global_costmap",
            "global_costmap.yaml",
        ),
        description="Full path to map yaml file to load",
    ),
]


def generate_launch_description():
    ld = LaunchDescription()

    namespace = LaunchConfiguration("namespace")
    plugin_name = LaunchConfiguration("plugin_name")
    log_level = LaunchConfiguration("log_level")

    test_system = Node(
        package="sopias4_framework",
        executable="global_planner_testserver.py",
        name="test_node",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "test_data_path": os.path.join(
                    get_package_share_directory("sopias4_framework"),
                    "assets",
                    "global_costmaps",
                    "global_costmap.json",
                ),
                "plugin_name": plugin_name,
            }
        ],
    )

    map_server = GroupAction(
        [
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sopias4_fleetbroker"),
                            "launch",
                            "map_server.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "map": os.path.join(
                        get_package_share_directory("sopias4_fleetbroker"),
                        "maps",
                        "default_map.yaml",
                    ),
                    "params_file": os.path.join(
                        get_package_share_directory("sopias4_fleetbroker"),
                        "config",
                        "map_server.yaml",
                    ),
                    "use_respawn": "false",
                    "log_level": log_level,
                    "autostart": "true",
                    "namespace": "namespace",
                }.items(),
            ),
        ]
    )

    rviz = GroupAction(
        [
            PushRosNamespace(namespace),
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
                        "rviz_testsystem.rviz",
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
