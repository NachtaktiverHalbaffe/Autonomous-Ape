import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_simulation",
        default_value="false",
        choices=["true", "false"],
        description="Run turtlebot in simulation instead of connecting to physical robot",
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
]


def generate_launch_description():
    ld = LaunchDescription()

    use_gazebo = LaunchConfiguration("use_simulation")
    namespace = LaunchConfiguration("namespace")
    use_composition = LaunchConfiguration("use_composition")
    log_level = LaunchConfiguration("log_level")
    autostart = LaunchConfiguration("autostart")

    turtlebot4_sim = GroupAction(
        [
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("turtlebot4_ignition_bringup"),
                            "launch",
                            "turtlebot4_ignition.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(use_gazebo),
                launch_arguments=[("namespace", namespace)],
            ),
        ]
    )

    # Localization. It is split out of the nav2_stack so different log levels can be applied without interfering each other
    amcl = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sopias4_framework"),
                            "launch",
                            "amcl.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_composition": "True",
                }.items(),
            ),
        ]
    )

    # Visualization and navigation. These are delayed launched to reduce stress to the network and the nodes before need to fully start anyways
    # before the nav2 stack can complete startup
    nav2_stack = GroupAction(
        [
            TimerAction(
                period=3.0,
                actions=[
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
                        launch_arguments=[("namespace", namespace)],
                    ),
                    TimerAction(
                        period=10.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    PathJoinSubstitution(
                                        [
                                            FindPackageShare("sopias4_framework"),
                                            "launch",
                                            "nav2.launch.py",
                                        ]
                                    )
                                ),
                                launch_arguments={
                                    "namespace": namespace,
                                    "use_composition": use_composition,
                                    "autostart": autostart,
                                    "log_level": log_level,
                                    "params_file": os.path.join(
                                        get_package_share_directory(
                                            "sopias4_application"
                                        ),
                                        "config",
                                        "nav2.yaml",
                                    ),
                                }.items(),
                            ),
                        ],
                    ),
                ],
            ),
        ]
    )
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(amcl)
    ld.add_action(nav2_stack)
    ld.add_action(turtlebot4_sim)

    return ld
