from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_simulation",
        default_value="false",
        choices=["true", "false"],
        description="Run turtlebot in simulation instead of connecting to physical robot",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def generate_launch_description():
    ld = LaunchDescription()

    use_gazebo = LaunchConfiguration("use_simulation")
    namespace = LaunchConfiguration("namespace")

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
            ),
        ]
    )

    turtlebot4_real = Node(
        package="turtlebot4_node",
        executable="turtlebot4_node",
        name="turtlebot4_node",
        namespace=namespace,
        condition=IfCondition(NotSubstitution(use_gazebo)),
    )

    amcl = GroupAction(
        [
            PushRosNamespace(namespace),
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
            ),
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
    )

    nav2 = GroupAction(
        [
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sopias4_framework"),
                            "launch",
                            "nav2.launch.py",
                        ]
                    )
                )
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    ld.add_action(rviz2)
    ld.add_action(amcl)
    ld.add_action(turtlebot4_sim)
    ld.add_action(turtlebot4_real)

    return ld
