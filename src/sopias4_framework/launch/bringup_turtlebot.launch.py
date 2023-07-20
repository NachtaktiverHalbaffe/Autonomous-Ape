from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            # PushRosNamespace(namespace),
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
                launch_arguments=[("namespace", namespace)],
            ),
        ]
    )

    rviz2 = GroupAction(
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
                launch_arguments=[("namespace", namespace)],
            ),
        ]
    )

    nav2 = GroupAction(
        [
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
                launch_arguments=[("namespace", namespace)],
            ),
        ]
    )

    tf_relay = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sopias4_framework"),
                            "launch",
                            "tf_relay.launch.py",
                        ]
                    )
                ),
                launch_arguments=[("namespace", namespace)],
            ),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    ld.add_action(rviz2)
    ld.add_action(amcl)
    ld.add_action(turtlebot4_sim)
    ld.add_action(tf_relay)

    return ld
