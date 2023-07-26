from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "sync",
        default_value="true",
        choices=["true", "false"],
        description="Use synchronous SLAM",
    ),
    DeclareLaunchArgument("namespace", default_value="", description="Robot namespace"),
]


def generate_launch_description():
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("turtlebot4_navigation"),
                    "launch",
                    "slam.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "sync": LaunchConfiguration("sync"),
        }.items(),
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("turtlebot4_viz"),
                    "launch",
                    "view_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam)
    ld.add_action(rviz2)

    return ld
