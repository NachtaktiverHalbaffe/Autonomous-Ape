from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    astar = Node(
        package="sopias4_application",
        executable="astar.py",
        name="astar",
        output="screen",
    )

    gui = Node(
        package="sopias4_application",
        executable="gui.py",
        name="gui",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(astar)
    ld.add_action(gui)

    return ld
