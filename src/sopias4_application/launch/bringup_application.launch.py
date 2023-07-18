from launch import LaunchDescription
from launch_ros.actions import Node


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

    robot_layer = Node(
        package="sopias4_application",
        executable="robot_layer.py",
        name="robot_layer",
        output="screen",
    )

    path_layer = Node(
        package="sopias4_application",
        executable="path_layer.py",
        name="path_layer",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(astar)
    ld.add_action(gui)
    ld.add_action(robot_layer)
    ld.add_action(path_layer)

    return ld
