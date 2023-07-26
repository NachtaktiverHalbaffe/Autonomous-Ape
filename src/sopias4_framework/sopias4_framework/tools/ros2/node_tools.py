#!/usr/bin/env python3
import signal
import subprocess


def start_launch_file(
    ros2_package: str, launch_file: str, arguments: str = ""
) -> subprocess.Popen:
    """
    Runs an ROS2 launch file as a shell process

    Args:
        ros2_package (str): The ROS2 package in which the launch file is located
        launch_file (str): The name of the launchfile
        arguments (str, optional): Launchfile arguments which should be passed. Written in syntax "arg_name:=arg_value"

    Returns:
        subprocess.Popen: The running instance of the shell process
    """
    if arguments != "":
        cmd = f"ros2 launch {ros2_package} {launch_file} {arguments}"
    else:
        cmd = f"ros2 launch {ros2_package} {launch_file}"
    return subprocess.Popen(cmd.split(" "))


def start_node(
    ros2_package: str,
    executable: str,
    ros_params: list[str] = [],
    params_file: str = "",
) -> subprocess.Popen:
    """
    Runs an ROS2 launch file as a shell process

    Args:
        ros2_package (str): The ROS2 package in which the launch file is located
        executable (str): Name of executable which should be run
        ros_params (list(str), optional): List of ROS2 parameters which should be passed
        params_file (str, optional): Path to a yaml configuration file where parameters are located which should be passed to the node

    Returns:
        subprocess.Popen: The running instance of the shell process
    """
    if len(ros_params) != 0:
        cmd = f"ros2 run {ros2_package}  {executable} {ros_params}"
    elif params_file != "":
        cmd = f"ros2 run {ros2_package} {executable} __params:={params_file}"
    else:
        cmd = f"ros2 run {ros2_package} {executable}"
    return subprocess.Popen(cmd.split(" "))


def shutdown_ros_shell_process(shell_process: subprocess.Popen | None) -> bool:
    """
    Shutdown a process which runs over a shell i.e. was called with `subprocess.Popen()`

    Args:
        shell_process (Popen or None): The shell process which should be should down

    Returns:
        bool: Indicating if process was shutdown successfully or not
    """
    if shell_process is not None:
        shell_process.send_signal(signal.SIGINT)
        shell_process.wait(timeout=30)
        shell_process = None
        return True
    else:
        return False


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
