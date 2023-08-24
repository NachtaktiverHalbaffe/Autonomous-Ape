#!/usr/bin/env python3
import signal
import subprocess
from typing import Any

import rclpy
from rclpy.client import Client
from rclpy.node import Node


class LaunchService:
    """
    This launch service is capable of launching either ROS2 launch files or nodes. It runs them in a own shell session in the \
    background which enables it running it in a non blocking manner. Also they can dynamically be started and shutdown. It\
    should also be robust against programm crashes so that no zombie processes continues running in the baclground if Sopias4\
    crashes due to some reason.

    Attributes:
        ros2_package (str): Name of the ROS2 package in which the launch file or the node is located
        launch_file (str, optional): Name of the launchfile which should be used. Dont forget the file extensions i.e. .py etc
        executable (str, optional): Name of the node (the executable) which should be run
        launch_file_arguments (str, optional): All the arguments which should be used when launching.  Only used when a launch file is run
        ros_params (list(str), optional): Params which should be passed to the node. Each item is a key-value pair e.g. "param1:=value". Only uses when a node is run
        params_file (str, optional): Path to a YAML-File with parameters in it. Only used when a node is run
    """

    def __init__(
        self,
        ros2_package: str,
        launch_file: str = "",
        executable: str = "",
        launch_file_arguments: str = "",
        ros_params: list[str] = [],
        params_file: str = "",
    ):
        self.package: str = ros2_package
        # Launch file params
        self.launch_file: str | None = launch_file if launch_file != "" else None
        self.arguments: str | None = (
            launch_file_arguments if launch_file_arguments != "" else None
        )
        # Node param
        self.executable: str | None = executable if executable != "" else None
        self.ros_params: list[str] | None = ros_params if len(ros_params) != 0 else None
        self.params_file: str | None = params_file if params_file != "" else None

        self.__shell_session: subprocess.Popen | None = None

    def __del__(self):
        self.shutdown()

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()

    def run(self) -> bool:
        """
        Runs the launch_file or node. It automatically detects if a launch_file or a node is run by checking the passed arguments of the class
        """
        if self.launch_file is not None:
            return self.__start_launchfile()
        elif self.executable is not None:
            return self.__start_node()
        else:
            print("Neither a launch file nor a executabel is specified for launching")
            return False

    def add_launchfile_arguments(self, arguments: str):
        """
        Add launchfile arguments to the launch service. Only needed/used when a launch file is run
        """
        self.arguments = arguments

    def __start_launchfile(self) -> bool:
        """
        Runs an ROS2 launch file as a shell process

        Args:
            ros2_package (str): The ROS2 package in which the launch file is located
            launch_file (str): The name of the launchfile
            arguments (str, optional): Launchfile arguments which should be passed. Written in syntax "arg_name:=arg_value"

        Returns:
            subprocess.Popen: The running instance of the shell process
        """
        if self.__shell_session is not None:
            print("Launchfile is already running. Skipping start")
            return False

        if self.arguments is not None:
            cmd = f"ros2 launch {self.package} {self.launch_file} {self.arguments}"
        else:
            cmd = f"ros2 launch {self.package} {self.launch_file}"

        self.__shell_session = subprocess.Popen(cmd.split(" "))
        return True

    def __start_node(
        self,
    ) -> bool:
        """
        Runs an ROS2 launch file as a shell process

        Returns:
            subprocess.Popen: The running instance of the shell process
        """
        if self.__shell_session is not None:
            print("Launchfile is already running. Skipping start")
            return False

        if self.ros_params is not None:
            cmd = f"ros2 run {self.package}  {self.executable} {self.ros_params}"
        elif self.params_file is not None:
            cmd = f"ros2 run {self.package} {self.executable} __params:={self.params_file}"
        else:
            cmd = f"ros2 run {self.package} {self.executable}"

        self.__shell_session = subprocess.Popen(cmd.split(" "))
        return True

    def shutdown(self) -> bool:
        """
        Shutdown a the running launch file or node

        Returns:
            bool: Indicating if process was shutdown successfully or not (It also returns False if the launchfile was already stopped/is not running)
        """
        if self.__shell_session is not None:
            self.__shell_session.send_signal(signal.SIGINT)
            self.__shell_session.wait(timeout=30)
            self.__shell_session = None
            return True
        else:
            return False


def call_service(
    client: Client, service_req: Any, calling_node: Node, timeout_sec: float = 60.0
) -> Any | None:
    """
    Calls an ROS2 service asynchronously and returns the response

    Args:
        client (rclpy.Client): The ROS2 service client which calls the service
        service_req (Any): The service request that should be sent. Must match the service type of the message
        calling_node (rclpy.Node): The ROS2 node of the service client which calls the service under the hood
        timeout_sec (float, optional): Second until the request should timeout if no response was received

    Returns:
        None or response type of service: The response from the service
    """
    if client.service_is_ready():
        # Call the service
        try:
            future = client.call_async(service_req)
        except TypeError:
            raise TypeError(
                f"Service type {type(service_req)} doesn't match required service type {client.srv_type}"
            )

        # Spin calling node until response is available
        try:
            calling_node.get_logger().debug("Spinning with global executor")
            rclpy.spin_until_future_complete(
                calling_node, future, timeout_sec=timeout_sec
            )
        except Exception as e:
            calling_node.get_logger().warning(
                f"Couldn't spin calling node during calling a service: {e}"
            )
        calling_node.get_logger().debug(f"Got service response {future.result()}")
        return future.result()
    else:
        calling_node.get_logger().warning(
            "Couldnt call service: Theres no service available"
        )
        return None


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
