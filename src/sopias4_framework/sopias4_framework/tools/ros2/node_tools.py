#!/usr/bin/env python3
import signal
import subprocess
from typing import Any

import rclpy
from rclpy.client import Client
from rclpy.node import Node


class LaunchService:
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

        self.shell_session: subprocess.Popen | None = None

    def __del__(self):
        self.shutdown()

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()

    def start(self) -> bool:
        if self.launch_file is not None:
            return self.__start_launchfile()
        elif self.executable is not None:
            return self.__start_node()
        else:
            print("Neither a launch file nor a executabel is specified for launching")
            return False

    def add_launchfile_arguments(self, arguments: str):
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
        if self.shell_session is not None:
            print("Launchfile is already running. Skipping start")
            return False

        if self.arguments is not None:
            cmd = f"ros2 launch {self.package} {self.launch_file} {self.arguments}"
        else:
            cmd = f"ros2 launch {self.package} {self.launch_file}"

        self.shell_session = subprocess.Popen(cmd.split(" "))
        return True

    def __start_node(
        self,
    ) -> bool:
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
        if self.shell_session is not None:
            print("Launchfile is already running. Skipping start")
            return False

        if self.ros_params is not None:
            cmd = f"ros2 run {self.package}  {self.executable} {self.ros_params}"
        elif self.params_file is not None:
            cmd = f"ros2 run {self.package} {self.executable} __params:={self.params_file}"
        else:
            cmd = f"ros2 run {self.package} {self.executable}"

        self.shell_session = subprocess.Popen(cmd.split(" "))
        return True

    def shutdown(self) -> bool:
        """
        Shutdown a process which runs over a shell i.e. was called with `subprocess.Popen()`

        Returns:
            bool: Indicating if process was shutdown successfully or not
        """
        if self.shell_session is not None:
            self.shell_session.send_signal(signal.SIGINT)
            self.shell_session.wait(timeout=30)
            self.shell_session = None
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
