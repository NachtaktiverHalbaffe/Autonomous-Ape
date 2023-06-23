#!/usr/bin/env python3
import asyncio
from multiprocessing import Process
from time import sleep

import launch_ros.actions
import osrf_pycommon
import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from osrf_pycommon import process_utils
from rclpy.action import ActionClient, ActionServer
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service

from sopias4_framework.srv import (
    EmptyWithStatuscode,
    GetNamespaces,
    GetRobotIdentity,
    GetRobots,
)


class RobotManager(Node):
    """
    A central aspect of the Robot Manager is to initialise and configure SLAM, AMCL, Navigation2 and Turtlebot4. The main task is to assign the namespace of the robot
    to the nodes and their topics and to start them if necessary. This allows them to be uniquely identified in a multi-robot scenario. Apart from initialisation and configuration,
    this node acts as an interface between the system and the user or GUI.

    Attributes:
        drive_to_pos_action (ActionServer): An action to let the Turtlebot drive to an Position. It is accessed via the action "drive_to_pos" (remember to add the namespace) and
                                                                        takes a Navigation2 Goal. It's basically a wrapper to pass the action to the Nav2 stack
        drive_service (Service):  A service to send a drive command to the Turtlebot. The service can be accessed via the service "<namespace>/drive"
        rotate_service (Service): A service to send a rotatecommand to the Turtlebot. The service can be accessed via the service "<namespace>/rotate"
        launch_service (Service): A service to start/connect to the Turtlebot. The service can be accessed via the service "<namespace>/launch"
        stop_service (Service): A service to stop/disconnect from the Turtlebot. The service can be accessed via the service "<namespace>/stop"
        start_mapping_service (Service): A service to start the mapping. The service can be accessed via the service "<namespace>/start_mapping". It uses the lifecycles in the background to set the slam node into an active state
        stop_mapping_service (Service): A service to stop the mapping. The service can be accessed via the service "<namespace>/stop_mapping". It uses the lifecycles in the background to set the slam node into an inactive state
    """

    # TODO Keep doc up to date
    def __init__(self, node_name="robot_manager") -> None:
        super().__init__(node_name)  # type: ignore
        self.other_robots: list = []

        # ------------------- Service server --------------------
        self.drive_to_pos_action: ActionServer  # TODO look if it can be left out
        self.drive_service: Service
        self.rotate_service: Service
        self.launch_service: Service = self.create_service(
            EmptyWithStatuscode, "launch", self.__launch_robot
        )
        self.stop_service: Service = self.create_service(
            EmptyWithStatuscode, "stop", self.__stop_robot
        )
        self.start_mapping_service: Service = self.create_service(
            EmptyWithStatuscode, "start_mapping", self.__start_mapping
        )
        self.stop_mapping_service: Service = self.create_service(
            EmptyWithStatuscode, "stop_mapping", self.__stop_mapping
        )

        # ------------------- Service clients--------------------
        self.__mrc_sclient_robots: Client = self.create_client(GetRobots, "get_robots")
        self.__mrc_sclient_robot_identity: Client = self.create_client(
            GetRobotIdentity, "get_robot_identity"
        )
        self.__mrc_sclient_namespaces: Client = self.create_client(
            GetNamespaces, "get_namespaces"
        )
        self.__amcl_sclient_lifecycle: Client = self.create_client(
            ChangeState, f"{self.get_namespace()}amcl/change_state"
        )
        self.__nav2_aclient_driveToPos: ActionClient

        # ---------- Launch services to launch nodes----------
        # Turtlebot launch stuff
        self.__ld_robot = self.__generate_launch_description()
        self.__ls_robot = LaunchService()
        self.__ls_robot.include_launch_description(self.__ld_robot)

        # SLAM/Mapping Launch stuff
        self.__ld_mapping = LaunchDescription()
        self.__ld_mapping.add_action(
            GroupAction(
                [
                    PushRosNamespace(self.get_namespace()),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("slam_toolbox"),
                                    "launch",
                                    "online_sync_launch.py",
                                ]
                            )
                        )
                    ),
                ]
            )
        )
        self.__ls_mapping = LaunchService()
        self.__ls_mapping.include_launch_description(self.__ld_mapping)

        # Running the launchfiles is done via processes
        self.__runRobot = Process(target=self.__ls_robot.run, daemon=True)
        self.__runMapping = Process(target=self.__ls_mapping.run, daemon=True)

    def __launch_robot(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Launches all nodes related to the robot itself
        """
        if not self.__runRobot.is_alive():
            self.__runRobot.start()
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_RUNNING

        return response_data

    def __stop_robot(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Stops all nodes related to the robot itself
        """
        if self.__runRobot.is_alive():
            self.__runRobot.kill()
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_STOPPED

        return response_data

    def __start_mapping(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Stops all nodes related to the robot itself
        """
        # ------ Set AMCL in Inactive state -------
        request: ChangeState.Request = ChangeState.Request()
        request.transition = Transition.TRANSITION_DEACTIVATE
        future = self.__amcl_sclient_lifecycle.call_async(request)

        # Make sure the node itself is spinnig
        while rclpy.ok():
            if future.done():
                try:
                    if future.result():
                        response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
                    else:
                        response_data.statuscode = (
                            EmptyWithStatuscode.Response.UNKNOWN_ERROR
                        )
                        return response_data
                except Exception as e:
                    raise e

        # ------ start slam toolbox ---------
        if not self.__runMapping.is_alive():
            self.__runMapping.start()
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_RUNNING

        return response_data

    def __stop_mapping(
        self,
        _request_data: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Stops all nodes related to the robot itself
        """
        # ------ Stop slam toolbox ---------
        if self.__runMapping.is_alive():
            self.__runMapping.kill()
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_STOPPED

        # ------ Set AMCL in active state -------
        request: ChangeState.Request = ChangeState.Request()
        request.transition = Transition.TRANSITION_ACTIVATE
        future = self.__amcl_sclient_lifecycle.call_async(request)

        # Make sure the node itself is spinnig
        while rclpy.ok():
            if future.done():
                try:
                    if future.result():
                        response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
                    else:
                        response_data.statuscode = (
                            EmptyWithStatuscode.Response.UNKNOWN_ERROR
                        )
                        return response_data
                except Exception as e:
                    raise e

        return response_data

    def __generate_launch_description(self) -> LaunchDescription:
        # Define the different components which should be launched
        turtlebot4 = launch_ros.actions.Node(
            package="turtlebot4_node",
            executable="turtlebot4_node",
            name="turtlebot4_node",
            namespace=self.get_namespace(),
        )

        amcl = launch_ros.actions.Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=self.get_namespace(),
        )

        rviz2 = launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=self.get_namespace(),
        )

        nav2 = GroupAction(
            [
                PushRosNamespace(self.get_namespace()),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nav2_bringup"),
                                "launch",
                                "navigation_launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "use_composition": "False",
                        # "params_file": LaunchConfiguration("params_file"),
                    }.items(),
                ),
            ]
        )

        # TODO add other descriptions/nodes
        ld = LaunchDescription()
        ld.add_action(turtlebot4)
        ld.add_action(amcl)
        ld.add_action(nav2)
        return ld


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of RobotManager
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    node = RobotManager()
    # Run node
    rclpy.spin(node)
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
