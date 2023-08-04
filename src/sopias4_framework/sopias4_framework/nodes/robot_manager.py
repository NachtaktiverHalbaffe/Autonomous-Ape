#!/usr/bin/env python3
import os
import random
import string
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs import srv as nav2_srv
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.service import Service
from slam_toolbox import srv as slam_toolbox_srv
from sopias4_framework.tools.ros2 import node_tools, yaml_tools
from std_srvs.srv import Empty

from sopias4_msgs.srv import (
    Drive,
    DriveToPos,
    EmptyWithStatuscode,
    LaunchTurtlebot,
    RegistryService,
    ShowDialog,
    StopMapping,
)


class RobotManager(Node):
    """
    A central aspect of the Robot Manager is to initialise and configure SLAM, AMCL, Navigation2 and Turtlebot4. The main task is to assign the namespace of the robot\
    to the nodes and their topics and to start them if necessary. This allows them to be uniquely identified in a multi-robot scenario. Apart from initialization and configuration,\
    this node acts as an interface between the system and the user or GUI. Running tasks is done with the help of services, so this node entirely communicates via ROS2 services.

    Attributes:
        drive_to_pos_action (ActionServer): An action to let the Turtlebot drive to an Position. It is accessed via the action "drive_to_pos" (remember to add the namespace) and\
                                                                        takes a Navigation2 Goal. It's basically a wrapper to pass the action to the Nav2 stack
        drive_service (Service):  A service to send a drive command to the Turtlebot. The service can be accessed via the service "<namespace>/drive"
        launch_service (Service): A service to start/connect to the Turtlebot. The service can be accessed via the service "<namespace>/launch"
        stop_service (Service): A service to stop/disconnect from the Turtlebot. The service can be accessed via the service "<namespace>/stop"
        start_mapping_service (Service): A service to start the mapping. The service can be accessed via the service "<namespace>/start_mapping". It uses the lifecycles in the background to set the slam node into an active state
        stop_mapping_service (Service): A service to stop the mapping. The service can be accessed via the service "<namespace>/stop_mapping". It uses the lifecycles in the background to set the slam node into an inactive state
        dock_service (Service): A service to let the turtlebot dock to its charging station. The service can be accessed via the service "<namespace>/dock"
        undock_service (Service): A service to let the turtlebot dock from its charging station. The service can be accessed via the service "<namespace>/undock"
    """

    def __init__(self, node_name="robot_manager", namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name)  # type: ignore
        else:
            super().__init__(node_name, namespace=namespace)  # type: ignore
        self.__goal_handle = None
        self.__result_future = None

        # ------------------- Service server --------------------
        self.drive_to_pos_action: Service = self.create_service(
            DriveToPos, "drive_to_pos", self.__drive__to_pos
        )
        self.drive_serive: Service = self.create_service(
            Drive, "drive", self.__drive_callback
        )
        self.launch_service: Service = self.create_service(
            LaunchTurtlebot, "connect", self.__launch_nav_stack
        )
        self.stop_service: Service = self.create_service(
            EmptyWithStatuscode, "disconnect", self.__stop_nav_stack
        )
        self.start_mapping_service: Service = self.create_service(
            EmptyWithStatuscode, "start_mapping", self.__start_mapping
        )
        self.stop_mapping_service: Service = self.create_service(
            StopMapping, "stop_mapping", self.__stop_mapping
        )
        self.dock_service: Service = self.create_service(
            EmptyWithStatuscode, "dock", self.__dock
        )
        self.undock_service: Service = self.create_service(
            EmptyWithStatuscode, "undock", self.__undock
        )

        # ------------------- Service clients--------------------
        # Create own sub node for service clients so they can spin independently
        self.__service_client_node: Node = rclpy.create_node("_robot_manager_service_clients", namespace=self.get_namespace())  # type: ignore

        # This service changes the lifecycle of the amcl node. It is used to set the amcl to an inactive state
        # (not operating) when slam is active. Make shure either amcl or slam is actively running, but not both at same time
        self.__amcl_sclient_lifecycle: Client = (
            self.__service_client_node.create_client(
                ChangeState, f"{self.get_namespace()}/amcl/change_state"
            )
        )
        # This service saves the current map in the Sopias4 Map-Server. Used when the mapping is finished to save the map
        # self.__ms_sclient_saveMap: Client = self.__service_client_node.create_client(
        #     slam_toolbox_srv.SaveMap, f"{self.get_namespace()}/slam_toolbox/save_map"
        # )
        self.__ms_sclient_saveMap: Client = self.__service_client_node.create_client(
            nav2_srv.SaveMap, "/map_saver/save_map"
        )
        #  This service allows the robot manager to show a dialog with which the user can interact
        self.__gui_sclient_showDialog: Client = (
            self.__service_client_node.create_client(
                ShowDialog, f"{self.get_namespace()}/show_dialog"
            )
        )
        # This service unregisters the namespace from the Multi roboter coordinator inside Sopias4 Mapserver
        self.__mrc__sclient__unregister = self.__service_client_node.create_client(
            RegistryService, "/unregister_namespace"
        )

        # ------- Action clients ---------------
        # This action lets the robot drive autonomously to an goal position
        self.__nav2_aclient_driveToPos: ActionClient = ActionClient(
            self.__service_client_node,
            NavigateToPose,
            f"{self.get_namespace()}/navigate_to_pose",
        )
        # This action lets the turtlebot dock to its charging station
        self.__tb4_aclient_dock: ActionClient = ActionClient(
            self.__service_client_node, Dock, f"{self.get_namespace()}/dock"
        )
        # This action lets the turtlebot undock from its charging station
        self.__tb4_aclient_undock: ActionClient = ActionClient(
            self.__service_client_node, Undock, f"{self.get_namespace()}/undock"
        )

        # ---------- Publishers ----------------
        self.__cmd_vel_pub = self.create_publisher(
            Twist, f"{self.get_namespace()}/cmd_vel", 10
        )

        # ---------- Shell processes to run nodes ----------
        # The necessary turtlebot and mapping nodes are run inside shell processes via subprocess.popen,
        # because so they can be shutdown cleanly during runtime and they run non-blocking
        self.__turtlebot_shell_process: subprocess.Popen | None = None
        self.__mapping_shell_process: subprocess.Popen | None = None

        # Log level 10 is debug
        self.get_logger().set_level(10)
        self.get_logger().info("Started node")

    def __drive__to_pos(
        self, pose: DriveToPos.Request, response: DriveToPos.Response
    ) -> DriveToPos.Response:
        """
        Callback for a service to let the Turtlebot drive to a given goal position. It's basically a wrapper for the corresponding \
        nav2 action. Its a nice2have service, but navigations goals can also be send directly via the corresponding nav2 action called\
        "navigate_to_pose"

        Args:
            pose (DriveToPose.Request): The data from the service request. Contains the goal pose. \
                                                              Look at service definition in srv/DriveToPos.srv
            response (DriveToPos.Response):  A response object into which the data for the response is written. \
                                                                      Look at service definition in srv/DriveToPos.srv
        
        Returns:
            DriveToPos.Response: The statuscode of the operation
        """
        # Generate action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose.goal
        goal_msg.behavior_tree = ""

        # Wait until action server is up
        while not self.__nav2_aclient_driveToPos.wait_for_server(timeout_sec=1.0):
            self.get_logger().debug(" Waiting for nav2 action server startup...")
            pass

        # Send action goal to action server
        send_goal_future = self.__nav2_aclient_driveToPos.send_goal_async(
            goal_msg, self._feedbackCallback
        )

        # Wait until the goal is either accepted or rejected
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5)
        self.__goal_handle = send_goal_future.result()

        # Check if goal was accepted and setting statuscode for response
        if not self.__goal_handle.accepted:  # type: ignore
            response.statuscode = DriveToPos.Response.GOAL_REJECTED
            return response

        self.__result_future = self.__goal_handle.get_result_async()  # type: ignore
        response.statuscode = DriveToPos.Response.SUCCESS

        return response

    def _feedbackCallback(self, msg: NavigateToPose.Feedback) -> None:
        """
        Feedback callback for the nav2 action client. Currently unused
        """
        pass

    def __drive_callback(
        self, request_data: Drive.Request, response_data: Drive.Response
    ) -> Drive.Response:
        """ 
        Callback function for a service which sends a driving command to the Turtlebot. It basically takes an Twist-Message and publishes it 
        to the right topic with the right namespace.

        Args:
            request_data (Drive.Request): The data from the request. Look at service definition in srv/Drive.srv
            response_data (Drive.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/Drive.srv
        
        Returns:
            Drive.Response: A response which contains the statuscode of the operation\
                                        Look at service definition in srv/Drive.srv
        """
        try:
            self.__cmd_vel_pub.publish(request_data.twist)
            response_data.statuscode = Drive.Response.SUCCESS
        except Exception as e:
            #  Error message for user in Case something goes wrong
            request_informUser = ShowDialog.Request()
            request_informUser.title = "Couldn't send drive command"
            request_informUser.icon = ShowDialog.Request.ICON_ERROR
            request_informUser.interaction_options = ShowDialog.Request.CONFIRM
            request_informUser.content = "Couldn't send drive command. Check if the Turtlebot4\
                        nodes inside Sopias4 Application are running"
            response_data.statuscode = Drive.Response.UNKNOWN_ERROR
            future = self.__gui_sclient_showDialog.call_async(request_informUser)
            rclpy.spin_until_future_complete(
                self.__service_client_node, future, timeout_sec=10
            )
        return response_data

    def __launch_nav_stack(
        self,
        request_data: LaunchTurtlebot.Request,
        response_data: LaunchTurtlebot.Response,
    ) -> LaunchTurtlebot.Response:
        """
        Callback function for a service which launches all nodes related to the robot itself. It basically launches AMCL, Turtlebot4, Rviz2 \
        and Navigation 2 node. If the usage of simulation is specified, the a launch description is generated which runs everything inside \
        a simulation, otherwise a launch description is generated which runs the real world robot. The corresponding launch description \
        is then run inside a own process.

        Args:
            request_data (LaunchTurtlebot.Request): The data from the request. Look at service definition in srv/LaunchTurtlebot.srv
            response_data (Launchturtlebot.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/LaunchTurtlebot.srv
        
        Returns:
            LaunchTurtlebot.Response: A response which contains the statuscode of the operation\
                                                            Look at service definition in srv/LaunchTurtlebot.srv
        """
        self.get_logger().info(
            "Got service request to launch necessary turtlebot nodes"
        )

        if self.__turtlebot_shell_process is None:
            # Launch launchfile
            self.__turtlebot_shell_process = node_tools.start_launch_file(
                ros2_package="sopias4_framework",
                launch_file="bringup_turtlebot.launch.py",
                arguments=f'namespace:={self.get_namespace()} use_simulation:={"true" if request_data.use_simulation  else "false"}',
            )
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            self.get_logger().warning(
                "Couldn't launch turtlebot nodes: Nodes already running"
            )
            # Robot is already running
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_RUNNING

            #  Inform user
            dialog_request = ShowDialog.Request()
            dialog_request.title = "Turtlebot already running"
            dialog_request.content = (
                "The Turtlebot is already running and doesn't need to be started"
            )
            dialog_request.icon = ShowDialog.Request.ICON_INFO
            dialog_request.interaction_options = ShowDialog.Request.CONFIRM

            future = self.__gui_sclient_showDialog.call_async(dialog_request)
            try:
                rclpy.spin_until_future_complete(
                    self.__service_client_node, future, timeout_sec=10
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't spin node while showing error dialog: {e}"
                )
            # Because we only confirm the user, we doen't need to check the response

        self.get_logger().info("Successfully started turtlebot nodes")
        return response_data

    def __stop_nav_stack(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Callback function for a service which stops all nodes related to the robot itself. It basically kills each node except the 
        robot_manager und gui node.

        Args:
            request_data (EmptyWithStatuscode.Request): The data from the request. Look at service definition in srv/EmptyWithStatusCode.srv
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation. \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        # Kill turtlebot nodes by killing the process which runs these
        self.get_logger().info("Got service rquest to stop turtlebot nodes")

        self.get_logger().debug("Shutting down turtlebot nodes")
        if node_tools.shutdown_ros_shell_process(self.__turtlebot_shell_process):
            self.__turtlebot_shell_process = None
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            self.get_logger().warning(
                "Couldn't shutdown turtlebot nodes: Nodes already stopped"
            )
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_STOPPED
            #  Inform user
            dialog_request = ShowDialog.Request()
            dialog_request.title = "Turtlebot already stopped"
            dialog_request.content = "The Turtlebot is already stopped. Was it stopped before or by another component?"
            dialog_request.icon = ShowDialog.Request.ICON_ERROR
            dialog_request.interaction_options = ShowDialog.Request.CONFIRM

            future = self.__gui_sclient_showDialog.call_async(dialog_request)
            try:
                rclpy.spin_until_future_complete(
                    self.__service_client_node, future, timeout_sec=30
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't spin node while showing error dialog: {e}"
                )
            # Because we only confirm the user, we doen't need to check the response

        self.get_logger().info("Successfully shutdown turtlebot nodes")
        return response_data

    def __start_mapping(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        Callback function for a service which starts the mapping. This is done by setting the amcl node to an \
        inactive state so it doesn't operate anymore and starting the slam node.

        Args:
            request_data (EmptyWithStatuscode.Request): The data from the request. Look at service definition in srv/EmptyWithStatusCode.srv
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation.  \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        self.get_logger().info("Got service request to start mapping")

        # ------ Set AMCL in Inactive state -------
        self.get_logger().debug("Setting AMCL to inactive state")
        if ("amcl", self.get_namespace()) in self.get_node_names_and_namespaces():
            request: ChangeState.Request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_DEACTIVATE
            try:
                future = self.__amcl_sclient_lifecycle.call_async(request)
                try:
                    rclpy.spin_until_future_complete(
                        self.__service_client_node, future, timeout_sec=10
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f"Couldn't spin node while starting mapping: {e}"
                    )

                response: ChangeState.Response | None = future.result()
                if response is None:
                    response_data.statuscode = (
                        EmptyWithStatuscode.Response.UNKNOWN_ERROR
                    )
                elif response.success:
                    response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
                else:
                    self.get_logger().warning(
                        "Couldn't set AMCL node in inactive state"
                    )
                    response_data.statuscode = (
                        EmptyWithStatuscode.Response.ALREADY_ACTIVE
                    )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't set AMCL node in inactive state: {e}"
                )

        # ------ start slam toolbox ---------
        self.get_logger().debug("Starting slam nodes")
        if self.__mapping_shell_process is None:
            self.__mapping_shell_process = node_tools.start_launch_file(
                ros2_package="sopias4_framework",
                launch_file="bringup_slam.launch.py",
                arguments=f"namespace:={self.get_namespace()}",
            )
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            self.get_logger().warning(
                "Couldn't start SLAM nodes: Nodes already running"
            )
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_RUNNING
            #  Error message for user in Case something goes wrong
            msg_request = ShowDialog.Request()
            msg_request.title = "Couldn't start mapping"
            msg_request.icon = ShowDialog.Request.ICON_ERROR
            msg_request.interaction_options = ShowDialog.Request.CONFIRM
            msg_request.content = "Couldn't start mapping. Check if the Turtlebot4 Nodes inside Sopias4 Application are running and thats theres no mapping already in progress"
            future_dialog = self.__gui_sclient_showDialog.call_async(msg_request)

            try:
                rclpy.spin_until_future_complete(
                    self.__service_client_node, future_dialog, timeout_sec=10
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't spin node while showing error dialog: {e}"
                )

            return response_data

        self.get_logger().info("Successfully started mapping")
        return response_data

    def __stop_mapping(
        self,
        request_data: StopMapping.Request,
        response_data: StopMapping.Response,
    ) -> StopMapping.Response:
        """
        Callback function for a service which stops the mapping. This is done by setting the amcl node to an \
        active state so it does operate again and stopping the slam node. Furthermore, the map is saved on Sopias4 Map-Server

        Args:
            request_data (StopMappingRequest): The data from the request. Look at service definition in srv/StopMapping.srv
            response_data (StopMapping.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/StopMapping.srv
        
        Returns:
            StopMapping.Response: A response which contains the statuscode of the operation.  \
                                                     Look at service definition in srv/StopMapping.srv
        """
        self.get_logger().info("Got service request to stop mapping")

        #  Save map
        self.get_logger().debug("Saving map")
        # If saving failed, it will already popup a dialog here, so error handling is not necessary here
        response_data = self.__save_map(request_data, response_data)  # type: ignore

        # ------ Stop slam toolbox ---------
        self.get_logger().debug("Stopping slam nodes")

        if node_tools.shutdown_ros_shell_process(self.__mapping_shell_process):
            self.__mapping_shell_process = None
            response_data.statuscode = StopMapping.Response.SUCCESS
        else:
            self.get_logger().warning("Couldn't stop slam nodes: Nodes already stopped")
            response_data.statuscode = StopMapping.Response.ALREADY_STOPPED

        # ------ Set AMCL in active state -------
        self.get_logger().debug("Setting AMCL in active state")
        if ("amcl", self.get_namespace()) in self.get_node_names_and_namespaces():
            request: ChangeState.Request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_ACTIVATE

            try:
                future = self.__amcl_sclient_lifecycle.call_async(request)

                try:
                    rclpy.spin_until_future_complete(
                        self.__service_client_node, future, timeout_sec=10
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f"Couldn't spin node while setting AMCL node to active state: {e}"
                    )

                response: ChangeState.Response | None = future.result()
                if response is None:
                    self.get_logger().warning(
                        "AMCL couldn't be set in active state after stopping mapping: Unknown Error"
                    )
                elif response.success:
                    self.get_logger().debug(
                        "AMCL successfully set in active state after stopping mapping"
                    )
                else:
                    self.get_logger().warning(
                        "Couldn't set AMCL in active state: Node already active"
                    )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't set AMCL in active state after stopping mapping: {e}"
                )

        self.get_logger().info("Successfully stopped mapping")
        return response_data

    def __save_map(
        self, save_params: StopMapping.Request, response_data: StopMapping.Response
    ) -> StopMapping.Response:
        """
        Handles the map saving process itself. If an error occurs, it informs the user and ask if it should be retried

        Args:
            response_data (EmptyWithStatuscode.Response): A response #object into which the data for the response is written. \
                                                                                                Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation.  \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """

        save_map_req: nav2_srv.SaveMap.Request = nav2_srv.SaveMap.Request()
        save_map_req.free_thresh = save_params.free_thres
        save_map_req.occupied_thresh = save_params.occupied_thres
        save_map_req.map_mode = save_params.map_mode
        save_map_req.image_format = save_params.image_format
        save_map_req.map_topic = save_params.map_topic
        save_map_req.map_url = save_params.map_name

        # Add namespace to topic if not given
        if self.get_namespace() not in save_map_req.map_topic:
            save_map_req.map_topic = f"{self.get_namespace()}/{save_map_req.map_topic}"

        self.get_logger().debug(
            "Send service request to save map to Sopias4 Map-Server"
        )
        future = self.__ms_sclient_saveMap.call_async(save_map_req)
        try:
            rclpy.spin_until_future_complete(
                self.__service_client_node, future, timeout_sec=10
            )
        except Exception as e:
            self.get_logger().warning(f"Couldn't spin node while saving map: {e}")

        response: nav2_srv.SaveMap.Response | None = future.result()
        # Check if map was saved successfully
        if response is None:
            self.get_logger().error("Couldn't save map: Unknown reason")
            response_data.statuscode = StopMapping.Response.UNKNOWN_ERROR
        elif response.result:
            response_data.statuscode = StopMapping.Response.SUCCESS
        else:
            response_data.statuscode = StopMapping.Response.SAVING_FAILED

            #  Inform user
            dialog_request = ShowDialog.Request()
            dialog_request.title = "Saving map failed"
            dialog_request.content = (
                "The map couldn't be saved. Check Sopias4 Map-Server"
            )
            dialog_request.icon = ShowDialog.Request.ICON_ERROR
            dialog_request.interaction_options = ShowDialog.Request.IGNORE_RETRY

            future_dialog = self.__gui_sclient_showDialog.call_async(dialog_request)
            try:
                rclpy.spin_until_future_complete(
                    self.__service_client_node, future_dialog
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Couldn't spin node while showing error dialog: {e}"
                )

            user_response = future_dialog.result()
            if user_response is None:
                response_data.statuscode = StopMapping.Response.SAVING_FAILED
            elif user_response.selected_option == ShowDialog.Response.RETRY:
                # User chose to retry operation
                #  Call function recursively to retry save operation
                return self.__save_map(save_params, response_data)
            else:
                #  User chose to ignore error
                response_data.statuscode = StopMapping.Response.SAVING_FAILED

        return response_data

    def __dock(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ):
        """
        Callback function for a service which let the turtlebot dock to its charging station. It uses the underlying action from the icreate3 robot

        Args:
            request_data (EmptyWithStatuscode.Request): The data from the request. Look at service definition in srv/EmptyWithStatusCode.srv
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation. \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        goal_msg = Dock.Goal()
        self.__tb4_aclient_dock.wait_for_server()

        future = self.__tb4_aclient_dock.send_goal_async(goal_msg)
        try:
            rclpy.spin_until_future_complete(self.__service_client_node, future)
        except Exception as e:
            self.get_logger().warning(f"Couldn't spin node while docking: {e}")

        response: Dock.Result | None = future.result()
        if response is None:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR
        elif response:
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR

        return response_data

    def __undock(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ):
        """
        Callback function for a service which let the turtlebot undock from its charging station. It uses the underlying action from the icreate3 robot

        Args:
            request_data (EmptyWithStatuscode.Request): The data from the request. Look at service definition in srv/EmptyWithStatusCode.srv
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation. \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        goal_msg = Undock.Goal()
        self.__tb4_aclient_dock.wait_for_server()

        future = self.__tb4_aclient_undock.send_goal_async(goal_msg)
        try:
            rclpy.spin_until_future_complete(self.__service_client_node, future)
        except Exception as e:
            self.get_logger().warning(f"Couldn't spin node while docking: {e}")

        response: Undock.Result | None = future.result()
        if response is None:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR
        elif response:
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR

        return response_data

    def destroy_node(self):
        """
        Clear up tasks when the node gets destroyed by e.g. a shutdown. Mainly releasing all service and action clients
        :meta private:
        """
        self.get_logger().info("Shutting down node")
        # Unregister namespace
        request = RegistryService.Request()
        request.name_space = self.get_namespace()
        self.__mrc__sclient__unregister.call_async(request)

        node_tools.shutdown_ros_shell_process(self.__turtlebot_shell_process)
        node_tools.shutdown_ros_shell_process(self.__mapping_shell_process)

        super().destroy_node()


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of RobotManager
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    node = RobotManager(namespace="".join(random.choices(string.ascii_lowercase, k=8)))
    # Run node
    rclpy.spin(node)
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
