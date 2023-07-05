#!/usr/bin/env python3
import asyncio
import os
import signal
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from rclpy.action import ActionClient, ActionServer
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service

from sopias4_msgs.srv import (
    Drive,
    DriveToPos,
    EmptyWithStatuscode,
    GetNamespaces,
    GetRobotIdentity,
    GetRobots,
    LaunchTurtlebot,
    ShowDialog,
    StopMapping,
    Unregister,
)


class RobotManager(Node):
    """
    A central aspect of the Robot Manager is to initialise and configure SLAM, AMCL, Navigation2 and Turtlebot4. The main task is to assign the namespace of the robot\
    to the nodes and their topics and to start them if necessary. This allows them to be uniquely identified in a multi-robot scenario. Apart from initialization and configuration,\
    this node acts as an interface between the system and the user or GUI. Running tasks is done with the help of services, so this node entirely communicates via ROS2 services.

    Attributes:
        other_robots (list<Robot>): A list containing the state of the other robots
        drive_to_pos_action (ActionServer): An action to let the Turtlebot drive to an Position. It is accessed via the action "drive_to_pos" (remember to add the namespace) and\
                                                                        takes a Navigation2 Goal. It's basically a wrapper to pass the action to the Nav2 stack
        drive_service (Service):  A service to send a drive command to the Turtlebot. The service can be accessed via the service "<namespace>/drive"
        launch_service (Service): A service to start/connect to the Turtlebot. The service can be accessed via the service "<namespace>/launch"
        stop_service (Service): A service to stop/disconnect from the Turtlebot. The service can be accessed via the service "<namespace>/stop"
        start_mapping_service (Service): A service to start the mapping. The service can be accessed via the service "<namespace>/start_mapping". It uses the lifecycles in the background to set the slam node into an active state
        stop_mapping_service (Service): A service to stop the mapping. The service can be accessed via the service "<namespace>/stop_mapping". It uses the lifecycles in the background to set the slam node into an inactive state
    """

    # TODO Keep doc up to date
    def __init__(self, node_name="robot_manager", namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name)  # type: ignore
        else:
            super().__init__(node_name, namespace=namespace)  # type: ignore
        self.other_robots: list = []
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
            LaunchTurtlebot, "launch", self.__launch_robot
        )
        self.stop_service: Service = self.create_service(
            EmptyWithStatuscode, "stop", self.__stop_robot
        )
        self.start_mapping_service: Service = self.create_service(
            EmptyWithStatuscode, "start_mapping", self.__start_mapping
        )
        self.stop_mapping_service: Service = self.create_service(
            StopMapping, "stop_mapping", self.__stop_mapping
        )

        # ------------------- Service clients--------------------
        # Create own sub node for service clients so they can spin independently
        self.__service_client_node: Node = rclpy.create_node("_robot_manager_service_clients", namespace=self.get_namespace())  # type: ignore
        # This service requests the states of the robot from the multi robot coordinator inside Sopias4 Map-Server
        self.__mrc_sclient_robots: Client = self.__service_client_node.create_client(
            GetRobots, "/get_robots"
        )
        self.__mrc_sclient_robot_identity: Client = (
            self.__service_client_node.create_client(
                GetRobotIdentity, "/get_robot_identity"
            )
        )
        # This service requests all registered namespaces of multi robot coordinator inside Sopias4 Map-Server
        self.__mrc_sclient_namespaces: Client = (
            self.__service_client_node.create_client(GetNamespaces, "/get_namespaces")
        )
        # This service changes the lifecycle of the amcl node. It is used to set the amcl to an inactive state
        # (not operating) when slam is active. Make shure either amcl or slam is actively running, but not both at same time
        self.__amcl_sclient_lifecycle: Client = (
            self.__service_client_node.create_client(
                ChangeState, f"{self.get_namespace()}/amcl/change_state"
            )
        )
        # This action lets the robot drive autonomously to an goal position
        self.__nav2_aclient_driveToPos: ActionClient = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        # This service saves the current map in the Sopias4 Map-Server. Used when the mapping is finished to save the map
        self.__ms_sclient_saveMap: Client = self.__service_client_node.create_client(
            SaveMap, "map_server/save_map"
        )
        #  This service allows the robot manager to show a dialog with which the user can interact
        self.__gui_sclient_showDialog: Client = (
            self.__service_client_node.create_client(
                ShowDialog, f"{self.get_namespace()}/show_dialog"
            )
        )
        # This service unregisters the namespace from the Multi roboter coordinator inside Sopias4 Mapserver
        self.__mrc__sclient__unregister = self.__service_client_node.create_client(
            Unregister, "/unregister_namespace"
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

        self.get_logger().info("Started node")

    def __drive__to_pos(
        self, pose: DriveToPos.Request, response: DriveToPos.Response
    ) -> DriveToPos.Response:
        """
         :meta public:
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
        rclpy.spin_until_future_complete(self, send_goal_future)
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
         :meta public:
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
            response_data.statuscode = Drive.Response.UNKNOWN_ERROR
        return response_data

    def __launch_robot(
        self,
        request_data: LaunchTurtlebot.Request,
        response_data: LaunchTurtlebot.Response,
    ) -> LaunchTurtlebot.Response:
        """
        :meta public:
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
        if self.__turtlebot_shell_process is None:
            # Launch launchfile
            cmd = f'ros2 launch sopias4_framework bringup_turtlebot.launch.py namespace:={self.get_namespace()} use_simulation:={"true" if request_data.use_simulation  else "false"}'
            self.__turtlebot_shell_process = subprocess.Popen(cmd.split(" "))
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
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

            self.__gui_sclient_showDialog.call_async(dialog_request)
            # Because we only confirm the user, we doen't need to check the response

        return response_data

    def __stop_robot(
        self,
        _: EmptyWithStatuscode.Request,
        response_data: EmptyWithStatuscode.Response,
    ) -> EmptyWithStatuscode.Response:
        """
        :meta public:
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

        if self.__shutdown_shell_process(self.__turtlebot_shell_process):
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_STOPPED
            #  Inform user
            dialog_request = ShowDialog.Request()
            dialog_request.title = "Turtlebot already stopped"
            dialog_request.content = "The Turtlebot is already stopped. Was it stopped before or by another component?"
            dialog_request.icon = ShowDialog.Request.ICON_INFO
            dialog_request.interaction_options = ShowDialog.Request.CONFIRM

            future = self.__gui_sclient_showDialog.call_async(dialog_request)
            rclpy.spin_until_future_complete(self.__service_client_node, future)
            # Because we only confirm the user, we doen't need to check the response

        # Kill slam nodes by killing the process which runs these. Because slam is not running every time,
        # no error status response is generated when node is already stopped
        self.__shutdown_shell_process(self.__mapping_shell_process)

        #  Unregister namespace so it can be used again
        unregister_request = Unregister.Request()
        unregister_request.name_space = self.get_namespace()

        future = self.__mrc__sclient__unregister.call_async(unregister_request)
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        response: Unregister.Response | None = future.result()
        if response is None:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR
        elif response.statuscode == Unregister.Response.SUCCESS:
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
        :meta public:
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
        # ------ Set AMCL in Inactive state -------
        request: ChangeState.Request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self.__amcl_sclient_lifecycle.call_async(request)

        rclpy.spin_until_future_complete(self.__service_client_node, future)
        response: ChangeState.Response | None = future.result()
        if response is None:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR
        elif response.success:
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_ACTIVE

        # ------ start slam toolbox ---------
        if self.__mapping_shell_process is not None:
            cmd = f"ros2 launch sopiaf4_framework slam.launch.py namespace:={self.get_namespace()}"
            self.__mapping_shell_process = subprocess.Popen(cmd.split(" "))
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_RUNNING

        return response_data

    def __stop_mapping(
        self,
        request_data: StopMapping.Request,
        response_data: StopMapping.Response,
    ) -> StopMapping.Response:
        """
        :meta public:
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
        # ------ Stop slam toolbox ---------
        if self.__shutdown_shell_process(self.__mapping_shell_process):
            response_data.statuscode = StopMapping.Response.SUCCESS
        else:
            response_data.statuscode = StopMapping.Response.ALREADY_STOPPED

        # ------ Set AMCL in active state -------
        request: ChangeState.Request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        future = self.__amcl_sclient_lifecycle.call_async(request)

        rclpy.spin_until_future_complete(self.__service_client_node, future)
        response: ChangeState.Response | None = future.result()
        if response is None:
            response_data.statuscode = EmptyWithStatuscode.Response.UNKNOWN_ERROR
        elif response.success:
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_ACTIVE
            return response_data

        #  Save map
        response_data = self.__save_map(request_data, response_data)  # type: ignore

        return response_data

    def __save_map(
        self, save_params: StopMapping.Request, response_data: StopMapping.Response
    ) -> StopMapping.Response:
        """
        Handles the map saving process itself. If an error occurs, it informs the user and ask if it should be retried

        Args:
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                                Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation.  \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        save_map_requ = SaveMap.Request()
        save_map_requ.map_topic = save_params.map_topic
        save_map_requ.map_url = save_params.map_name
        save_map_requ.map_mode = save_params.map_mode
        save_map_requ.free_thresh = save_params.free_thres
        save_map_requ.occupied_thresh = save_params.occupied_thres
        # save_map_requ.image_format = save_params.image_format

        future = self.__ms_sclient_saveMap.call_async(save_map_requ)
        rclpy.spin_until_future_complete(self.__service_client_node, future)
        response: SaveMap.Response | None = future.result()
        # Check if map was saved successfully
        if response is None:
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

            user_response: ShowDialog.Response = self.__gui_sclient_showDialog.call(
                dialog_request
            )

            # User chose to retry operation
            if user_response.selected_option == ShowDialog.Response.RETRY:
                #  Call function recursively to retry save operation
                return self.__save_map(save_params, response_data)
            #  User chose to ignore error
            else:
                response_data.statuscode = StopMapping.Response.SAVING_FAILED

        return response_data

    def __shutdown_shell_process(self, shell_process: subprocess.Popen | None) -> bool:
        """
        Shutdown a process which runs over a shell i.e. was called with `subprocess.Popen()`

        Args:
            shell_process (Popen or None): The shell process which should be should down

        Returns:
            bool: Indicating if process was shutdown successfully or not
        """
        if shell_process is not None:
            shell_process.send_signal(signal.SIGINT)
            shell_process.wait()
            shell_process = None
            return True
        else:
            return False

    def destroy_node(self):
        """
        self.__service_client_node.destroy_node()
        Clear up tasks when the node gets destroyed by e.g. a shutdown. Mainly releasing all service and action clients
        :meta private:
        """
        # Unregister namespace
        request = Unregister.Request()
        request.name_space = self.get_namespace()
        self.__mrc__sclient__unregister.call_async(request)
        # Release service clients
        self.__mrc_sclient_namespaces.destroy()
        self.__mrc_sclient_robot_identity.destroy()
        self.__mrc_sclient_robots.destroy()
        self.__amcl_sclient_lifecycle.destroy()
        self.__nav2_aclient_driveToPos.destroy()
        self.__service_client_node.destroy_node()
        # Release services
        self.launch_service.destroy()
        self.stop_service.destroy()
        self.start_mapping_service.destroy()
        self.stop_mapping_service.destroy()
        self.drive_to_pos_action.destroy()
        # Shutdown processes
        self.__shutdown_shell_process(self.__turtlebot_shell_process)
        self.__shutdown_shell_process(self.__mapping_shell_process)

        self.get_logger().info("Shutting down node")
        super().destroy_node()


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
