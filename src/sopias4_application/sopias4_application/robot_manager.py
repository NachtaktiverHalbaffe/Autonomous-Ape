#!/usr/bin/env python3
import asyncio
from multiprocessing import Process
from time import sleep

import launch_ros.actions
import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from rclpy.action import ActionClient, ActionServer
from rclpy.client import Client
from rclpy.node import Node
from rclpy.service import Service

from sopias4_framework.srv import (
    DriveToPos,
    EmptyWithStatuscode,
    GetNamespaces,
    GetRobotIdentity,
    GetRobots,
    LaunchTurtlebot,
    ShowDialog,
)


class RobotManager(Node):
    """
    A central aspect of the Robot Manager is to initialise and configure SLAM, AMCL, Navigation2 and Turtlebot4. The main task is to assign the namespace of the robot
    to the nodes and their topics and to start them if necessary. This allows them to be uniquely identified in a multi-robot scenario. Apart from initialisation and configuration,
    this node acts as an interface between the system and the user or GUI.

    Attributes:
        other_robots (list<Robot>): A list containing the state of the other robots
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
        self.__goal_handle = None
        self.__result_future = None

        # ------------------- Service server --------------------
        self.drive_to_pos_action: Service = self.create_service(
            DriveToPos, "drive_to_pos", self.__drive__to_pos
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
        # TODO maybe own service definition with custom map
        self.stop_mapping_service: Service = self.create_service(
            EmptyWithStatuscode, "stop_mapping", self.__stop_mapping
        )

        # ------------------- Service clients--------------------
        # This service requests the states of the robot from the multi robot coordinator inside Sopias4 Map-Server
        self.__mrc_sclient_robots: Client = self.create_client(GetRobots, "get_robots")
        self.__mrc_sclient_robot_identity: Client = self.create_client(
            GetRobotIdentity, "get_robot_identity"
        )
        # This service requests all registered namespaces of multi robot coordinator inside Sopias4 Map-Server
        self.__mrc_sclient_namespaces: Client = self.create_client(
            GetNamespaces, "get_namespaces"
        )
        # This service changes the lifecycle of the amcl node. It is used to set the amcl to an inactive state
        # (not operating) when slam is active. Make shure either amcl or slam is actively running, but not both at same time
        self.__amcl_sclient_lifecycle: Client = self.create_client(
            ChangeState, f"{self.get_namespace()}/amcl/change_state"
        )
        # This action lets the robot drive autonomously to an goal position
        self.__nav2_aclient_driveToPos: ActionClient = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        # This service saves the current map in the Sopias4 Map-Server. Used when the mapping is finished to save the map
        self.__ms_sclient_saveMap: Client = self.create_client(
            SaveMap, "map_server/save_map"
        )
        #  This service allows the robot manager to show a dialog with which the user can interact
        self.__gui_sclient_showDialog: Client = self.create_client(
            ShowDialog, f"{self.get_namespace()}/show_dialog"
        )

        # ---------- Launch services to launch nodes----------
        # Turtlebot launch description and service for launching corresponding node
        self.__ld_robot = self.__generate_launch_description()
        self.__ls_robot = LaunchService()

        # SLAM/Mapping launch description and service for launching slam node
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

        # Running the launchfiles is done via processes because they can be killed and are non-blocking.
        # This gives us the control to shutdown the corresponding nodes without shutting down the robot manager itself
        self.__runRobot = Process(target=self.__ls_robot.run, daemon=True)
        self.__runMapping = Process(target=self.__ls_mapping.run, daemon=True)

        self.get_logger().info("[Robot Manager] Started node")

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
            self.get_logger().debug(
                "[ROBOT MANAGER] Waiting for nav2 action server startup..."
            )
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
        if not self.__runRobot.is_alive():
            # Generate launch files
            if request_data.use_simulation:
                # Use simulation/gazebo
                self.__ld_robot = self.__generate_launch_description(use_gazebo=True)
            else:
                # Use real turtlebot
                self.__ld_robot = self.__generate_launch_description(use_gazebo=False)

            # Launch launchfile
            self.__ls_robot = LaunchService()
            self.__ls_robot.include_launch_description(self.__ld_robot)
            self.__runRobot = Process(target=self.__ls_robot.run, daemon=True)
            self.__runRobot.start()

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
        Callback function for a service which lstops all nodes related to the robot itself. It basically kills each node except the 
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
        if self.__runRobot.is_alive():
            self.__runRobot.terminate()
            response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
        else:
            response_data.statuscode = EmptyWithStatuscode.Response.ALREADY_STOPPED
            #  Inform user
            dialog_request = ShowDialog.Request()
            dialog_request.title = "Turtlebot already stopped"
            dialog_request.content = "The Turtlebot is already stopped. Was it stopped before or by another component?"
            dialog_request.icon = ShowDialog.Request.ICON_INFO
            dialog_request.interaction_options = ShowDialog.Request.CONFIRM

            self.__gui_sclient_showDialog.call_async(dialog_request)
            # Because we only confirm the user, we doen't need to check the response

        # Kill slam nodes by killing the process which runs these. Because slam is not running every time,
        # no error status response is generated when node is already stopped
        if self.__runMapping.is_alive():
            self.__runMapping.terminate()

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
                            EmptyWithStatuscode.Response.ALREADY_ACTIVE
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
        :meta public:
        Callback function for a service which stops the mapping. This is done by setting the amcl node to an \
        active state so it does operate again and stopping the slam node. Furthermore, the map is saved on Sopias4 Map-Server

        Args:
            request_data (EmptyWithStatuscode.Request): The data from the request. Look at service definition in srv/EmptyWithStatusCode.srv
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation.  \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
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
                            EmptyWithStatuscode.Response.ALREADY_ACTIVE
                        )
                        return response_data
                except Exception as e:
                    raise e

        #  Save map
        response_data = self.__save_map(response_data)  # type: ignore

        return response_data

    def __save_map(
        self, response_data: EmptyWithStatuscode.Response
    ) -> EmptyWithStatuscode.Response:
        """
        Handles the map saving process itself. If an error occurs, it informs the user and ask if it should be retried

        Args:
            response_data (EmptyWithStatuscode.Response): A response object into which the data for the response is written. \
                                                                                                Look at service definition in srv/EmptyWithStatusCode.srv
        
        Returns:
            EmptyWithStatuscode.Response: A response which contains the statuscode of the operation.  \
                                                                    Look at service definition in srv/EmptyWithStatusCode.srv
        """
        # TODO Change to constants from Sopias4 Map Server
        save_map_requ = SaveMap.Request()
        save_map_requ.map_topic = "/map"
        save_map_requ.map_url = "my_map"
        save_map_requ.map_mode = "trinary"
        save_map_requ.free_thresh = 0.25
        save_map_requ.occupied_thresh = 0.65

        future = self.__ms_sclient_saveMap.call_async(save_map_requ)
        # Check if map was saved successfully
        while rclpy.ok():
            if future.done():
                try:
                    if future.result():
                        response_data.statuscode = EmptyWithStatuscode.Response.SUCCESS
                    else:
                        response_data.statuscode = (
                            EmptyWithStatuscode.Response.SAVEING_FAILED
                        )

                        #  Inform user
                        dialog_request = ShowDialog.Request()
                        dialog_request.title = "Saving map failed"
                        dialog_request.content = (
                            "The map couldn't be saved. Check Sopias4 Map-Server"
                        )
                        dialog_request.icon = ShowDialog.Request.ICON_ERROR
                        dialog_request.interaction_options = (
                            ShowDialog.Request.IGNORE_RETRY
                        )

                        self.__gui_sclient_showDialog.call_async(dialog_request)
                        while rclpy.ok():
                            if future.done():
                                try:
                                    # User chose to retry operation
                                    if future.result() == ShowDialog.Response.RETRY:
                                        #  Call function recursively to retry save operation
                                        return self.__save_map(response_data)
                                    #  User chose to ignore error
                                    else:
                                        response_data.statuscode = (
                                            EmptyWithStatuscode.Response.SAVEING_FAILED
                                        )
                                        return response_data
                                except Exception as e:
                                    raise e

                except Exception as e:
                    raise e

        return response_data

    def __generate_launch_description(self, use_gazebo=False) -> LaunchDescription:
        """
        It generates a launch description which includes all nodes needed to run Sopias4 Application
        execept SLAM, Robot-Manager and GUI because these are started on other places
        """

        ld = LaunchDescription()

        if use_gazebo:
            # Run turtlebot in gazebo/simulation
            turtlebot4 = GroupAction(
                [
                    PushRosNamespace(self.get_namespace()),
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
                    ),
                ]
            )
        else:
            # Connect to real turtlebot
            turtlebot4 = launch_ros.actions.Node(
                package="turtlebot4_node",
                executable="turtlebot4_node",
                name="turtlebot4_node",
                namespace=self.get_namespace(),
            )
        ld.add_action(turtlebot4)

        amcl = launch_ros.actions.Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            namespace=self.get_namespace(),
        )
        ld.add_action(amcl)

        rviz2 = launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=self.get_namespace(),
        )
        ld.add_action(rviz2)

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
        ld.add_action(nav2)

        return ld

    def destroy_node(self):
        """
        Clear up tasks when the node gets destroyed by e.g. a shutdown. Mainly releasing all service and action clients
        :meta private:
        """
        # Release service clients
        self.__mrc_sclient_namespaces.destroy()
        self.__mrc_sclient_robot_identity.destroy()
        self.__mrc_sclient_robots.destroy()
        self.__amcl_sclient_lifecycle.destroy()
        self.__nav2_aclient_driveToPos.destroy()
        # Release services
        self.launch_service.destroy()
        self.stop_service.destroy()
        self.start_mapping_service.destroy()
        self.stop_mapping_service.destroy()
        self.drive_to_pos_action.destroy()
        self.get_logger().info("[Robot Manager] Shutting down node")
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
