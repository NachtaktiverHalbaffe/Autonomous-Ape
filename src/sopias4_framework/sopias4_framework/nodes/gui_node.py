import abc
import random
import string
from multiprocessing import Process
from threading import Event, Thread

import launch_ros
import rclpy
from geometry_msgs.msg import Twist
from launch import LaunchDescription, LaunchService
from PyQt5.QtWidgets import QMainWindow, QMessageBox
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sopias4_framework.nodes.robot_manager import RobotManager
from sopias4_framework.tools.ros2 import drive_tools

from sopias4_msgs.srv import (
    Drive,
    EmptyWithStatuscode,
    LaunchTurtlebot,
    Register,
    ShowDialog,
    StopMapping,
)


class GUINode(QMainWindow):
    """
    This class is a Base GUI class which can be used to building a own GUI in the Sopias4 project. This class handles the QT GUI portion of the Node, the ROS2 Node itself 
    is a class attribute (node) which is instantiated by this class and runs under the hood in another object/Thread.

    For generating a GUI, create a UI file with QT Designer with which you can design the GUI graphically. For this purpose, you can run `designer` in the terminal to open 
    QT-Designer from the Dev-Container environment, otherwise you can install it on your host OS and use it there. After that, save the UI-File in the assets folder in Sopias4 
    Application and run the script `python3 generate_ui.py -i <path to ui file> -o <path to directory where the generated Python class should be saved>` which converts it to a 
    Python class. The script is located in `sopias4_framework/tools/scripts` in this ROS2-package. After that, you can inherit from the GUI Node and do the necessary steps:

    .. highlight:: python
    .. code-block:: python

            class YourImplementationClass(GUINode):

                def __init__(self) ->None:
                    # The script should name the Python object this way, but can vary. Doublecheck the class name
                    # within the generated Python file to be sure
                    self.ui: Ui_MainWindow # To enable auto-completion
                    ui_file = Ui_MainWindow() 
                    super().__init(self, ui_file)

                def connect_callbacks(self):
                    # You need to overrride this method. Connect your UI elements with the callacks here.
                    # The ui elements are in self.ui field of the GUINode class
                    self.ui.example_button.clicked.connect(self.__foobar)

                def set_default_values(self):
                    # You need to override this method. Set default values or default choices of your ui elements here
                    self.ui.example_textbox.setText("Hello World")

                def __foobar():
                    print("Hello World!")


    Following methods needs to be overriden (look further into documentation for more details):
        - connect_callbacks()
        - set_default_values()
    
    It also has builtin methods which you can use a callback functionas for certain tasks in Sopias4 (look further into documentation for more details):
        - register_namespace(): Register namespace in Sopias4 Map-Server
        - launch_robot(): Start all the nodes in Sopias4 Application which are interacting with the Turtlebot
        - stop_robot(): Stops all the nodes in Sopias4 Application which are interacting with the Turtlebot

    It provides following ROS2-services:
        - show_dialog: Show a dialog with the users. Also interaction options can be specified

    Attributes:
            ui (Ui_MainWindow()): The Python UI Object which is generated by PyQt5. Should normally be a Ui_MainWindow() instance if using the provided scripts.\
            node_name (str, optional): Name of the Node. Defaults to gui_node
            namespace (str, optional): The namespace of the node. It should not be specified by the developer. Instead the User should use the register service and \
                                                        let the service set the namespace of this class. Defaults to None
            turtlebot_running (bool): If the nodes which are interacting with the Turtlebot are running (True) or not (False). Defaults to False on startup
            is_mapping (bool): If the mapping process is running (True) or not (False). Defaults to False on startup
    """

    __metaclass__ = abc.ABCMeta

    def __init__(
        self, ui, node_name: str = "gui_node", namespace: str | None = None
    ) -> None:
        # General setup
        super().__init__()
        # Class attributes
        self.ui = ui
        self.node_name: str = node_name
        self.namespace: str | None = namespace
        self.turtlebot_running: bool = False
        self.is_mapping: bool = False
        # Private class attributes
        self.__rm_node: RobotManager
        self.__restart_flag = Event()

        # Setup GUI
        self.ui.setupUi(self)
        # Connecting the ui elements with callbacks and set values
        self.connect_callbacks()
        self.set_default_values()

        rclpy.init()
        self.node: GrapficalNode = GrapficalNode(
            node_name=node_name, namespace=namespace
        )
        self.__executor = MultiThreadedExecutor()
        self.__executor.add_node(self.node)
        self.__spin_node_thread = Thread(target=self.__executor.spin)
        self.__spin_node_thread.start()

    @abc.abstractmethod
    def connect_callbacks(self) -> None:
        """
        Connect the callback functions from the QT UI elements here. Needs to be overridden

        Example:
            .. highlight:: python
            .. code-block:: python

                    def connect_callbacks(self):
                        # The ui elements are in self.ui field of the GUINode class
                        self.ui.example_button.clicked.connect(self.hello_world)

        """

    @abc.abstractmethod
    def set_default_values(self) -> None:
        """
        Set the default values from the QT UI elements here. Needs to be overridden

        Example:
            .. highlight:: python
            .. code-block:: python

                  def set_default_values(self):
                    # Set default values or default choices of your ui elements here
                    self.ui.example_textbox.setText("Hello World")
        """

    def register_namespace(self, namespace: str) -> None:
        """
        Register a namespace on the Sopias4 Map-Server. It's basically a wrapper and calling the register_namespace
        service client in the underlying node object. If successful, it will restart `self.node` with the namespace.
        This must be done before connecting to the Turtlebot.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed

        Args:
            namespace (str): The namespace which should be registered
        """
        try:
            if self.node.register_namespace(namespace):
                self.namespace = namespace
                # Set restart flag so GUI recognizes the node shutdown as intentional and doesn't close
                self.__restart_flag.set()
            else:
                return
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Could'nt register name space: {e}")
            raise e

        # Set namespace of physical turtlebot
        # TODO send ssh command to turtlebGrot
        # SSH command "turtlebot4-setup"

        # Restart node with namespace
        self.__executor.remove_node(self.node)
        self.node.destroy_node()
        self.node = GrapficalNode(node_name=self.node_name, namespace=self.namespace)
        self.__executor.add_node(self.node)
        # Start robot manager
        self.__rm_node = RobotManager(namespace=self.namespace)
        self.__executor.add_node(self.__rm_node)

    def launch_robot(self, use_simulation: bool = False) -> None:
        """
        Launches all the nodes in Sopias4 Application so the system is connected to the Turtlebot. It's basically
        a wrapper and calling the launcg service client in the underlying node object. Before running this, a namespace
        must already be registered and the gui node needs to be running under this namespace. If the operation was
        successful, then it sets `self.turtlebot_running` to `True`

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        #  Error message for user in Case something goes wrong
        msg_request = ShowDialog.Request()
        msg_request.title = "Couldn't launch robot"
        msg_request.icon = ShowDialog.Request.ICON_ERROR
        msg_request.interaction_options = ShowDialog.Request.CONFIRM_RETRY
        msg_request.content = "Couldn't launch robot. Check if the Turtlebot4\
                    nodes inside Sopias4 Application aren't running and that the physical \
                    robot is started and connected to the network"

        try:
            status_response = self.node.launch_turtlebot(use_simulation=use_simulation)

            if status_response:
                self.turtlebot_running = True
                self.node.get_logger().debug("[GUI] Launched robot")
            else:
                self.turtlebot_running = False
                self.node.get_logger().error(
                    f"[GUI] Could'nt launch robot. Turtlebot is either already running or is'nt reachable"
                )
                self.__inform_and_retry(msg_request, self.launch_robot)
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Couldnt launch robot: {e}")
            raise e

    def stop_robot(self) -> None:
        """
        Stops all the nodes in Sopias4 Application so the system is disconnected to the Turtlebot. It's basically
        a wrapper and calling the launch service client in the underlying node object. If the operation was
        successful, then it sets `self.turtlebot_running` to `True`

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        #  Error message for user in Case something goes wrong
        msg_request = ShowDialog.Request()
        msg_request.title = "Couldn't stop robot"
        msg_request.icon = ShowDialog.Request.ICON_ERROR
        msg_request.interaction_options = ShowDialog.Request.CONFIRM_RETRY
        msg_request.content = "Couldn't stop Robot. Check if the Turtlebot4\
                    nodes inside Sopias4 Application are already stopped"
        try:
            status_response = self.node.stop_turtlebot()

            if status_response:
                self.turtlebot_running = False
                self.namespace = None
                self.node.get_logger().debug("[GUI] Stopped robot")
            else:
                self.turtlebot_running = True
                self.node.get_logger().error(
                    "[GUI] Couldnt stop robot: Nodes are either already stopped or error is unknown"
                )
                self.__inform_and_retry(msg_request, self.stop_robot)
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Couldnt stop robot: {e}")
            raise e

    def start_mapping(self) -> None:
        """
        Starts the mapping process. This can only be done if Sopias4 Application is fully running, otherwise
        will directly abort this process. It's basically a wrapper and calling the start_mapping service client
        in the underlying node object. If the operation was successful, then it sets `self.is_mapping` to `True`

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        #  Error message for user in Case something goes wrong
        msg_request = ShowDialog.Request()
        msg_request.title = "Couldn't start mapping"
        msg_request.icon = ShowDialog.Request.ICON_ERROR
        msg_request.interaction_options = ShowDialog.Request.CONFIRM_RETRY
        msg_request.content = "Couldn't start mapping. Check if the Turtlebot4\
                    Nodes inside Sopias4 Application are running and thats theres no mapping already in progress"
        try:
            if self.turtlebot_running and not self.is_mapping:
                status_response = self.node.start_mapping()

                if status_response:
                    self.is_mapping = True
                    self.node.get_logger().debug("[GUI] Started mapping")
                else:
                    self.is_mapping = False
                    self.node.get_logger().error(
                        "[GUI] Couldnt start mapping: Slam node couldn't be launched or unknown error occured"
                    )
                    # Inform user about failure and see for it's response
                    self.__inform_and_retry(msg_request, self.start_mapping)
            else:
                self.node.get_logger().warning(
                    "[GUI] Couldnt start mapping: Mapping already in process or Turtlebot is offline"
                )
                # Inform user about failure and see for it's response
                self.__inform_and_retry(msg_request, self.start_mapping)
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Couldnt start mapping: {e}")
            raise e

    def stop_mapping(self) -> None:
        """
        Stops the mapping process. This can only be done if Sopias4 Application and the mapping process is fully running,
        otherwise itwill directly abort this process. It's basically a wrapper and calling the start_mapping service
        client in the underlying node object. If the operation was successful, then it sets `self.is_mapping` to `False`

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        #  Error message for user in Case something goes wrong
        request_informUser = ShowDialog.Request()
        request_informUser.title = "Couldn't stop mapping"
        request_informUser.icon = ShowDialog.Request.ICON_ERROR
        request_informUser.interaction_options = ShowDialog.Request.CONFIRM_RETRY
        request_informUser.content = "Couldn't stop mapping. Check if the Turtlebot4\
                    nodes inside Sopias4 Application are running and thats theres a mapping already in progress"
        try:
            if self.turtlebot_running and self.is_mapping:
                status_response = self.node.stop_mapping()
                if status_response:
                    self.turtlebot_running = True
                    self.node.get_logger().debug("[GUI] Stopped Mapping")
                else:
                    self.turtlebot_running = False
                    self.node.get_logger().error(
                        "[GUI] Couldnt stop mapping: Either SLAM node couldn't be shutdown or a unknown error occured"
                    )
                    # Inform user about failure and see for it's response
                    self.__inform_and_retry(request_informUser, self.stop_mapping)
            else:
                self.node.get_logger().warning(
                    "[GUI] Couldn't stop mapping: Mapping is already stopped or Turtlebot is offline"
                )
                # Inform user about failure and see for it's response
                self.__inform_and_retry(request_informUser, self.stop_mapping)
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Couldnt stop mapping: {e}")
            raise e

    def drive(
        self,
        twist_msg: Twist | None = None,
        direction: str = "stop",
        vel_rel: float = 1.0,
    ) -> None:
        """
        Lets the Turtlebot drive. This can only be done if Sopias4 Application is fully running, otherwise it will directly\
        abort this process. It's basically a wrapper and calling the sdrive service client in the underlying node object. 

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed

        Args:
            twist_msg (Twist, optional): The twist message which specifies how the Turtlebot should drive. If None (default), \
                                                            when a appropriate Twist message will be generated (Important: In this casem direction and vel_rel must be provided). 
            direction (str, optional): The direction which the Turtlebot should drive. Can be either forward, backward, left, \
                                                     right, rotate_left, rotate_right and stop. Defaults to stop
            vel_rel (float, optional): The relative velocity. The value is normed to the maximum speed of the Turtlebot,\
                                                     so e.g. 1.0 is maximum speed and 0 is standing still. Defaults to 1.0
        """
        #  Error message for user in Case something goes wrong
        request_informUser = ShowDialog.Request()
        request_informUser.title = "Couldn't send drive command"
        request_informUser.icon = ShowDialog.Request.ICON_ERROR
        request_informUser.interaction_options = ShowDialog.Request.CONFIRM_RETRY
        request_informUser.content = "Couldn't send drive command. Check if the Turtlebot4\
                    nodes inside Sopias4 Application are running"
        try:
            if self.turtlebot_running:
                status_response = self.node.drive(twist_msg, direction, vel_rel)
                if status_response:
                    self.node.get_logger().debug(
                        "[GUI] Successfully send drive command to Turtlebot"
                    )
                else:
                    self.node.get_logger().error(
                        "[GUI] Couldnt send drive command to Turtlebot du to unknown error"
                    )
                    # Inform user about failure and see for it's response
                    self.__inform_and_retry(request_informUser, self.stop_mapping)
            else:
                self.node.get_logger().warning(
                    "[GUI] Couldn't send drive command to Turtlebot: Turtlebot is offline"
                )
                # Inform user about failure and see for it's response
                self.__inform_and_retry(request_informUser, self.stop_mapping)
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"[GUI] Couldnt stop mapping: {e}")
            raise e

    def closeEvent(self, event):
        """
        Executes when the GUI is closed

        :meta private:
        """
        self.node.destroy_node()
        self.__rm_node.destroy_node()
        rclpy.shutdown()
        event.accept()

    def __inform_and_retry(self, msg: ShowDialog.Request, retry_fn):
        """
        It creates a message which is shown to the user and if the user\
        chooses to retry operation calls the passed function

        Args:
            msg (ShowDialog.Request): The message which should be shown to the user
            retry_fn (function): The function which should be executed in case the user\
                                             chooses to retry the operation. If the function takes arguments,\
                                             then pass this function as a lambda e.g. `__inform_and_rety(msg, lambda: foo("bar"))`
        """
        user_response = self.node._show_dialog(msg, ShowDialog.Response())
        if user_response == ShowDialog.Response.RETRY:
            self.node.get_logger().debug(f"[GUI] User chose to retry {retry_fn}")
            retry_fn()


class GrapficalNode(Node):
    """
    This is the ROS2 Node which runs under the hood in the GUI. To avoid problems by multiple inheritance etc, \
    the node is done in this separate class which is instantiated as a attribute in the GUI. As a result, this class only
    handles the ROS2 Node specific stuff e.b. implementing services, actions, messages etc.

    Attributes:
        node_name (str): (optional) Name of the Node. Defaults to gui_node
        namespace (str): (optional) The namespace of the node. When not specified, the GUI has to register itself.\
                                    The namespace should not be set by the developer but instead let the register_namespace service set the namespace
    """

    def __init__(
        self, node_name: str = "gui_node", namespace: str | None = None
    ) -> None:
        if namespace is not None:
            super().__init__(node_name, namespace=namespace)  # type: ignore
        else:
            ns = "".join(random.choices(string.ascii_lowercase, k=8))
            super().__init__(node_name, namespace=ns)  # type: ignore

        # Log level 10 is debug
        self.get_logger().set_level(10)
        self.get_logger().info(
            f"[GUI] Node started with namespace {self.get_namespace()}"
        )

        # ---- Setup services -----
        self.show_dialog_service = self.create_service(
            ShowDialog, "show_dialog", self._show_dialog
        )

        # --- Setup service clients ---
        # Create own sub node for service clients so they can spin independently
        self.__service_client_node: Node = rclpy.create_node("_gui_service_clients", namespace=self.get_namespace())  # type: ignore
        # This service registers the namespace in the multi robot coordinator
        # inside the Sopias4 Map-server
        self.__mrc_sclient_register: Client = self.__service_client_node.create_client(
            Register, "/register_namespace"
        )
        # This service launches/connects to the corresponding Turtlebot
        # by launching the nodes of Sopias4 Application
        self.__rm_sclient_launch: Client = self.__service_client_node.create_client(
            LaunchTurtlebot, f"{self.get_namespace()}/launch"
        )
        # This service stops the running nodes of Sopias4 Application
        # so that the system isn't connected anymore to the physical robot
        self.__rm_sclient_stop_robot: Client = self.__service_client_node.create_client(
            EmptyWithStatuscode, f"{self.get_namespace()}/stop"
        )
        # This service starts the mapping process
        self.__rm_sclient_start_mapping: Client = (
            self.__service_client_node.create_client(
                EmptyWithStatuscode, f"{self.get_namespace()}/start_mapping"
            )
        )
        # This service stops the mapping provess
        self.__rm_sclient_stop_mapping: Client = (
            self.__service_client_node.create_client(
                StopMapping, f"{self.get_namespace()}/stop_mapping"
            )
        )
        # This service sends a manual drive command to the robot
        self.__rm_sclient_drive: Client = self.__service_client_node.create_client(
            Drive, f"{self.get_namespace()}/drive"
        )

    def register_namespace(self, namespace: str):
        """
        Runs a service client to register the namespace in Sopias4 Map-Server.
        On Failure, the user is informed and has a choice to retry

        Args:
            namespace (str): The namespace which should be registered

        Returns:
            bool: If namespace was registered successfully or not
        """
        self.get_logger().debug(
            f"[GUI] Sending service request to register namespace {namespace}"
        )
        request: Register.Request = Register.Request()
        request.namespace_canditate = f"/{namespace}"
        future = self.__mrc_sclient_register.call_async(request)
        self.get_logger().debug(
            "[GUI] Service request for registering namespace sent. Waiting for response"
        )

        # Make sure the node itself is spinnig
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        response: Register.Response | None = future.result()
        if response is None:
            return False
        elif response.statuscode == Register.Response.SUCCESS:
            return True
        else:
            # Inform user about error
            msg_2_user = ShowDialog.Request()
            msg_2_user.title = "Error while registering namespace"
            msg_2_user.icon = ShowDialog.Request.ICON_ERROR

            match response.statuscode:
                case Register.Response.COLLISION_ERROR:
                    self.get_logger().error(
                        "[GUI] Couldn't register namespace: Already registered"
                    )
                    msg_2_user.content = (
                        "Namespace is already registered. Choose another one"
                    )
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM
                case Register.Response.ILLEGAL_NAMESPACE_ERROR:
                    self.get_logger().error(
                        "[GUI] Couldn't register namespace: Namespace contains illegal characters"
                    )
                    msg_2_user.content = (
                        "Namespace contains illegal characters. Choose another one"
                    )
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM
                case Register.Response.UNKOWN_ERROR:
                    self.get_logger().error(
                        "[GUI] Couldn't register namespace: Unkown error"
                    )
                    msg_2_user.content = "Unknown error occured"
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM_RETRY

            user_response = self._show_dialog(msg_2_user, ShowDialog.Response())

            # If user response is to retry, then recursively call this function, otherwise return False
            if user_response.selected_option == ShowDialog.Response.CONFIRMED:
                return False
            elif user_response.selected_option == ShowDialog.Response.RETRY:
                return self.register_namespace(namespace)
            else:
                return False

    def launch_turtlebot(self, use_simulation: bool = False) -> bool:
        """
        Runs a service client to start the nodes in Sopias4 Application so the system is connected
        to the robot and ready for operation.

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("[GUI] Sending service request to launch Turtlebot")
        request = LaunchTurtlebot.Request()
        request.use_simulation = use_simulation
        future = self.__rm_sclient_launch.call_async(request)

        self.get_logger().debug(
            "[GUI] Sent service request to launch Turtlebot. Waiting for response"
        )
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        # Check response
        response: LaunchTurtlebot.Response | None = future.result()
        if response is None:
            return False
        elif response.statuscode == LaunchTurtlebot.Response.SUCCESS:
            return True
        else:
            return False

    def stop_turtlebot(self) -> bool:
        """
        Runs a service client to stop the nodes in Sopias4 Application so the system is disconnected
        from the robot and ready for operation.

        Returns:
            bool: If operation was successful
        """
        # --- Stop the nodes ---
        self.get_logger().debug(
            "[GUI] Stops Turtlebot. Sending service request to stop nodes"
        )
        stop_request = EmptyWithStatuscode.Request()
        future = self.__rm_sclient_stop_robot.call_async(stop_request)

        self.get_logger().debug(
            "[GUI] Service request to stop nodes sent. Waiting for response"
        )
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        # Check response
        response: EmptyWithStatuscode.Response | None = future.result()

        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def start_mapping(self) -> bool:
        """
        Runs a service client to start the mapping process. The Sopias4 Application should
        be fully launched before running this service

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("[GUI] Send service request to start mapping")
        request = EmptyWithStatuscode.Request()
        future = self.__rm_sclient_start_mapping.call_async(request)

        self.get_logger().debug(
            "[GUI] Service request to start mapping sent. Waiting for response"
        )
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        # Check response
        response: EmptyWithStatuscode.Response | None = future.result()
        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def stop_mapping(self) -> bool:
        """
        Runs a service client to stop the mapping process. The Sopias4 Application should
        be fully launched and the mapping process running before running this service

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("[GUI] Sending service request to stop mapping")
        request = StopMapping.Request()
        future = self.__rm_sclient_stop_mapping.call_async(request)

        self.get_logger().debug(
            "[GUI] Service request to start mapping sent. Waiting for response"
        )
        rclpy.spin_until_future_complete(self.__service_client_node, future)

        # Check response
        response: StopMapping.Response | None = future.result()
        if response is None:
            return False
        elif response.statuscode == StopMapping.Response.SUCCESS:
            return True
        else:
            return False

    def drive(
        self,
        twist_msgs: Twist | None = None,
        direction: str = "stop",
        vel_rel: float = 1.0,
    ) -> bool:
        """
        Runs a service client to send a driving command to the Turtlebot. The Sopias4 Application
        should be fully launched and the mapping process running before running this service

        Returns:
            bool: If operation was successful
        """
        request = Drive.Request()
        if twist_msgs is None:
            request.twist = drive_tools.generate_twist_msg(
                direction=direction, vel_rel=vel_rel
            )
        else:
            request.twist = twist_msgs
        future = self.__rm_sclient_drive.call_async(request)

        rclpy.spin_until_future_complete(self.__service_client_node, future)
        response: Drive.Response | None = future.result()

        if response is None:
            return False
        elif response.statuscode == Drive.Response.SUCCESS:
            return True
        else:
            return False

    def _show_dialog(
        self, request_data: ShowDialog.Request, response_data: ShowDialog.Response
    ) -> ShowDialog.Response:
        """Callback function for the show_dialog service. It creates a QT dialog, fills it with the values from the service request and shows it.
        Afterwards, if specified it returns the input from the user.

        Args:
            request_data (ShowDialog.Request): The data from the request. Look at service definition in srv/ShowDialog.srv
            reponse_data (ShowDialog.Response):  A response object into which the data for the response is written. Look at service definition in srv/ShowDialog.srv

        Returns:
            ShowDialog.Response: It returns response_data in which contains the selected option from the user input

        Raises:
            ValueError: If theres invalid data in the response
            NotImplementedError: If the user chooses a interaction option which isn't specified in the service response
        """
        self.get_logger().info(
            f"[GUI] Got service request to display dialog with title {request_data.title}"
        )
        dlg = QMessageBox()
        # Set static data that doesn't need validation
        dlg.setWindowTitle(request_data.title)
        dlg.setInformativeText(request_data.title)

        # Choose icon from set of QMessagebox icons
        match request_data.icon:
            case ShowDialog.Request.ICON_QUESTION:
                dlg.setIcon(QMessageBox.Question)
            case ShowDialog.Request.ICON_INFO:
                dlg.setIcon(QMessageBox.Information)
            case ShowDialog.Request.ICON_WARNING:
                dlg.setIcon(QMessageBox.Warning)
            case ShowDialog.Request.ICON_ERROR:
                dlg.setIcon(QMessageBox.Critical)
            case _:
                raise ValueError(
                    f"Specified icon: {request_data.icon} isn't implemented or has wrong value. \
                                Implemented icons: {ShowDialog.Request.ICON_QUESTION}, {ShowDialog.Request.ICON_INFO},\
                                {ShowDialog.Request.ICON_WARNING} and {ShowDialog.Request.ICON_ERROR}"
                )

        # Choose buttons which are displayed depending on the chosen interaction option in the request
        # The service supports the buttons which are defined as constants in the service definition (see srv/ShowDialog.srv)
        # PyQT support following buttons:  QMessageBox.Ok , QMessageBox.Open , QMessageBox.Save
        # QMessageBox.Cancel, QMessageBox.Close, QMessageBox.Yes, QMessageBox.No, QMessageBox.Abort
        # QMessageBox.Retry and QMessageBox.Ignore
        match request_data.interaction_options:
            case ShowDialog.Request.CONFIRM:
                dlg.setStandardButtons(QMessageBox.Ok)
            case ShowDialog.Request.CONFIRM_ABORT:
                dlg.setStandardButtons(QMessageBox.Abort | QMessageBox.Ok)
            case ShowDialog.Request.CONFIRM_RETRY:
                dlg.setStandardButtons(QMessageBox.Retry | QMessageBox.Ok)
            case ShowDialog.Request.CONFIRM_CANCEL:
                dlg.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ok)
            case ShowDialog.Request.YES_NO:
                dlg.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
            case ShowDialog.Request.IGNORE_CANCEL:
                dlg.setStandardButtons(QMessageBox.Ignore | QMessageBox.Cancel)
            case ShowDialog.Request.IGNORE_ABORT:
                dlg.setStandardButtons(QMessageBox.Ignore | QMessageBox.Abort)
            case ShowDialog.Request.IGNORE_RETRY:
                dlg.setStandardButtons(QMessageBox.Ignore | QMessageBox.Retry)
            case _:
                raise ValueError(
                    f"Specified interaction option: {request_data.interaction_options} has isn't implemented \
                                    or has wrong value. Implemented icons: {ShowDialog.Request.CONFIRM}, {ShowDialog.Request.CONFIRM_ABORT} \
                                    {ShowDialog.Request.CONFIRM_CANCEL}, {ShowDialog.Request.CONFIRM_RETRY} and {ShowDialog.Request.YES_NO}"
                )

        self.get_logger().debug(f'[GUI] Showing dialog "{request_data.title}"')
        # Show dialog and get the pressed button
        selected_option = dlg.exec()

        # Here we can look for each button, even if they werent displayed,
        # because we only want to know the pressed button so the ones that arent displayed are not selected
        match selected_option:
            case QMessageBox.Ok:
                response_data.selected_option = ShowDialog.Response.CONFIRMED
            case QMessageBox.Abort:
                response_data.selected_option = ShowDialog.Response.ABORT
            case QMessageBox.Retry:
                response_data.selected_option = ShowDialog.Response.RETRY
            case QMessageBox.Yes:
                response_data.selected_option = ShowDialog.Response.YES
            case QMessageBox.No:
                response_data.selected_option = ShowDialog.Response.NO
            case QMessageBox.Ignore:
                response_data.selected_option = ShowDialog.Response.IGNORE
            case QMessageBox.Cancel:
                response_data.selected_option = ShowDialog.Response.CANCEL
            case _:
                raise NotImplementedError(
                    f"Dialog returned selected option: {selected_option} which is not\
                        implemented in ROS Service. Currently supported options are:\
                        QMessageBox.Ok, QMessageBox.Abort, QMessageBox.Retry, \
                        QMessageBox.Yes, QMessageBox.No, QMessageBox.Ignore and QMessageBox.Cancel"
                )

        return response_data

    def destroy_node(self):
        """
        Clear up tasks when the node gets destroyed by e.g. a shutdown. Mainly releasing all service and action clients
        :meta private:
        """
        # Release service clients
        self.__rm_sclient_launch.destroy()
        self.__rm_sclient_stop_robot.destroy()
        self.__rm_sclient_start_mapping.destroy()
        self.__rm_sclient_stop_mapping.destroy()
        self.__mrc_sclient_register.destroy()
        self.__service_client_node.destroy_node()
        # Release services
        self.show_dialog_service.destroy()
        self.get_logger().info("[GUI] Shutting down node")

        super().destroy_node()


if __name__ == "__main__":
    print(
        "Only run this node directly for testing purposes. In production this node should be extendend properly"
    )
