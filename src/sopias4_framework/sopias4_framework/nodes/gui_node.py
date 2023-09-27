#!/usr/bin/env python3
import abc
import multiprocessing
import os
import random
import string
import time
from threading import Thread

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QFileDialog, QMainWindow, QMessageBox
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sopias4_framework.nodes.robot_manager import RobotManager
from sopias4_framework.tools.ros2 import drive_tools, node_tools

from sopias4_msgs.srv import (
    Drive,
    EmptyWithStatuscode,
    LaunchNav2Stack,
    RegistryService,
    ShowDialog,
    StopMapping,
)


# TODO Disable services when robot isnt registered
class GUINode(QMainWindow):
    """
    This class is a Base GUI class which can be used to building a own GUI in the Sopias4 project. This class handles the QT GUI portion of the Node, the ROS2 Node itself 
    is a class attribute (node) which is instantiated by this class and runs under the hood in another object/Thread.

    For generating a GUI, create a UI file with QT Designer with which you can design the GUI graphically. For this purpose, you can run `designer` in the terminal to open 
    QT-Designer from the Dev-Container environment, otherwise you can install it on your host OS and use it there. After that, save the UI-File in the assets folder in Sopias4 
    Application and run the script 
    
    .. highlight:: bash
    .. code-block:: bash

        python3 generate_ui.py -i <path to ui file> -o <path to directory where the generated Python class should be saved>

    which converts it to a Python class. The script is located in `sopias4_framework/tools/scripts` in this ROS2-package. After that, you can inherit from the GUI Node and do the necessary steps:

    .. highlight:: python
    .. code-block:: python

            class YourImplementationClass(GUINode):

                def __init__(self) ->None:
                    # The script should name the Python object this way, but can vary. Doublecheck the class name
                    # within the generated Python file to be sure
                    self.ui: Ui_MainWindow # To enable auto-completion in IDE
                    super().__init__(Ui_MainWindow())

                def connect_ui_callbacks(self):
                    # You need to overrride this method. Connect your UI elements with the callacks here.
                    # The ui elements are in self.ui field of the GUINode class
                    self.ui.example_button.clicked.connect(lambda: Thread(target=self.__foobar).start())

                def set_default_values(self):
                    # You need to override this method. Set default values or default choices of your ui elements here
                    self.ui.example_textbox.setText("Hello World")
                
                def set_initial_enabled_elements(self):
                    # Disable the desired elements here
                    self.ui.pushButton_example.setEnabled(False)

                def connect_ros2_callbacks(self):
                    # CReate and connect subscriptions here
                    GuiLogger(
                        widget=self.ui.textEdit,
                        node=self.node,
                        namespace_filter=self.node.get_namespace(),
                    ) 
                    LabelSubscriptionHandler(
                        widget=self.ui.label_battery, node=self.node, message_type=BatteryState
                    )
                    
                def __foobar():
                    print("Hello World!")


    Following methods needs to be overriden (look further into documentation for more details):
        - connect_ui_callbacks()
        - connect_ros2_callbacks()
        - set_default_values()
        - set_initial_enabled_elements()
    
    It also has builtin methods which you can use as callback function for certain tasks in Sopias4 (look further into documentation for more details):
        - load_ros2_node(): Loads and runs a Python ROS2 node into the executor
        - unload_ros2_node(): Unloads and stops a Ptyhon ROS2 node from the executor
        - register_namespace(): Register namespace in Sopias4 Map-Server
        - unregister_namespace(): Unregisters a namespace in Sopias4 Fleetbroker
        - launch_nav2_stack(): Start all the nodes in Sopias4 Application which are interacting with the Turtlebot
        - stop_nav2_stack(): Stops all the nodes in Sopias4 Application which are interacting with the Turtlebot
        - start_mapping(): Starts the mapping process
        - stop_mapping(): Stops the mapping process and saves the map on the Sopias4 Map-Server
        - drive(): Let the robot execute a specific drive command

    It provides following ROS2-services:
        - show_dialog: Show a dialog to the user. Also interaction options can be specified
    
    **It has following attributes:**

    Attributes:
            ui (Ui_MainWindow): The Python UI Object which is generated by PyQt5. Should normally be a Ui_MainWindow() instance if using the provided scripts.\
            node_name (str, optional): Name of the Node. Defaults to gui_node
            namespace (str, optional): The namespace of the node. It should not be specified by the developer. Instead the User should use the register service and \
                                                        let the service set the namespace of this class. Defaults to None
            node (rclpy.node): The underlying ROS2 node which runs in the background
            node_executor (rclpy.executors.MultiThreadedExecutor): A executor which runs the nodes which are loaded into it. By default the GUI node and the\
                                                                                                            RobotManager (if application is registered in the Sopias4 Fleetbroker) are loaded into it
    """

    __metaclass__ = abc.ABCMeta
    # Signals for displaying dialogs because they need to be called from main thread and nodes run in a background thread
    __display_dialog_signal = pyqtSignal(ShowDialog.Request)
    __showed_dialog_signal = pyqtSignal(ShowDialog.Response)
    __registered_signal = pyqtSignal()

    def __init__(
        self, ui, node_name: str = "gui_node", namespace: str | None = None
    ) -> None:
        # General setup
        super().__init__()
        # Class attributes
        self.ui = ui
        self.node_name: str = node_name
        self.namespace: str | None = namespace
        # Private class attributes
        self.__rm_node: RobotManager | None = None
        self.__display_dialog_signal.connect(self.display_dialog)
        self.__registered_signal.connect(self.connect_ros2_callbacks)

        rclpy.init()
        self.node: GrapficalNode = GrapficalNode(
            showed_dialog_signal=self.__showed_dialog_signal,
            display_dialog_signal=self.__display_dialog_signal,
            node_name=node_name,
            namespace=namespace,
        )
        self.node_executor = MultiThreadedExecutor()
        self.load_ros2_node(self.node)
        Thread(target=self.node_executor.spin).start()

        # Setup GUI
        self.ui.setupUi(self)
        # Connecting the ui elements with callbacks and set values
        self.connect_ui_callbacks()
        self.connect_ros2_callbacks()
        self.set_default_values()
        self.set_initial_disabled_elements()

    @abc.abstractmethod
    def connect_ui_callbacks(self) -> None:
        """
        Connect the callback functions from the QT UI elements here. This connects user interactions like clicking a button with the execution of an callback function. Needs to be overridden

        Example:
            .. highlight:: python
            .. code-block:: python

                    def connect_ui_callbacks(self):
                        # The ui elements are in self.ui field of the GUINode class
                        self.ui.example_button.clicked.connect( lambda: Thread(target=self.hello_world).start())

        """

    @abc.abstractmethod
    def connect_ros2_callbacks(self) -> None:
        """
        Connects ROS2 callbacks e.g. subscriptions. Mainly used to create LabelSubcriptions which sets the value of UI labels then a \
        subscriptions receives a message.

        Example:
            .. highlight:: python
            .. code-block:: python

                    def connect_ros2_callback(self):
                        # Create and connect subscriptions here
                        gui_logger = GuiLogger(self.ui.textEdit)
                        self.node.create_subscription(Log, "/rosout", gui_logger.add_log_msg, 10)
        """

    @abc.abstractmethod
    def set_default_values(self) -> None:
        """
        Set the default values from the QT UI elements which are shown at startup of the GUI here. Needs to be overridden

        Example:
            .. highlight:: python
            .. code-block:: python

                  def set_default_values(self):
                    # Set default values or default choices of your ui elements here
                    self.ui.example_textbox.setText("Hello World")
        """

    @abc.abstractmethod
    def set_initial_disabled_elements(self) -> None:
        """
        Sets which QT elements should initially be disabled at startup i.e with which elements can't be interacted at startup. All elements
        which aren't disabled here are enabled by default. Remember to enable the disabled UI elements when the necessary confitions are met. Needs to be overridden

        Example:
            .. highlight:: python
            .. code-block:: python

                    def set_initial_enabled_elements(self):
                        # Disable the desired elements here
                        self.ui.pushButton_example.setEnabled(False)
        """

    def load_ros2_node(self, node: Node) -> bool:
        """
        Loads and runs a ROS2 node during runtime. Only Python nodes are supported. For C++ nodes and/or launchfiles,
        use the sopias4_framework.tools.ros.LaunchService package.

        Args:
            node (rclpy.Node): A instantiated object of the node which should be loaded

        Returns:
            bool: If node was loaded successfully
        """
        if self.node_executor.add_node(node):
            self.node.get_logger().info(
                f"Loaded node {node.get_name()} into running executor"
            )
            return True
        else:
            self.node.get_logger().info(
                f"Coulnd't load node {node.get_name()} into running executor"
            )
            return False

    def unload_ros2_node(self, node: Node | None) -> bool:
        """
        Unloads and stops a ROS2 node during runtime. Only use if node was loaded with `load_ros2_node` before

        Args:
            node (rclpy.Node): A instantiated object of the node which should be unloaded

        Returns:
            bool: If node was loaded successfully
        """
        if node is not None:
            self.node_executor.remove_node(node)
            self.node_executor.wake()

            if node not in self.node_executor.get_nodes():
                # Stop and cleanup node itself
                self.node.get_logger().info(
                    f"Unloaded node {node.get_name()} from running executor"
                )
                node.destroy_node()
                return True
            else:
                self.node.get_logger().info(
                    f"Coulnd't unload node {node.get_name()} from running executor"
                )
                return False
        else:
            self.node.get_logger().warning(
                "Couldn't unload node from running executor: Node is already unloaded or is None"
            )
            return False

    def register_namespace(self, namespace: str) -> bool:
        """
        Register a namespace on the Sopias4 Map-Server. It's basically a wrapper and calling the register_namespace
        service client in the underlying node object. If successful, it will restart `self.node` with the namespace.
        This must be done before connecting to the Turtlebot.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed

        Args:
            namespace (str): The namespace which should be registered

        Returns:
            bool: If namespace was registered successfully
        """
        try:
            if self.node._register_namespace(namespace):
                self.namespace = namespace
                # Set restart flag so GUI recognizes the node shutdown as intentional and doesn't close
            else:
                return False
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Could'nt register name space: {e}")
            raise e

        # Restart node with namespace
        for node in self.node_executor.get_nodes():
            self.unload_ros2_node(node)

        self.node = GrapficalNode(
            showed_dialog_signal=self.__showed_dialog_signal,
            display_dialog_signal=self.__display_dialog_signal,
            node_name=self.node_name,
            namespace=self.namespace,
        )
        # Start robot manager
        self.__rm_node = RobotManager(namespace=self.namespace)
        self.load_ros2_node(self.node)
        self.load_ros2_node(self.__rm_node)

        self.__registered_signal.emit()
        self.node.get_logger().info(
            f"Successfully registered namespace {self.namespace}"
        )
        return True

    def unregister_namespace(self, namespace: str) -> bool:
        """
        Unregister a namespace on the Sopias4 Map-Server. It's basically a wrapper and calling the unregister_namespace
        service client in the underlying node object. Shouldn't be needed in normal operation, but sometimes manually
        unregistering is necessary.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed

        Args:
            namespace (str): The namespace which should be registered

        Returns:
            bool: If namespace was registered successfully
        """
        try:
            if self.node._unregister_namespace(namespace):
                # Remove robot manager if running
                self.namespace = None
                if self.__rm_node is not None:
                    self.unload_ros2_node(self.__rm_node)
                    self.__rm_node = None

                self.node.get_logger().info(
                    f"Successfully unregistered namespace {namespace}"
                )
                return True
            else:
                return False
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Could'nt register name space: {e}")
            raise e

    def launch_nav_stack(self, use_simulation: bool = False) -> None:
        """
        Launches all the nodes in Sopias4 Application so the system is ready for autonomous navigation. It's basically
        a wrapper and calling the launcg service client in the underlying node object. Before running this, a namespace
        must already be registered and the gui node needs to be running under this namespace.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        try:
            status_response = self.node._launch_nav_stack(use_simulation=use_simulation)

            if status_response:
                self.node.get_logger().info("Launched robot")
            else:
                self.node.get_logger().error(
                    f"Could'nt launch robot. Turtlebot is either already running or is'nt reachable"
                )
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt launch robot: {e}")
            raise e

    def stop_nav_stack(self) -> None:
        """
        Stops all the nodes in Sopias4 Application so the system cant navigate autonomously anymore. It's basically
        a wrapper and calling the launch service client in the underlying node object.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        try:
            status_response = self.node._disconnect_turtlebot()

            if status_response:
                self.node.get_logger().debug("Stopped robot")
            else:
                self.node.get_logger().error(
                    "Couldnt stop robot: Nodes are either already stopped or error is unknown"
                )
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt stop robot: {e}")
            raise e

    def start_mapping(self) -> None:
        """
        Starts the mapping process. This can only be done if Sopias4 Application is fully running, otherwise
        will directly abort this process. It's basically a wrapper and calling the start_mapping service client
        in the underlying node object.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        try:
            status_response = self.node._start_mapping()
            if status_response:
                self.node.get_logger().info("Started mapping")
            else:
                self.node.get_logger().error(
                    "Couldnt start mapping: Slam node couldn't be launched or unknown error occured"
                )
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt start mapping: {e}")
            raise e

    def stop_mapping(
        self,
        map_path: str = "map_default",
        map_topic: str = "/map",
        image_format: str = "png",
        map_mode: str = "trinary",
        free_thres: float = 0.196,
        occupied_thres: float = 0.65,
    ) -> None:
        """
        Stops the mapping process. This can only be done if Sopias4 Application and the mapping process is fully running,
        otherwise itwill directly abort this process. It's basically a wrapper and calling the start_mapping service
        client in the underlying node object.

        Args:
            map_path(str, optional): The path to the map. Can be absolute or relative to a ros package. Defaults to "maps/map_default"
            map_topic (str, optional): The topic under which the map should be served. Defaults to "/map"
            image_format (str, optional): The image format under which the visualization of the map is saved. Can be either "png" "pgm", or "bmp". Defaults to "png"
            map_mode (str, optional): Map modes: "trinary", "scale" or "raw". Defaults to "trinary"
            free_thres (float, optional): Threshold over which a region is considered as free. Defaults to 0.196
            occupied_thres (float, optional): Threshold over which a region is considered as occupied/obstacle. Defaults to 0.65

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """

        try:
            status_response = self.node._stop_mapping(
                image_format=image_format,
                map_topic=map_topic,
                map_path=map_path,
                occupied_thres=occupied_thres,
                free_thres=free_thres,
                map_mode=map_mode,
            )
            if status_response:
                self.node.get_logger().debug("Stopped Mapping")
            else:
                self.node.get_logger().error(
                    "Couldnt stop mapping: Either SLAM node couldn't be shutdown or a unknown error occured"
                )
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt stop mapping: {e}")
            raise e

    def dock(self) -> None:
        """
        Starts the docking process. This can only be done if the namespace is registered. It's basically a wrapper and calling the
        dock service client in the underlying node object.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        try:
            status_response = self.node._dock()

            if status_response:
                self.node.get_logger().info("Docked")
            else:
                self.node.get_logger().error("Couldnt dock: Unknown error occured")
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt dock: {e}")
            raise e

    def undock(self) -> None:
        """
        Starts the undocking process. This can only be done if the namespace is registered. It's basically a wrapper and calling the
        dock service client in the underlying node object.

        Under normal circumstances, you use this as an callback to connect to Ui element when it is e.g. pressed
        """
        try:
            status_response = self.node._undock()

            if status_response:
                self.node.get_logger().info("Undocked")
            else:
                self.node.get_logger().error("Couldnt undock: Unknown error occured")
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(f"Couldnt undock: {e}")
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
        try:
            status_response = self.node._drive(twist_msg, direction, vel_rel)
            if status_response:
                self.node.get_logger().debug(
                    "Successfully send drive command to Turtlebot"
                )
            else:
                self.node.get_logger().error(
                    "Couldnt send drive command to Turtlebot du to unknown error"
                )
                # Inform user about failure and see for it's response
        except Exception as e:
            # Re-raise exception if one occurs. Only for debugging and shouldn't
            # appear on production if carefully tested
            self.node.get_logger().error(
                f"Couldnt send drive command to Turtlebot: {e}"
            )
            raise e

    def display_dialog(self, request_data: ShowDialog.Request):
        """
        :meta private:
        """
        response_data = ShowDialog.Response()
        dlg = QMessageBox()
        # Set static data that doesn't need validation
        dlg.setWindowTitle(request_data.title)
        dlg.setInformativeText(request_data.content)

        # Choose icon from set of QMessagebox icons
        self.node.get_logger().debug(f"Selected icon: {request_data.icon}")
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
        self.node.get_logger().debug(
            f"Selected interaction options: {request_data.interaction_options}"
        )
        match request_data.interaction_options:
            case ShowDialog.Request.CONFIRM:
                dlg.setStandardButtons(QMessageBox.Ok)
            case ShowDialog.Request.CONFIRM_ABORT:
                dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Abort)
            case ShowDialog.Request.CONFIRM_RETRY:
                dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Retry)
            case ShowDialog.Request.CONFIRM_CANCEL:
                dlg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            case ShowDialog.Request.YES_NO:
                dlg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            case ShowDialog.Request.IGNORE_CANCEL:
                dlg.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ignore)
            case ShowDialog.Request.IGNORE_ABORT:
                dlg.setStandardButtons(QMessageBox.Abort | QMessageBox.Ignore)
            case ShowDialog.Request.IGNORE_RETRY:
                dlg.setStandardButtons(QMessageBox.Retry | QMessageBox.Ignore)
            case _:
                raise ValueError(
                    f"Specified interaction option: {request_data.interaction_options} has isn't implemented \
                        or has wrong value. Implemented icons: {ShowDialog.Request.CONFIRM}, {ShowDialog.Request.CONFIRM_ABORT} \
                        {ShowDialog.Request.CONFIRM_CANCEL}, {ShowDialog.Request.CONFIRM_RETRY} and {ShowDialog.Request.YES_NO}"
                )

        self.node.get_logger().debug(f'Showing dialog "{request_data.title}"')
        # Show dialog and get the pressed button
        selected_option = dlg.exec_()

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

        self.__showed_dialog_signal.emit(response_data)

    def show_file_picker(
        self, info_msg: str = "Open File", initial_path: str | None = None
    ) -> str:
        """
        Opens a file picker to select a file

        Args:
            info_msg (str, optional): How the filepicker should be labeled. Defaults to "Open File"
            intial_path (str, optional): If specified the filepicker opens at this path

        Returns:
            str: Path to the selected file
        """
        # Set initial path is one was given
        if initial_path is not None:
            file = QFileDialog.getOpenFileName(
                self, info_msg, initial_path, "All Files (*)"
            )
        else:
            file = QFileDialog.getOpenFileName(self, info_msg, "", "All Files (*)")

        return file[0]

    def show_filepath_picker(
        self, info_msg: str = "Open Directory", initial_path: str | None = None
    ) -> str:
        """
        Opens a file picker to select a path/directory

        Args:
            info_msg (str, optional): How the filepicker should be labeled. Defaults to "Open File"
            intial_path (str, optional): If specified the filepicker opens at this path

        Returns:
            str: Path to the selected directory
        """
        # Set initial path is one was given
        if initial_path is not None:
            filepath = QFileDialog.getExistingDirectory(
                self,
                info_msg,
                initial_path,
            )
        else:
            filepath = QFileDialog.getExistingDirectory(
                self,
                info_msg,
            )

        return filepath

    def closeEvent(self, event):
        """
        Executes when the GUI is closed

        :meta private:
        """
        for node in self.node_executor.get_nodes():
            self.node_executor.remove_node(node)
            node.destroy_node()
        self.node_executor.wake()
        self.node_executor.shutdown(timeout_sec=10)

        rclpy.shutdown()
        event.accept()


class GrapficalNode(Node):
    """
    This is the ROS2 Node which runs under the hood in the GUI. To avoid problems caused by multiple inheritance etc. \
    the node is done in this separate class which is instantiated as a attribute in the GUI. As a result, this class only
    handles the ROS2 Node specific stuff e.g. implementing service clients, actions clients, subscriptions etc.

    Attributes:
        node_name (str): (optional) Name of the Node. Defaults to gui_node
        namespace (str): (optional) The namespace of the node. When not specified, the GUI has to register itself.\
                                    The namespace should not be set by the developer but instead let the register_namespace service set the namespace
    """

    def __init__(
        self,
        display_dialog_signal,
        showed_dialog_signal,
        node_name: str = "gui_node",
        namespace: str | None = None,
    ) -> None:
        if namespace is not None:
            super().__init__(node_name, namespace=namespace)  # type: ignore
        else:
            ns = "".join(random.choices(string.ascii_lowercase, k=8))
            super().__init__(node_name, namespace=ns)  # type: ignore

        self.show_dialog_signal = display_dialog_signal
        showed_dialog_signal.connect(self.__set_response_data)
        self.dialog_return_data: ShowDialog.Response | None = None
        self.get_logger().info(f"Node started with namespace {self.get_namespace()}")

        # ---- Setup services -----
        self.show_dialog_service = self.create_service(
            ShowDialog, "show_dialog", self.__show_dialog
        )

        # --- Setup service clients ---
        # Create own sub node for service clients so they can spin independently
        self.service_client_node: Node = rclpy.create_node("_gui_service_clients", namespace=self.get_namespace())  # type: ignore
        self.service_client_node.get_logger().set_level(20)
        # This service registers the namespace in the multi robot coordinator
        # inside the Sopias4 Map-server
        self.__mrc_sclient_register: Client = self.service_client_node.create_client(
            RegistryService, "/register_namespace"
        )
        # This service unregisters the namespace in the multi robot coordinator
        # inside the Sopias4 Map-server
        self.__mrc_sclient_manual_unregister: Client = (
            self.service_client_node.create_client(
                RegistryService, "/unregister_namespace"
            )
        )
        # This service launches/connects to the corresponding Turtlebot
        # by launching the nodes of Sopias4 Application
        self.__rm_sclient_launch: Client = self.service_client_node.create_client(
            LaunchNav2Stack, f"{self.get_namespace()}/launch_nav2_stack"
        )
        # This service stops the running nodes of Sopias4 Application
        # so that the system isn't connected anymore to the physical robot
        self.__rm_sclient_stop_robot: Client = self.service_client_node.create_client(
            EmptyWithStatuscode, f"{self.get_namespace()}/stop_nav2_stack"
        )
        # This service starts the mapping process
        self.__rm_sclient_start_mapping: Client = (
            self.service_client_node.create_client(
                EmptyWithStatuscode, f"{self.get_namespace()}/start_mapping"
            )
        )
        # This service stops the mapping provess
        self.__rm_sclient_stop_mapping: Client = self.service_client_node.create_client(
            StopMapping, f"{self.get_namespace()}/stop_mapping"
        )
        # This service sends a manual drive command to the robot
        self.__rm_sclient_drive: Client = self.service_client_node.create_client(
            Drive, f"{self.get_namespace()}/drive"
        )

        # This service lets the tutlebot dock to its charging station
        self.__rm_sclient_dock: Client = self.service_client_node.create_client(
            EmptyWithStatuscode, f"{self.get_namespace()}/dock"
        )
        # This service lets the tutlebot undock from its charging station
        self.__rm_sclient_undock: Client = self.service_client_node.create_client(
            EmptyWithStatuscode, f"{self.get_namespace()}/undock"
        )

    def _register_namespace(self, namespace: str):
        """
        Runs a service client to register the namespace in Sopias4 Map-Server.
        On Failure, the user is informed and has a choice to retry

        Args:
            namespace (str): The namespace which should be registered

        Returns:
            bool: If namespace was registered successfully or not
        """
        self.get_logger().debug(
            f"Sending service request to register namespace {namespace}"
        )
        request: RegistryService.Request = RegistryService.Request()
        request.name_space = f"/{namespace}" if namespace[0] != "/" else namespace
        response: RegistryService.Response | None = node_tools.call_service(
            client=self.__mrc_sclient_register,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == RegistryService.Response.SUCCESS:
            return True
        else:
            # Inform user about error
            msg_2_user = ShowDialog.Request()
            msg_2_user.title = "Error while registering namespace"
            msg_2_user.icon = ShowDialog.Request.ICON_ERROR

            match response.statuscode:
                case RegistryService.Response.COLLISION_ERROR:
                    self.get_logger().error(
                        "Couldn't register namespace: Already registered"
                    )
                    msg_2_user.content = (
                        "Namespace is already registered. Choose another one"
                    )
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM
                case RegistryService.Response.ILLEGAL_NAMESPACE_ERROR:
                    self.get_logger().error(
                        "Couldn't register namespace: Namespace contains illegal characters"
                    )
                    msg_2_user.content = (
                        "Namespace contains illegal characters. Choose another one"
                    )
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM
                case RegistryService.Response.UNKNOWN_ERROR:
                    self.get_logger().error("Couldn't register namespace: Unkown error")
                    msg_2_user.content = "Unknown error occured"
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM_RETRY

            user_response = self.__show_dialog(msg_2_user, ShowDialog.Response())

            # If user response is to retry, then recursively call this function, otherwise return False
            if user_response.selected_option == ShowDialog.Response.CONFIRMED:
                return False
            elif user_response.selected_option == ShowDialog.Response.RETRY:
                return self._register_namespace(namespace)
            else:
                return False

    def _unregister_namespace(self, namespace: str):
        """
        Runs a service client to unregister the namespace in Sopias4 Map-Server.
        On Failure, the user is informed and has a choice to retry

        Args:
            namespace (str): The namespace which should be registered

        Returns:
            bool: If namespace was registered successfully or not
        """
        self.get_logger().debug(
            f"Sending service request to register namespace {namespace}"
        )
        request: RegistryService.Request = RegistryService.Request()
        request.name_space = f"/{namespace}" if namespace[0] != "/" else namespace
        response: RegistryService.Response | None = node_tools.call_service(
            client=self.__mrc_sclient_manual_unregister,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == RegistryService.Response.SUCCESS:
            return True
        else:
            # Inform user about error
            msg_2_user = ShowDialog.Request()
            msg_2_user.title = "Error while unregistering namespace"
            msg_2_user.icon = ShowDialog.Request.ICON_ERROR

            match response.statuscode:
                case RegistryService.Response.NS_NOT_FOUND:
                    self.get_logger().error(
                        "Couldn't unregister namespace: Namespace not found"
                    )
                    msg_2_user.content = "Namespace isn't registered. Choose another one or unregistering is not neccessary"
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM
                case RegistryService.Response.UNKNOWN_ERROR:
                    self.get_logger().error("Couldn't register namespace: Unkown error")
                    msg_2_user.content = "Unknown error occured"
                    msg_2_user.interaction_options = ShowDialog.Request.CONFIRM_RETRY

            user_response = self.__show_dialog(msg_2_user, ShowDialog.Response())

            # If user response is to retry, then recursively call this function, otherwise return False
            if user_response.selected_option == ShowDialog.Response.CONFIRMED:
                return False
            elif user_response.selected_option == ShowDialog.Response.RETRY:
                return self._register_namespace(namespace)
            else:
                return False

    def _launch_nav_stack(self, use_simulation: bool = False) -> bool:
        """
        Runs a service client to start the nodes in Sopias4 Application so the system is connected
        to the robot and ready for operation.

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("Sending service request to launch Turtlebot")
        request = LaunchNav2Stack.Request()
        request.use_simulation = use_simulation
        response: LaunchNav2Stack.Response | None = node_tools.call_service(
            client=self.__rm_sclient_launch,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )
        if response is None:
            return False
        elif response.statuscode == LaunchNav2Stack.Response.SUCCESS:
            return True
        else:
            return False

    def _disconnect_turtlebot(self) -> bool:
        """
        Runs a service client to stop the nodes in Sopias4 Application so the system is disconnected
        from the robot and ready for operation.

        Returns:
            bool: If operation was successful
        """
        # --- Stop the nodes ---
        self.get_logger().debug(
            "Stops Turtlebot. Sending service request to stop nodes"
        )
        stop_request = EmptyWithStatuscode.Request()
        response: EmptyWithStatuscode.Response | None = node_tools.call_service(
            client=self.__rm_sclient_stop_robot,
            service_req=stop_request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def _start_mapping(self) -> bool:
        """
        Runs a service client to start the mapping process. The Sopias4 Application should
        be fully launched before running this service

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("Send service request to start mapping")
        request = EmptyWithStatuscode.Request()
        response: EmptyWithStatuscode.Response | None = node_tools.call_service(
            client=self.__rm_sclient_start_mapping,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def _stop_mapping(
        self,
        map_path: str = "maps/map_default",
        map_topic: str = "/map",
        image_format: str = "png",
        map_mode: str = "trinary",
        free_thres: float = 0.196,
        occupied_thres: float = 0.65,
    ) -> bool:
        """
        Runs a service client to stop the mapping process. The Sopias4 Application should
        be fully launched and the mapping process running before running this service

        Args:
            map_name (str, optional): The path to the map. Can be absolute or relative to a ros package. Defaults to "maps/map_default"
            map_topic (str, optional): The topic under which the map should be served. Defaults to "/map"
            image_format (str, optional): The image format under which the visualization of the map is saved. Can be either "png" "pgm", or "bmp". Defaults to "png"
            map_mode (str, optional): Map modes: "trinary", "scale" or "raw". Defaults to "trinary"
            free_thres (float, optional): Threshold over which a region is considered as free. Defaults to 0.196
            occupied_thres (float, optional): Threshold over which a region is considered as occupied/obstacle. Defaults to 0.65

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("Sending service request to stop mapping")
        request = StopMapping.Request()
        request.map_name = map_path
        request.map_topic = map_topic
        request.image_format = image_format
        request.map_mode = map_mode
        request.free_thres = free_thres
        request.occupied_thres = occupied_thres
        response: StopMapping.Response | None = node_tools.call_service(
            client=self.__rm_sclient_stop_mapping,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == StopMapping.Response.SUCCESS:
            return True
        else:
            return False

    def _drive(
        self,
        twist_msg: Twist | None = None,
        direction: str = "stop",
        vel_rel: float = 1.0,
    ) -> bool:
        """
        Runs a service client to send a driving command to the Turtlebot. The Sopias4 Application
        should be fully launched and the mapping process running before running this service

        Args:
            twist_msg (Twist, optional): The twist message which specifies how the Turtlebot should drive. If None (default), \
                                                            when a appropriate Twist message will be generated (Important: In this casem direction and vel_rel must be provided). 
            direction (str, optional): The direction which the Turtlebot should drive. Can be either forward, backward, left, \
                                                     right, rotate_left, rotate_right and stop. Defaults to stop
            vel_rel (float, optional): The relative velocity. The value is normed to the maximum speed of the Turtlebot,\
                                                     so e.g. 1.0 is maximum speed and 0 is standing still. Defaults to 1.0

        Returns:
            bool: If operation was successful
        """
        request = Drive.Request()
        if twist_msg is None:
            request.twist = drive_tools.generate_twist_msg(
                direction=direction, vel_rel=vel_rel
            )
        else:
            request.twist = twist_msg
        response: Drive.Response | None = node_tools.call_service(
            client=self.__rm_sclient_drive,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=2,
        )

        if response is None:
            return False
        elif response.statuscode == Drive.Response.SUCCESS:
            return True
        else:
            return False

    def _dock(self) -> bool:
        """
        Runs a service client to start the docking process. The namespace should be registered before running this service

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("Send service request to dock")
        request = EmptyWithStatuscode.Request()
        response: EmptyWithStatuscode.Response | None = node_tools.call_service(
            client=self.__rm_sclient_dock,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def _undock(self) -> bool:
        """
        Runs a service client to start the undocking process. The namespace should be registered before running this service

        Returns:
            bool: If operation was successful
        """
        self.get_logger().debug("Send service request to undock")
        request = EmptyWithStatuscode.Request()
        response: EmptyWithStatuscode.Response | None = node_tools.call_service(
            client=self.__rm_sclient_undock,
            service_req=request,
            calling_node=self.service_client_node,
            timeout_sec=5,
        )

        if response is None:
            return False
        elif response.statuscode == EmptyWithStatuscode.Response.SUCCESS:
            return True
        else:
            return False

    def __show_dialog(
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
            f"Got service request to display dialog with title {request_data.title}"
        )
        # Send Signal to main thread so dialog is shown
        self.show_dialog_signal.emit(request_data)

        # Wait until user responded. The feedback is returned via a QT signal which sets the self.dialog_return_data variable
        while self.dialog_return_data is None:
            time.sleep(0.2)

        self.get_logger().debug(
            f"Got user response: {self.dialog_return_data.selected_option}"
        )
        response_data = self.dialog_return_data
        self.dialog_return_data = None
        return response_data

    def __set_response_data(self, data: ShowDialog.Response):
        self.dialog_return_data = data


if __name__ == "__main__":
    print(
        "Only run this node directly for testing purposes. In production this node should be extendend properly"
    )
