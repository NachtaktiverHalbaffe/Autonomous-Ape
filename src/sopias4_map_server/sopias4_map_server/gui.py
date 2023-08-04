#!/usr/bin/env python3
import os
import subprocess
import sys
from threading import Thread

import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point
from nav2_msgs.srv import LoadMap, SaveMap
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QTableWidgetItem
from rclpy.client import Client
from sopias4_framework.nodes.gui_node import GUINode
from sopias4_framework.tools.gui.gui_logger import GuiLogger
from sopias4_framework.tools.ros2 import node_tools
from sopias4_map_server.ui_object import Ui_MainWindow

from sopias4_msgs.msg import Robot, RobotStates
from sopias4_msgs.srv import ShowDialog


class GUI(GUINode):
    robot_states_signal = pyqtSignal(RobotStates)

    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        super().__init__(Ui_MainWindow(), node_name="gui_sopias4_map_server")
        self.__launch_process_system: subprocess.Popen | None = None
        self.__launch_process_mapserver: subprocess.Popen | None = None
        self.__launch_process_mrc: subprocess.Popen | None = None

        self.robot_states_signal.connect(self.__fill_tableview_robotstates)
        self.__sclient_load_map: Client = self.node.create_client(
            LoadMap, "/map_server/load_map"
        )
        self.__sclient_save_map: Client = self.node.create_client(
            SaveMap, "/map_saver/save_map"
        )

        self.node.create_subscription(
            RobotStates, "/robot_states", self.__callback_robot_states, 10
        )
        self.node.get_logger().set_level(10)

    def connect_labels_to_subscriptions(self):
        GuiLogger(
            widget=self.ui.textEdit_logger,
            node=self.node,
            namespace_filter="no_namespace",
        )

    def connect_callbacks(self):
        # Pushbuttons
        self.ui.pushButton_bringup_server.clicked.connect(
            lambda: Thread(target=self.__start_map_server).start()
        )
        self.ui.pushButton_stop_map_server.clicked.connect(
            lambda: Thread(target=self.__stop_map_server).start()
        )
        self.ui.pushButton_launch_map_server.clicked.connect(
            lambda: Thread(target=self.__launch_mapserver).start()
        )
        self.ui.pushButton_launch_mrv.clicked.connect(
            lambda: Thread(target=self.__launch_mrc).start()
        )
        # Filepickers
        self.ui.pushButton_pick_params_file.clicked.connect(
            lambda: self.ui.lineEdit_path_params_file.setText(
                self.show_file_picker(
                    "Pick configuration file", self.ui.lineEdit_path_params_file.text()
                )
            )
        )
        self.ui.pushButton_load_map.clicked.connect(self.__load_map)
        self.ui.pushButton_save_map.clicked.connect(self.__save_map)

    def set_default_values(self):
        # map saving
        self.ui.comboBox_map_mode.addItems(["trinary", "scale", "raw"])
        self.ui.comboBox_map_mode.setCurrentIndex(0)
        self.ui.comboBox_image_format.addItems(["png", "pgm", "bmp"])
        self.ui.comboBox_image_format.setCurrentIndex(0)
        self.ui.lineEdit_map_name.setText("default_map")
        self.ui.lineEdit_map_topic.setText("/map")
        self.ui.doubleSpinBox_free_thres.setValue(0.196)
        self.ui.doubleSpinBox_occupied_thres.setValue(0.65)
        # Default paths
        self.ui.lineEdit_path_params_file.setText(
            str(
                os.path.join(
                    get_package_share_directory("sopias4_map_server"),
                    "config",
                    "map_server.yaml",
                )
            )
        )
        self.ui.lineEdit.setText(
            str(
                os.path.join(
                    get_package_share_directory("sopias4_map_server"),
                    "maps",
                    "default_map.yaml",
                )
            )
        )

    def set_initial_disabled_elements(self):
        self.ui.checkBox_use_autostart.setChecked(True)
        self.ui.checkBox_use_respawn.setChecked(False)
        self.ui.pushButton_stop_map_server.setEnabled(False)

    def __start_map_server(self):
        """
        Starts the nodes itself. It is done by running a ros2 launch command via a shell command
        """
        self.node.get_logger().info("Starting Sopias4 Map-Server nodes")

        # Construct launch arguments
        launch_args_list: list[str] = []
        if self.ui.checkBox_use_autostart.isChecked():
            self.node.get_logger().debug("Use autostart of lifecycle nodes")
            launch_args_list.append("autostart:=true")
        if self.ui.checkBox_use_respawn.isChecked():
            self.node.get_logger().debug("Use respawning nodes if they crash")
            launch_args_list.append("use_respawn:=true")

        # Launch launchfile in own subprocess
        launch_args: str = " ".join(launch_args_list)
        self.__launch_process_system = node_tools.start_launch_file(
            ros2_package="sopias4_map_server",
            launch_file="bringup_server.launch.py",
            arguments=launch_args,
        )
        # Enable buttons which are only useable when map server launched
        self.ui.pushButton_stop_map_server.setEnabled(True)
        self.ui.pushButton_bringup_server.setEnabled(False)
        self.ui.pushButton_launch_mrv.setEnabled(False)
        self.ui.pushButton_launch_map_server.setEnabled(False)

        self.node.get_logger().info("Successfully started Sopias4 Map-Server nodes")

    def __stop_map_server(self):
        """
        Stops the nodes itself. It is done by sending a SIG-INT signal (STRG+C) to the shell process which runs the nodes
        """
        self.node.get_logger().info("Stopping Sopias4 Mp-Server nodes")
        if node_tools.shutdown_ros_shell_process(self.__launch_process_system):
            self.__launch_process_system = None

        # Enable buttons which are only useable when map server isn't running
        self.ui.pushButton_stop_map_server.setEnabled(False)
        self.ui.pushButton_bringup_server.setEnabled(True)
        self.ui.pushButton_launch_mrv.setEnabled(True)
        self.ui.pushButton_launch_map_server.setEnabled(True)

        self.node.get_logger().info("Successfully stopped Sopias4 Map-Server nodes")

    def __launch_mrc(self):
        self.__launch_process_mrc = node_tools.start_node(
            ros2_package="sopias4_map_server", executable="multi_robot_coordinator"
        )
        self.ui.pushButton_launch_mrv.setEnabled(False)

    def __launch_mapserver(self):
        self.__launch_process_mapserver = node_tools.start_launch_file(
            ros2_package="sopias4_map_server", launch_file="map_server.launch.py"
        )
        self.ui.pushButton_launch_map_server.setEnabled(False)

    def __load_map(self):
        """
        Loads a map from a file which can be picked with a file picker and loads it into the map server
        """
        # Pick map file
        self.node.get_logger().info("Loading map")
        map_path = self.show_file_picker("Load map", self.ui.lineEdit.text())
        if map_path == "":
            self.node.get_logger().debug(
                "Aborted loading map because no map was selected"
            )
            # No map was selected/file picker cancelled
            return

        # Call map saver service
        self.node.get_logger().debug("Sending corresponding service call")
        req = LoadMap.Request()
        req.map_url = map_path
        future = self.__sclient_load_map.call_async(req)
        try:
            rclpy.spin_until_future_complete(
                self.node.service_client_node, future, timeout_sec=10
            )
        except Exception as e:
            self.node.get_logger().warning(f"Couldn't spin node while loading map: {e}")

        response: LoadMap.Response | None = future.result()
        dlg_req = ShowDialog.Request()
        dlg_req.title = "Could't load map"
        dlg_req.icon = ShowDialog.Request.ICON_ERROR
        dlg_req.interaction_options = ShowDialog.Request.CONFIRM
        if response is None:
            return
        elif response.result == LoadMap.Response.RESULT_SUCCESS:
            self.node.get_logger().debug("Successfully loaded map")
            self.ui.lineEdit.setText(map_path)
            return
        # From here it is only executed if service call was an failure
        elif response.result == LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST:
            err_msg = "Couldn't load map because it doesn't exist"
        elif response.result == LoadMap.Response.RESULT_INVALID_MAP_DATA:
            err_msg = "Couldn't load map because map data is invalid"
        elif response.result == LoadMap.Response.RESULT_INVALID_MAP_METADATA:
            err_msg = "Couldn't load map because map has invalid metadata"
        else:
            err_msg = "Couldn't load map because unknown error occured"

        self.node.get_logger().error(err_msg)
        dlg_req.content = err_msg
        self.display_dialog(dlg_req)

    def __save_map(self):
        """
        Saves a map which location can be picked with a file picker
        """
        # Pick map file
        self.node.get_logger().info("Saving map")
        map_path = self.show_filepath_picker("Load map", self.ui.lineEdit.text())
        if map_path == "":
            self.node.get_logger().debug(
                "Aborted loading map because no map was selected"
            )
            # No map was selected/file picker cancelled
            return

        # Call map saver service
        self.node.get_logger().debug("Calling corresponding service")

        req = SaveMap.Request()
        req.map_topic = self.ui.lineEdit_map_topic.text()
        req.map_url = f"{map_path}/{self.ui.lineEdit_map_name.text()}"
        req.map_mode = self.ui.comboBox_map_mode.currentText()
        req.free_thresh = self.ui.doubleSpinBox_free_thres.value()
        req.occupied_thresh = self.ui.doubleSpinBox_occupied_thres.value()
        req.image_format = self.ui.comboBox_image_format.currentText()

        future = self.__sclient_save_map.call_async(req)
        try:
            rclpy.spin_until_future_complete(
                self.node.service_client_node, future, timeout_sec=5
            )
        except Exception as e:
            self.node.get_logger().warning(f"Couldn't spin node while saving map: {e}")

        response: SaveMap.Response | None = future.result()
        if response is None:
            return
        elif response.result:
            self.node.get_logger().info("Successfully saved map")
            self.ui.lineEdit.setText(req.map_url)

    def __fill_tableview_robotstates(self, robot_states: RobotStates):
        """
        Fills the tableview which displays the states of the registered robots
        """
        self.ui.tableWidget_registered_robots.setRowCount(0)
        for robot in robot_states.robot_states:
            row_position = self.ui.tableWidget_registered_robots.rowCount()
            self.ui.tableWidget_registered_robots.insertRow(row_position)
            # Namespace
            self.ui.tableWidget_registered_robots.setItem(
                row_position, 0, QTableWidgetItem(robot.name_space)
            )
            # Current pose
            current_pose: Point = robot.pose.pose.pose.position
            self.ui.tableWidget_registered_robots.setItem(
                row_position,
                1,
                QTableWidgetItem(
                    f"({ round(current_pose.x, 2)}, {round(current_pose.y,2)})"
                ),
            )
            # Current navigation goal
            if len(robot.nav_path.poses) != 0:
                nav_goal: Point = robot.nav_path.poses[-1].pose.position
                self.ui.tableWidget_registered_robots.setItem(
                    row_position,
                    2,
                    QTableWidgetItem(f"({nav_goal.x}, {nav_goal.y})"),
                )
            else:
                self.ui.tableWidget_registered_robots.setItem(
                    row_position,
                    2,
                    QTableWidgetItem("None"),
                )

    def __callback_robot_states(self, robot_states: RobotStates):
        self.robot_states_signal.emit(robot_states)

    def closeEvent(self, event):
        """Cleanup process when GUI is closed"""
        node_tools.shutdown_ros_shell_process(self.__launch_process_system)
        node_tools.shutdown_ros_shell_process(self.__launch_process_mapserver)
        node_tools.shutdown_ros_shell_process(self.__launch_process_mrc)
        super().closeEvent(event)

    def destroy_node(self):
        """Cleanup process when GUI node is destroyed"""
        node_tools.shutdown_ros_shell_process(self.__launch_process_system)
        node_tools.shutdown_ros_shell_process(self.__launch_process_mapserver)
        node_tools.shutdown_ros_shell_process(self.__launch_process_mrc)
        super().destroy_node()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = GUI()
    widget.show()
    sys.exit(app.exec())
