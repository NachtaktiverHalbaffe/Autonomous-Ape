#!/usr/bin/env python3
import logging
import os
import subprocess
import sys
from threading import Thread

from ament_index_python import get_package_share_directory
from PyQt5.QtWidgets import QApplication
from sopias4_framework.nodes.gui_node import GUINode
from sopias4_framework.tools.gui.gui_logger import GuiLogger
from sopias4_framework.tools.ros2 import node_tools
from sopias4_map_server.ui_object import Ui_MainWindow


class GUI(GUINode):
    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        super().__init__(Ui_MainWindow(), node_name="gui_sopias4_map_server")
        self.__launch_process_mapserver: subprocess.Popen | None = None

    def connect_labels_to_subscriptions(self):
        GuiLogger(widget=self.ui.textEdit_logger, node=self.node)

    def connect_callbacks(self):
        self.ui.pushButton_bringup_server.clicked.connect(
            lambda: Thread(target=self.__start_map_server).start()
        )
        self.ui.pushButton_stop_map_server.clicked.connect(
            lambda: Thread(target=self.__stop_map_server).start()
        )
        # TODO Service calls for saving and loading maps
        # TODO Loading files

    def set_default_values(self):
        # Tmap saving
        self.ui.comboBox_map_mode.addItems(["trinary", "scale", "raw"])
        self.ui.comboBox_map_mode.setCurrentIndex(0)
        self.ui.comboBox_image_format.addItems(["png", "pgm", "bmp"])
        self.ui.comboBox_image_format.setCurrentIndex(0)
        self.ui.lineEdit_map_name.setText("maps/default_map.yaml")
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
        launch_args_list: list[str] = []
        # TODO check ui elements and add launch args
        if self.ui.checkBox_use_autostart.isChecked():
            launch_args_list.append("autostart:=true")
        if self.ui.checkBox_use_respawn.isChecked():
            launch_args_list.append("use_respawn:=true")

        launch_args: str = " ".join(launch_args_list)
        self.__launch_process_mapserver = node_tools.start_launch_file(
            ros2_package="sopias4_map_server",
            launch_file="bringup_server.launch.py",
            arguments=launch_args,
        )

        self.ui.pushButton_stop_map_server.setEnabled(True)
        self.ui.pushButton_bringup_server.setEnabled(False)
        self.ui.pushButton_launch_mrv.setEnabled(False)
        self.ui.pushButton_launch_map_server.setEnabled(False)

    def __stop_map_server(self):
        if node_tools.shutdown_nodes_launch_file(self.__launch_process_mapserver):
            self.__launch_process_mapserver = None

        self.ui.pushButton_stop_map_server.setEnabled(False)
        self.ui.pushButton_bringup_server.setEnabled(True)
        self.ui.pushButton_launch_mrv.setEnabled(True)
        self.ui.pushButton_launch_map_server.setEnabled(True)

    def closeEvent(self, event):
        node_tools.shutdown_nodes_launch_file(self.__launch_process_mapserver)
        super().closeEvent(event)

    def destroy_node(self):
        node_tools.shutdown_nodes_launch_file(self.__launch_process_mapserver)
        super().destroy_node()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = GUI()
    widget.show()
    sys.exit(app.exec())
