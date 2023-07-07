#!/usr/bin/env python3
import logging
import sys
from threading import Thread

from irobot_create_msgs.msg import DockStatus, KidnapStatus, WheelVels
from PyQt5.QtWidgets import QApplication
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState
from sopias4_application.ui_object import Ui_MainWindow
from sopias4_framework.nodes.gui_node import GUINode
from sopias4_framework.tools.gui_logger import GuiLogger
from sopias4_framework.tools.label_subscription_handle import LabelSubscriptionHandler


class GUI(GUINode):
    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        super().__init__(Ui_MainWindow())

    def connect_labels_to_subscriptions(self):
        GuiLogger(widget=self.ui.textEdit, node=self.node)
        try:
            LabelSubscriptionHandler(
                widget=self.ui.label_battery, node=self.node, message_type=BatteryState
            )
            LabelSubscriptionHandler(self.ui.label_current_speed, self.node, WheelVels)
            LabelSubscriptionHandler(self.ui.label_docked, self.node, DockStatus)
            LabelSubscriptionHandler(self.ui.label_kidnapped, self.node, KidnapStatus)

        except Exception as e:
            self.node.get_logger().error(f"Couldn't add LabelSubscriptionHandler: {e}")

    def connect_callbacks(self):
        #########################
        # ------- Push buttons -------
        #########################
        # --- Tab: System initialization and configuration ---
        self.ui.pushButton_namespace.clicked.connect(
            lambda: Thread(target=self.__register_turtlebot).start()
        )
        self.ui.pushButton_launch_turtlebot.clicked.connect(
            lambda: Thread(target=self.__launch_turtlebot).start()
        )
        self.ui.pushButton_stop_turtlebot.clicked.connect(
            lambda: Thread(target=self.__stop_turtlebot).start()
        )
        # ---  Tab: Mapping ---
        self.ui.pushButton_start_mapping.clicked.connect(
            lambda: Thread(target=self.__start_mapping).start()
        )
        self.ui.pushButton_stop_mapping.clicked.connect(
            lambda: Thread(
                target=self.__stop_mapping,
                kwargs={
                    "map_path": self.ui.lineEdit_map_name.text(),
                    "map_topic": self.ui.lineEdit_map_topic.text(),
                    "image_format": self.ui.comboBox_image_format.currentText(),
                    "map_mode": self.ui.comboBox_map_mode.currentText(),
                    "free_thres": float(self.ui.doubleSpinBox_free_thres.value()),
                    "occupied_thres": float(
                        self.ui.doubleSpinBox_occupied_thres.value()
                    ),
                },
            ).start()
        )
        # --- Tab: Advanced/Manual operation ---
        # Each button is connected to two signals: Pressed which sends the corresponding drive command
        # and released which sends a stop command when the button is released
        self.ui.pushButton_back.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="backward",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_back.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )
        self.ui.pushButton_forward.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="forward",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_forward.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )
        self.ui.pushButton_left.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="left",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_left.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )
        self.ui.pushButton_right.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="right",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_right.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )
        self.ui.pushButton_rotate_right.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="rotate_right",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_rotate_right.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )
        self.ui.pushButton_rotate_left.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="rotate_left",
                    vel_rel=float(self.ui.horizontalSlider_velocity.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_rotate_left.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )

    def set_default_values(self):
        self.ui.comboBox_map_mode.addItems(["trinary", "scale", "raw"])
        self.ui.comboBox_map_mode.setCurrentIndex(0)
        self.ui.comboBox_image_format.addItems(["png", "pgm", "bmp"])
        self.ui.comboBox_image_format.setCurrentIndex(0)
        self.ui.lineEdit_map_name.setText("maps/default_map")
        self.ui.lineEdit_map_topic.setText("/map")
        self.ui.doubleSpinBox_free_thres.setValue(0.196)
        self.ui.doubleSpinBox_occupied_thres.setValue(0.65)
        self.ui.label_kidnapped.setText("Unknown")
        self.ui.label_battery.setText("Unknown")
        self.ui.label_docked.setText("Unknown")
        self.ui.label_is_navigating.setText("Unknown")
        self.ui.label_kidnapped.setText("Unknown")
        self.ui.label_current_speed.setText("Unknown")

    def set_initial_disabled_elements(self):
        # --- Tab: System initialization and configuration ---
        self.ui.pushButton_launch_turtlebot.setEnabled(False)
        self.ui.pushButton_stop_turtlebot.setEnabled(False)
        # --- Tab: Mapping ---
        self.ui.pushButton_start_mapping.setEnabled(False)
        self.ui.pushButton_stop_mapping.setEnabled(False)
        # --- Tab: Manual and advanced operation ---
        self.__enable_drive_buttons(False)

    def __register_turtlebot(self):
        if self.register_namespace(self.ui.lineEdit_namespace.text()):
            self.ui.pushButton_launch_turtlebot.setEnabled(True)
            self.ui.pushButton_namespace.setEnabled(False)

    def __launch_turtlebot(self):
        self.launch_robot(use_simulation=self.ui.checkBox_use_simulation.isChecked())
        self.ui.pushButton_start_mapping.setEnabled(True)
        self.ui.pushButton_stop_turtlebot.setEnabled(True)
        self.ui.pushButton_launch_turtlebot.setEnabled(False)
        self.__enable_drive_buttons(True)

    def __stop_turtlebot(self):
        self.stop_robot()
        self.ui.pushButton_start_mapping.setEnabled(False)
        self.ui.pushButton_stop_turtlebot.setEnabled(False)
        self.ui.pushButton_launch_turtlebot.setEnabled(True)
        self.ui.pushButton_left.setEnabled(True)
        self.__enable_drive_buttons(False)

    def __start_mapping(self):
        self.start_mapping()
        self.ui.pushButton_stop_mapping.setEnabled(True)
        self.ui.pushButton_start_mapping.setEnabled(False)

    def __stop_mapping(
        self,
        map_path: str,
        map_topic: str,
        image_format: str,
        map_mode: str,
        free_thres: float,
        occupied_thres: float,
    ):
        self.stop_mapping(
            map_path=map_path,
            map_mode=map_mode,
            map_topic=map_topic,
            image_format=image_format,
            free_thres=free_thres,
            occupied_thres=occupied_thres,
        )
        self.ui.pushButton_stop_mapping.setEnabled(False)
        self.ui.pushButton_start_mapping.setEnabled(True)

    def __enable_drive_buttons(self, isEnabled: bool):
        self.ui.pushButton_forward.setEnabled(isEnabled)
        self.ui.pushButton_back.setEnabled(isEnabled)
        self.ui.pushButton_left.setEnabled(isEnabled)
        self.ui.pushButton_right.setEnabled(isEnabled)
        self.ui.pushButton_rotate_left.setEnabled(isEnabled)
        self.ui.pushButton_rotate_right.setEnabled(isEnabled)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = GUI()
    widget.show()
    sys.exit(app.exec())
