#!/usr/bin/env python3
import sys
from threading import Thread

import rclpy
from PyQt5.QtWidgets import QApplication
from sopias4_application.ui_object import Ui_MainWindow
from sopias4_framework.nodes.gui_node import GUINode


class GUI(GUINode):
    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        super().__init__(Ui_MainWindow())

    def connect_callbacks(self):
        #########################
        # ------- Push buttons -------
        #########################
        # --- Tab: System initialization and configuration --
        self.ui.pushButton_namespace.clicked.connect(
            lambda: Thread(
                target=self.register_namespace, args=[self.ui.lineEdit_namespace.text()]
            ).start()
        )
        self.ui.pushButton_launch_turtlebot.clicked.connect(
            lambda: Thread(target=self.launch_robot).start()
        )
        self.ui.pushButton_stop_turtlebot.clicked.connect(
            lambda: Thread(target=self.stop_robot).start()
        )
        # ---  Tab: Mapping ---
        self.ui.pushButton_start_mapping.clicked.connect(
            lambda: Thread(target=self.start_mapping).start()
        )
        self.ui.pushButton_stop_mapping.clicked.connect(
            lambda: Thread(target=self.stop_mapping).start()
        )
        # --- Tab: Advanced/Manual operation ---
        # TODO provide vel_rel
        # Each button is connected to two signals: Pressed which sends the corresponding drive command
        # and released which sends a stop command when the button is released
        self.ui.pushButton_back.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="back",
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
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
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
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
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
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
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
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
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
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
                    vel_rel=self.ui.horizontalSlider_velocity.value() / 100,
                )
            ).start()
        )
        self.ui.pushButton_rotate_left.released.connect(
            lambda: Thread(target=self.drive(direction="stop")).start()
        )


if __name__ == "__main__":
    rclpy.init()

    app = QApplication(sys.argv)
    widget = GUI()
    widget.show()
    rclpy.shutdown()
    sys.exit(app.exec())
