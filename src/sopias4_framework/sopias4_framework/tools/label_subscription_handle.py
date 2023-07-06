from irobot_create_msgs.msg import DockStatus, KidnapStatus, WheelVels
from PyQt5.QtWidgets import QLabel
from sensor_msgs.msg import BatteryState


class LabelSubscriptionHandler:
    def __init__(self, widget: QLabel):
        self.widget = widget

    def set_label_kidnap_status(self, msg: KidnapStatus):
        self.widget.setText(str(msg.is_kidnapped))

    def set_label_battery(self, msg: BatteryState):
        self.widget.setText(str(msg.percentage))

    def set_label_dockstatus(self, msg: DockStatus):
        self.widget.setText(str(msg.is_docked))

    def set_label_velocity(self, msg: WheelVels):
        vel = (msg.velocity_left + msg.velocity_right) / 2
        self.widget.setText(str(vel))
