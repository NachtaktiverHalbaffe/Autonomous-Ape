from irobot_create_msgs.msg import DockStatus, KidnapStatus, WheelVels
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QLabel
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class LabelSubscriptionHandler(QObject):
    """
    This class takes a QLabel element as a widget and connects it to an ROS2 subscription to\
    to display the content of its messages in the GUI

    Example:
    
    .. highlight:: python
    .. code-block:: python

            class YourGuiImplementation(GUINode):
                
                def __init__(self) -> None:
                    self.ui: Ui_MainWindow
                    super().__init__(Ui_MainWindow())

                def connect_labels_to_subscriptions(self):
                    # reference the desired QLabel element, the gui node and the message type here
                    LabelSubscriptionHandler(
                        widget=self.ui.label_battery, node=self.node, message_type=BatteryState
                    )

    """

    value_signal = pyqtSignal(str)

    def __init__(self, widget: QLabel, node: Node, message_type):
        """
        Args:
            widget (QLabel): The QLabel element onto which the content should be printed
            node (rclpy.Node): The gui node
            message_type: Type of the message which should be printed
        """
        super().__init__()
        self.widget = widget
        self.value_signal.connect(self.__set_text)

        if message_type == BatteryState:
            node.create_subscription(
                BatteryState,
                f"{node.get_namespace()}/battery_state",
                self.set_label_battery,
                10,
            )
        elif message_type == WheelVels:
            node.create_subscription(
                WheelVels,
                f"{node.get_namespace()}/wheel_vels",
                self.set_label_velocity,
                10,
            )
        elif message_type == DockStatus:
            node.create_subscription(
                DockStatus,
                f"{node.get_namespace()}/dock",
                self.set_label_dockstatus,
                10,
            )
        elif message_type == KidnapStatus:
            node.create_subscription(
                KidnapStatus,
                f"{node.get_namespace()}/kidnap_status",
                self.set_label_kidnap_status,
                10,
            )
        else:
            raise NotImplementedError(
                f"{type(message_type)} not implemented as a message type"
            )

    def __set_text(self, msg: str):
        self.widget.setText(msg)

    def set_label_kidnap_status(self, msg: KidnapStatus):
        """
        :meta private:
        """
        self.value_signal.emit(str(msg.is_kidnapped))

    def set_label_battery(self, msg: BatteryState):
        """
        :meta private:
        """
        self.value_signal.emit(str(round(msg.percentage, 2)))

    def set_label_dockstatus(self, msg: DockStatus):
        """
        :meta private:
        """
        self.value_signal.emit(str(msg.is_docked))

    def set_label_velocity(self, msg: WheelVels):
        """
        :meta private:
        """
        vel = (msg.velocity_left + msg.velocity_right) / 2
        self.value_signal.emit(str(round(vel, 2)))


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
