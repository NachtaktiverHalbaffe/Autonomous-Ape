from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import DockStatus, KidnapStatus, WheelVels
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QLabel
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


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

                def connect_ui_callbacks(self):
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
            self.sub = node.create_subscription(
                BatteryState,
                f"{node.get_namespace()}/battery_state"
                if node.get_namespace() != "/"
                else "/battery_state",
                self.set_label_battery,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=5,
                ),
            )
        elif message_type == WheelVels:
            self.sub = node.create_subscription(
                WheelVels,
                f"{node.get_namespace()}/wheel_vels"
                if node.get_namespace() != "/"
                else "/wheels_vels",
                self.set_label_velocity,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=5,
                ),
            )
        elif message_type == DockStatus:
            self.sub = node.create_subscription(
                DockStatus,
                f"{node.get_namespace()}/dock_status"
                if node.get_logger() != "/"
                else "/dock_status",
                self.set_label_dockstatus,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=5,
                ),
            )
        elif message_type == KidnapStatus:
            self.sub = node.create_subscription(
                KidnapStatus,
                f"{node.get_namespace()}/kidnap_status"
                if node.get_namespace() != "/"
                else "/kidnap_status",
                self.set_label_kidnap_status,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=5,
                ),
            )
        elif message_type == Bool:
            self.sub = node.create_subscription(
                Bool,
                f"{node.get_namespace()}/is_navigating"
                if node.get_namespace() != "/"
                else "/is_navigating",
                self.set_label_navstatus,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                    depth=5,
                ),
            )
        else:
            raise NotImplementedError(
                f"{type(message_type)} not implemented as a message type"
            )

    def __del__(self):
        self.sub.destroy()

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
        # average wheel speed * wheel radius
        vel = ((msg.velocity_left + msg.velocity_right) / 2) * 0.035
        self.value_signal.emit(str(round(vel, 2)))

    def set_label_navstatus(self, msg: Bool):
        self.value_signal.emit(str(msg.data))


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
