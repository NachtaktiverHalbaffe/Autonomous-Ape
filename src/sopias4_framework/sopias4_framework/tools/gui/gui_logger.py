import rclpy
from PyQt5.QtGui import QColor, QFont, QTextCursor, QTextOption
from PyQt5.QtWidgets import QTextEdit
from rcl_interfaces.msg import Log
from rclpy.node import Node

COLOR_DEBUG = QColor(0, 255, 0)
COLOR_INFO = QColor(0, 0, 0)
COLOR_WARNING = QColor(255, 255, 0)
COLOR_ERROR = QColor(255, 0, 0)


class GuiLogger:
    """
    This class takes a QTextEdit element as a widget and displays the ros_messages in this element. For this\
    purpose, it internally creates a subscription to the /rosout topic where all log messages are sent to.\
    For this purpose, it is neccessary that the gui node is running. As a best practise, instantiate this logger\
    inside the `connect_labels_to_subscriptions()` method. 

    Example:
    
    .. highlight:: python
    .. code-block:: python

            class YourGuiImplementation(GUINode):
                def __init__(self) -> None:
                    self.ui: Ui_MainWindow
                    super().__init__(Ui_MainWindow())


                def connect_labels_to_subscriptions(self):
                    # reference the desired QTextEdit element and the gui node here
                    GuiLogger(widget=self.ui.textEdit, node=self.node)
    """

    def __init__(
        self, widget: QTextEdit, node: Node, fontSize: int = 12, *args, **kwargs
    ):
        """
        Args:
            widget (QTextEdit): The QTextEdit widget which you want to convert to an logging box
            node (rclpy.Node): The node which runs the gui
            fontsize (int, optional): The font size of the log messages. Defaults to 12
        """
        super(GuiLogger, self).__init__(*args, **kwargs)
        self.widget = widget
        # Disable the editability of the edittextbox
        self.widget.setReadOnly(True)
        # Set text Size
        font = QFont()
        font.setPointSize(fontSize)
        self.widget.setFont(font)
        # Set textwrap modes
        self.widget.setLineWrapMode(QTextEdit.LineWrapMode.WidgetWidth)
        self.widget.setWordWrapMode(QTextOption.WrapMode.WrapAtWordBoundaryOrAnywhere)
        self.widget.setTextColor(COLOR_INFO)
        node.create_subscription(Log, "/rosout", self.add_log_msg, 10)

    def add_log_msg(self, msg: Log):
        """
        Appends a log message to the text box

        :meta private:
        """
        match msg.level:
            case 10:
                self.widget.setTextColor(COLOR_DEBUG)
            case 20:
                self.widget.setTextColor(COLOR_INFO)
            case 30:
                self.widget.setTextColor(COLOR_WARNING)
            case 40:
                self.widget.setTextColor(COLOR_ERROR)

        formatted_log = f"[{msg.name}] {msg.msg}"
        self.widget.append(formatted_log)
        self.widget.moveCursor(QTextCursor.End)


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
