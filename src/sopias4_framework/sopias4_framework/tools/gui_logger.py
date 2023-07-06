from PyQt5.QtGui import QColor, QFont, QTextCursor, QTextOption
from PyQt5.QtWidgets import QTextEdit
from rcl_interfaces.msg import Log

COLOR_DEBUG = QColor(0, 255, 0)
COLOR_INFO = QColor(0, 0, 0)
COLOR_WARNING = QColor(255, 255, 0)
COLOR_ERROR = QColor(255, 0, 0)


class GuiLogger:
    # TODO Docs
    def __init__(self, parent: QTextEdit, fontSize=12, *args, **kwargs):
        super(GuiLogger, self).__init__(*args, **kwargs)
        self.widget = parent
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

    def add_log_msg(self, msg: Log):
        match msg.level:
            case 10:
                self.widget.setTextColor(COLOR_DEBUG)
            case 20:
                self.widget.setTextColor(COLOR_INFO)
            case 30:
                self.widget.setTextColor(COLOR_WARNING)
            case 40:
                self.widget.setTextColor(COLOR_ERROR)

        self.widget.append(msg.msg)
        self.widget.moveCursor(QTextCursor.End)
