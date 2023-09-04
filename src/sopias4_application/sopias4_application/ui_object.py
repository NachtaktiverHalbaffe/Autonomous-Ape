# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'src/sopias4_application/assets/frontend/gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1580, 650)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_conf_init = QtWidgets.QWidget()
        self.tab_conf_init.setObjectName("tab_conf_init")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.tab_conf_init)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label = QtWidgets.QLabel(self.tab_conf_init)
        self.label.setTextFormat(QtCore.Qt.MarkdownText)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_2.addWidget(self.label)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_2.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)
        self.label_3 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_3.addWidget(self.label_3)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_4 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_4.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 2, 0, 1, 1)
        self.pushButton_unregister = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_unregister.setObjectName("pushButton_unregister")
        self.gridLayout.addWidget(self.pushButton_unregister, 2, 3, 1, 1)
        self.comboBox_namespace = QtWidgets.QComboBox(self.tab_conf_init)
        self.comboBox_namespace.setObjectName("comboBox_namespace")
        self.gridLayout.addWidget(self.comboBox_namespace, 2, 1, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem, 2, 5, 1, 1)
        self.pushButton_namespace = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_namespace.setObjectName("pushButton_namespace")
        self.gridLayout.addWidget(self.pushButton_namespace, 2, 4, 1, 1)
        self.verticalLayout_3.addLayout(self.gridLayout)
        self.verticalLayout_2.addLayout(self.verticalLayout_3)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_5 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_5.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_4.addWidget(self.label_5)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.label_8 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_8.setObjectName("label_8")
        self.verticalLayout_4.addWidget(self.label_8)
        self.checkBox_use_simulation = QtWidgets.QCheckBox(self.tab_conf_init)
        self.checkBox_use_simulation.setObjectName("checkBox_use_simulation")
        self.verticalLayout_4.addWidget(self.checkBox_use_simulation)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pushButton_launch_turtlebot = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_turtlebot.setObjectName("pushButton_launch_turtlebot")
        self.horizontalLayout_2.addWidget(self.pushButton_launch_turtlebot)
        self.pushButton_stop_turtlebot = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_stop_turtlebot.setObjectName("pushButton_stop_turtlebot")
        self.horizontalLayout_2.addWidget(self.pushButton_stop_turtlebot)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.label_6 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_6.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.label_7 = QtWidgets.QLabel(self.tab_conf_init)
        self.label_7.setObjectName("label_7")
        self.verticalLayout_4.addWidget(self.label_7)
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.pushButton_launch_rviz2 = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_rviz2.setObjectName("pushButton_launch_rviz2")
        self.gridLayout_6.addWidget(self.pushButton_launch_rviz2, 1, 0, 1, 1)
        self.pushButton_launch_amcl = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_amcl.setObjectName("pushButton_launch_amcl")
        self.gridLayout_6.addWidget(self.pushButton_launch_amcl, 1, 2, 1, 1)
        self.pushButton_launch_nav2 = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_nav2.setObjectName("pushButton_launch_nav2")
        self.gridLayout_6.addWidget(self.pushButton_launch_nav2, 1, 1, 1, 1)
        self.pushButton_launch_pathlayer = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_launch_pathlayer.setObjectName("pushButton_launch_pathlayer")
        self.gridLayout_6.addWidget(self.pushButton_launch_pathlayer, 2, 0, 1, 1)
        self.pushButton_stop_pathlayer = QtWidgets.QPushButton(self.tab_conf_init)
        self.pushButton_stop_pathlayer.setObjectName("pushButton_stop_pathlayer")
        self.gridLayout_6.addWidget(self.pushButton_stop_pathlayer, 2, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_6)
        self.verticalLayout_2.addLayout(self.verticalLayout_4)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem1)
        self.tabWidget.addTab(self.tab_conf_init, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.tab_2)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_14 = QtWidgets.QLabel(self.tab_2)
        self.label_14.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_6.addWidget(self.label_14)
        self.label_10 = QtWidgets.QLabel(self.tab_2)
        self.label_10.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_10.setObjectName("label_10")
        self.verticalLayout_6.addWidget(self.label_10)
        self.label_11 = QtWidgets.QLabel(self.tab_2)
        self.label_11.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_11.setObjectName("label_11")
        self.verticalLayout_6.addWidget(self.label_11)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_13 = QtWidgets.QLabel(self.tab_2)
        self.label_13.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_13.setObjectName("label_13")
        self.gridLayout_2.addWidget(self.label_13, 0, 2, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.tab_2)
        self.label_12.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_12.setObjectName("label_12")
        self.gridLayout_2.addWidget(self.label_12, 0, 0, 1, 1)
        self.label_is_navigating = QtWidgets.QLabel(self.tab_2)
        self.label_is_navigating.setObjectName("label_is_navigating")
        self.gridLayout_2.addWidget(self.label_is_navigating, 0, 3, 1, 1)
        self.label_current_speed = QtWidgets.QLabel(self.tab_2)
        self.label_current_speed.setObjectName("label_current_speed")
        self.gridLayout_2.addWidget(self.label_current_speed, 0, 1, 1, 1)
        self.verticalLayout_6.addLayout(self.gridLayout_2)
        self.label_15 = QtWidgets.QLabel(self.tab_2)
        self.label_15.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_15.setObjectName("label_15")
        self.verticalLayout_6.addWidget(self.label_15)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label_18 = QtWidgets.QLabel(self.tab_2)
        self.label_18.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_18.setObjectName("label_18")
        self.gridLayout_3.addWidget(self.label_18, 0, 4, 1, 1)
        self.label_battery = QtWidgets.QLabel(self.tab_2)
        self.label_battery.setObjectName("label_battery")
        self.gridLayout_3.addWidget(self.label_battery, 0, 1, 1, 1)
        self.label_docked = QtWidgets.QLabel(self.tab_2)
        self.label_docked.setObjectName("label_docked")
        self.gridLayout_3.addWidget(self.label_docked, 0, 3, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.tab_2)
        self.label_16.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_16.setObjectName("label_16")
        self.gridLayout_3.addWidget(self.label_16, 0, 0, 1, 1)
        self.label_17 = QtWidgets.QLabel(self.tab_2)
        self.label_17.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_17.setObjectName("label_17")
        self.gridLayout_3.addWidget(self.label_17, 0, 2, 1, 1)
        self.label_kidnapped = QtWidgets.QLabel(self.tab_2)
        self.label_kidnapped.setObjectName("label_kidnapped")
        self.gridLayout_3.addWidget(self.label_kidnapped, 0, 5, 1, 1)
        self.verticalLayout_6.addLayout(self.gridLayout_3)
        self.verticalLayout_7.addLayout(self.verticalLayout_6)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_9 = QtWidgets.QLabel(self.tab_2)
        self.label_9.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_9.setObjectName("label_9")
        self.verticalLayout_5.addWidget(self.label_9)
        self.textEdit = QtWidgets.QTextEdit(self.tab_2)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_5.addWidget(self.textEdit)
        self.verticalLayout_7.addLayout(self.verticalLayout_5)
        self.tabWidget.addTab(self.tab_2, "")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayout_10 = QtWidgets.QVBoxLayout(self.tab)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.label_19 = QtWidgets.QLabel(self.tab)
        self.label_19.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.verticalLayout_10.addWidget(self.label_19)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_20 = QtWidgets.QLabel(self.tab)
        self.label_20.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_20.setObjectName("label_20")
        self.verticalLayout_8.addWidget(self.label_20)
        self.label_21 = QtWidgets.QLabel(self.tab)
        self.label_21.setObjectName("label_21")
        self.verticalLayout_8.addWidget(self.label_21)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.pushButton_start_mapping = QtWidgets.QPushButton(self.tab)
        self.pushButton_start_mapping.setObjectName("pushButton_start_mapping")
        self.horizontalLayout_3.addWidget(self.pushButton_start_mapping)
        self.pushButton_stop_mapping = QtWidgets.QPushButton(self.tab)
        self.pushButton_stop_mapping.setObjectName("pushButton_stop_mapping")
        self.horizontalLayout_3.addWidget(self.pushButton_stop_mapping)
        self.verticalLayout_8.addLayout(self.horizontalLayout_3)
        self.verticalLayout_10.addLayout(self.verticalLayout_8)
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label_22 = QtWidgets.QLabel(self.tab)
        self.label_22.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_22.setObjectName("label_22")
        self.verticalLayout_9.addWidget(self.label_22)
        self.label_23 = QtWidgets.QLabel(self.tab)
        self.label_23.setObjectName("label_23")
        self.verticalLayout_9.addWidget(self.label_23)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.lineEdit_map_name = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_map_name.setObjectName("lineEdit_map_name")
        self.gridLayout_4.addWidget(self.lineEdit_map_name, 0, 1, 1, 1)
        self.label_24 = QtWidgets.QLabel(self.tab)
        self.label_24.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_24.setObjectName("label_24")
        self.gridLayout_4.addWidget(self.label_24, 0, 0, 1, 1)
        self.comboBox_image_format = QtWidgets.QComboBox(self.tab)
        self.comboBox_image_format.setObjectName("comboBox_image_format")
        self.gridLayout_4.addWidget(self.comboBox_image_format, 1, 3, 1, 1)
        self.label_27 = QtWidgets.QLabel(self.tab)
        self.label_27.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_27.setObjectName("label_27")
        self.gridLayout_4.addWidget(self.label_27, 0, 2, 1, 1)
        self.label_26 = QtWidgets.QLabel(self.tab)
        self.label_26.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_26.setObjectName("label_26")
        self.gridLayout_4.addWidget(self.label_26, 1, 2, 1, 1)
        self.comboBox_map_mode = QtWidgets.QComboBox(self.tab)
        self.comboBox_map_mode.setObjectName("comboBox_map_mode")
        self.gridLayout_4.addWidget(self.comboBox_map_mode, 1, 1, 1, 1)
        self.lineEdit_map_topic = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_map_topic.setObjectName("lineEdit_map_topic")
        self.gridLayout_4.addWidget(self.lineEdit_map_topic, 0, 3, 1, 1)
        self.label_25 = QtWidgets.QLabel(self.tab)
        self.label_25.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_25.setObjectName("label_25")
        self.gridLayout_4.addWidget(self.label_25, 1, 0, 1, 1)
        self.label_28 = QtWidgets.QLabel(self.tab)
        self.label_28.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_28.setObjectName("label_28")
        self.gridLayout_4.addWidget(self.label_28, 2, 0, 1, 1)
        self.doubleSpinBox_occupied_thres = QtWidgets.QDoubleSpinBox(self.tab)
        self.doubleSpinBox_occupied_thres.setMaximum(1.0)
        self.doubleSpinBox_occupied_thres.setSingleStep(0.01)
        self.doubleSpinBox_occupied_thres.setObjectName("doubleSpinBox_occupied_thres")
        self.gridLayout_4.addWidget(self.doubleSpinBox_occupied_thres, 2, 1, 1, 1)
        self.label_29 = QtWidgets.QLabel(self.tab)
        self.label_29.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_29.setObjectName("label_29")
        self.gridLayout_4.addWidget(self.label_29, 2, 2, 1, 1)
        self.doubleSpinBox_free_thres = QtWidgets.QDoubleSpinBox(self.tab)
        self.doubleSpinBox_free_thres.setMaximum(1.0)
        self.doubleSpinBox_free_thres.setSingleStep(0.01)
        self.doubleSpinBox_free_thres.setObjectName("doubleSpinBox_free_thres")
        self.gridLayout_4.addWidget(self.doubleSpinBox_free_thres, 2, 3, 1, 1)
        self.verticalLayout_9.addLayout(self.gridLayout_4)
        self.verticalLayout_10.addLayout(self.verticalLayout_9)
        spacerItem2 = QtWidgets.QSpacerItem(20, 446, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_10.addItem(spacerItem2)
        self.tabWidget.addTab(self.tab, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.verticalLayout_14 = QtWidgets.QVBoxLayout(self.tab_4)
        self.verticalLayout_14.setObjectName("verticalLayout_14")
        self.label_30 = QtWidgets.QLabel(self.tab_4)
        self.label_30.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_30.setAlignment(QtCore.Qt.AlignCenter)
        self.label_30.setObjectName("label_30")
        self.verticalLayout_14.addWidget(self.label_30)
        self.verticalLayout_12 = QtWidgets.QVBoxLayout()
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.label_32 = QtWidgets.QLabel(self.tab_4)
        self.label_32.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_32.setObjectName("label_32")
        self.verticalLayout_12.addWidget(self.label_32)
        self.label_33 = QtWidgets.QLabel(self.tab_4)
        self.label_33.setObjectName("label_33")
        self.verticalLayout_12.addWidget(self.label_33)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.pushButton_right = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_right.setObjectName("pushButton_right")
        self.gridLayout_5.addWidget(self.pushButton_right, 1, 3, 1, 1)
        self.pushButton_left = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_left.setObjectName("pushButton_left")
        self.gridLayout_5.addWidget(self.pushButton_left, 1, 1, 1, 1)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem3, 0, 0, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem4, 0, 4, 1, 1)
        self.pushButton_forward = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_forward.setObjectName("pushButton_forward")
        self.gridLayout_5.addWidget(self.pushButton_forward, 0, 2, 1, 1)
        self.pushButton_back = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_back.setObjectName("pushButton_back")
        self.gridLayout_5.addWidget(self.pushButton_back, 1, 2, 1, 1)
        self.horizontalLayout_4.addLayout(self.gridLayout_5)
        self.verticalLayout_11 = QtWidgets.QVBoxLayout()
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.horizontalSlider_velocity = QtWidgets.QSlider(self.tab_4)
        self.horizontalSlider_velocity.setMaximum(100)
        self.horizontalSlider_velocity.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_velocity.setObjectName("horizontalSlider_velocity")
        self.verticalLayout_11.addWidget(self.horizontalSlider_velocity)
        self.label_31 = QtWidgets.QLabel(self.tab_4)
        self.label_31.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_31.setAlignment(QtCore.Qt.AlignCenter)
        self.label_31.setObjectName("label_31")
        self.verticalLayout_11.addWidget(self.label_31)
        self.horizontalLayout_4.addLayout(self.verticalLayout_11)
        self.verticalLayout_12.addLayout(self.horizontalLayout_4)
        self.verticalLayout_14.addLayout(self.verticalLayout_12)
        self.verticalLayout_13 = QtWidgets.QVBoxLayout()
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.label_34 = QtWidgets.QLabel(self.tab_4)
        self.label_34.setTextFormat(QtCore.Qt.MarkdownText)
        self.label_34.setObjectName("label_34")
        self.verticalLayout_13.addWidget(self.label_34)
        self.label_35 = QtWidgets.QLabel(self.tab_4)
        self.label_35.setObjectName("label_35")
        self.verticalLayout_13.addWidget(self.label_35)
        self.gridLayout_7 = QtWidgets.QGridLayout()
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.pushButton_dock = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_dock.setObjectName("pushButton_dock")
        self.gridLayout_7.addWidget(self.pushButton_dock, 0, 0, 1, 1)
        self.pushButton_undock = QtWidgets.QPushButton(self.tab_4)
        self.pushButton_undock.setObjectName("pushButton_undock")
        self.gridLayout_7.addWidget(self.pushButton_undock, 0, 1, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_7.addItem(spacerItem5, 0, 2, 1, 1)
        self.verticalLayout_13.addLayout(self.gridLayout_7)
        self.verticalLayout_14.addLayout(self.verticalLayout_13)
        spacerItem6 = QtWidgets.QSpacerItem(20, 330, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_14.addItem(spacerItem6)
        self.tabWidget.addTab(self.tab_4, "")
        self.verticalLayout.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Sopias4 Application"))
        self.label.setText(_translate("MainWindow", "# Initialization and Configuration"))
        self.label_2.setText(_translate("MainWindow", "## Register a namespace"))
        self.label_3.setText(_translate("MainWindow", "Register a unique namespace in the system. The name space is used to identify the individual robot. Because of this, this has to be done before launching the Turtlebot itself"))
        self.label_4.setText(_translate("MainWindow", "**Name space:**"))
        self.pushButton_unregister.setText(_translate("MainWindow", "Unregister"))
        self.pushButton_namespace.setText(_translate("MainWindow", "Register"))
        self.label_5.setText(_translate("MainWindow", "## Manage Turtlebot"))
        self.label_8.setText(_translate("MainWindow", "Before launching the Turtlebot, make sure the name space is registered. If using the simulation, then make sure to set the namespace also inside gazebo"))
        self.checkBox_use_simulation.setText(_translate("MainWindow", "Use simulation"))
        self.pushButton_launch_turtlebot.setText(_translate("MainWindow", "Launch navigation stack"))
        self.pushButton_stop_turtlebot.setText(_translate("MainWindow", "Stop navigation stack"))
        self.label_6.setText(_translate("MainWindow", "### Manual launch sub-components"))
        self.label_7.setText(_translate("MainWindow", "By launching sub-components individually, the \"Stop Turtlebot\" button doesn\'t work as intended."))
        self.pushButton_launch_rviz2.setText(_translate("MainWindow", "Launch Rviz2"))
        self.pushButton_launch_amcl.setText(_translate("MainWindow", "Launch AMCL"))
        self.pushButton_launch_nav2.setText(_translate("MainWindow", "Launch Navigation2"))
        self.pushButton_launch_pathlayer.setText(_translate("MainWindow", "Launch Path-Layer"))
        self.pushButton_stop_pathlayer.setText(_translate("MainWindow", "Stop Path-Layer"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_conf_init), _translate("MainWindow", "Initialization/Configuration"))
        self.label_14.setText(_translate("MainWindow", "# Monitoring"))
        self.label_10.setText(_translate("MainWindow", "## Turtlebot state"))
        self.label_11.setText(_translate("MainWindow", "### Navigation"))
        self.label_13.setText(_translate("MainWindow", "**Is navigating:**"))
        self.label_12.setText(_translate("MainWindow", "**Current speed:**"))
        self.label_is_navigating.setText(_translate("MainWindow", "TextLabel"))
        self.label_current_speed.setText(_translate("MainWindow", "TextLabel"))
        self.label_15.setText(_translate("MainWindow", "### Turtlebot status"))
        self.label_18.setText(_translate("MainWindow", "**Kidnapped:**"))
        self.label_battery.setText(_translate("MainWindow", "TextLabel"))
        self.label_docked.setText(_translate("MainWindow", "TextLabel"))
        self.label_16.setText(_translate("MainWindow", "**Battery:**"))
        self.label_17.setText(_translate("MainWindow", "**Docked:**"))
        self.label_kidnapped.setText(_translate("MainWindow", "TextLabel"))
        self.label_9.setText(_translate("MainWindow", "## Logging"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "Monitoring"))
        self.label_19.setText(_translate("MainWindow", "# Mapping"))
        self.label_20.setText(_translate("MainWindow", "## Operating"))
        self.label_21.setText(_translate("MainWindow", "Starting and stopping the mapping process. When stopped, the map is saved on the Sopias4 Map-Server with the confiugation settings below"))
        self.pushButton_start_mapping.setText(_translate("MainWindow", "Start Mapping"))
        self.pushButton_stop_mapping.setText(_translate("MainWindow", "Stop Mapping and save map"))
        self.label_22.setText(_translate("MainWindow", "## Configuration for saving map"))
        self.label_23.setText(_translate("MainWindow", "Configure the parameters with which the map is saved on the Sopias4 Map-Server. The default configuration is the map which is automatically loaded when launchong Sopias4 Map-Server without passing parameters"))
        self.label_24.setText(_translate("MainWindow", "**Map name:**"))
        self.label_27.setText(_translate("MainWindow", "**Map topic:**"))
        self.label_26.setText(_translate("MainWindow", "**Image format:**"))
        self.label_25.setText(_translate("MainWindow", "**Map mode:**"))
        self.label_28.setText(_translate("MainWindow", "**Threshold occupied:**"))
        self.label_29.setText(_translate("MainWindow", "**Free threshold:**"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Mapping"))
        self.label_30.setText(_translate("MainWindow", "# Manual operation and advanced features"))
        self.label_32.setText(_translate("MainWindow", "## Driving"))
        self.label_33.setText(_translate("MainWindow", "Manual driving the robot. Note that there are no safety features in place to prevent you from crashing"))
        self.pushButton_right.setText(_translate("MainWindow", "Right"))
        self.pushButton_left.setText(_translate("MainWindow", "Left"))
        self.pushButton_forward.setText(_translate("MainWindow", "Forward"))
        self.pushButton_back.setText(_translate("MainWindow", "Backwards"))
        self.label_31.setText(_translate("MainWindow", "**Velocity (%)**"))
        self.label_34.setText(_translate("MainWindow", "## Docking"))
        self.label_35.setText(_translate("MainWindow", "Perform docking operations. Make sure Turtlebot is docked or in reasonable range to dock"))
        self.pushButton_dock.setText(_translate("MainWindow", "Dock"))
        self.pushButton_undock.setText(_translate("MainWindow", "Undock"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "Advanced/Manual operation"))
