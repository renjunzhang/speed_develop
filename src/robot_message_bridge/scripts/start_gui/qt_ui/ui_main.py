# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_main.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(714, 486)
        self.label_error_messages = QtWidgets.QLabel(Form)
        self.label_error_messages.setGeometry(QtCore.QRect(90, 410, 461, 31))
        self.label_error_messages.setText("")
        self.label_error_messages.setObjectName("label_error_messages")
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setGeometry(QtCore.QRect(451, 306, 32, 24))
        self.label_5.setObjectName("label_5")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(440, 216, 48, 24))
        self.label_3.setObjectName("label_3")
        self.pushButton_2 = QtWidgets.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(70, 120, 88, 32))
        self.pushButton_2.setObjectName("pushButton_2")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(451, 171, 32, 24))
        self.label_2.setObjectName("label_2")
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setGeometry(QtCore.QRect(451, 261, 32, 24))
        self.label_4.setObjectName("label_4")
        self.label_platform = QtWidgets.QLabel(Form)
        self.label_platform.setGeometry(QtCore.QRect(520, 265, 51, 19))
        self.label_platform.setStyleSheet("background-color: rgb(85, 85, 127);")
        self.label_platform.setText("")
        self.label_platform.setObjectName("label_platform")
        self.label_arm = QtWidgets.QLabel(Form)
        self.label_arm.setGeometry(QtCore.QRect(520, 220, 51, 19))
        self.label_arm.setStyleSheet("background-color: rgb(85, 85, 127);")
        self.label_arm.setText("")
        self.label_arm.setObjectName("label_arm")
        self.label_camera = QtWidgets.QLabel(Form)
        self.label_camera.setGeometry(QtCore.QRect(520, 175, 51, 19))
        self.label_camera.setStyleSheet("background-color: rgb(85, 85, 127);\n"
"border-radius:10px;")
        self.label_camera.setText("")
        self.label_camera.setObjectName("label_camera")
        self.pushButton_3 = QtWidgets.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(70, 280, 80, 32))
        self.pushButton_3.setObjectName("pushButton_3")
        self.label_radar = QtWidgets.QLabel(Form)
        self.label_radar.setGeometry(QtCore.QRect(520, 310, 51, 19))
        self.label_radar.setStyleSheet("background-color: rgb(85, 85, 127);")
        self.label_radar.setText("")
        self.label_radar.setObjectName("label_radar")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(270, 140, 101, 81))
        self.pushButton.setObjectName("pushButton")
        self.progressBar = QtWidgets.QProgressBar(Form)
        self.progressBar.setGeometry(QtCore.QRect(550, 40, 118, 23))
        self.progressBar.setMinimum(-1)
        self.progressBar.setProperty("value", -1)
        self.progressBar.setObjectName("progressBar")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(490, 40, 41, 19))
        self.label.setObjectName("label")

        self.retranslateUi(Form)
        self.pushButton.clicked['bool'].connect(Form.StartStop)
        self.pushButton_2.clicked.connect(Form.ArmRest)
        self.pushButton_3.clicked.connect(Form.Grip)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_5.setText(_translate("Form", "雷达"))
        self.label_3.setText(_translate("Form", "机械臂"))
        self.pushButton_2.setText(_translate("Form", "机械臂复位"))
        self.label_2.setText(_translate("Form", "相机"))
        self.label_4.setText(_translate("Form", "底盘"))
        self.pushButton_3.setText(_translate("Form", "开夹爪"))
        self.pushButton.setText(_translate("Form", "启动"))
        self.label.setText(_translate("Form", "电量"))
