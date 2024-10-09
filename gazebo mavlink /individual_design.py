# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'individual_designer.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(400, 300)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Dialog)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.individual_arm_button_2 = QtWidgets.QPushButton(Dialog)
        self.individual_arm_button_2.setObjectName("individual_arm_button_2")
        self.verticalLayout_4.addWidget(self.individual_arm_button_2, 0, QtCore.Qt.AlignHCenter)
        self.individual_force_arm_button_2 = QtWidgets.QPushButton(Dialog)
        self.individual_force_arm_button_2.setObjectName("individual_force_arm_button_2")
        self.verticalLayout_4.addWidget(self.individual_force_arm_button_2, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout.addLayout(self.verticalLayout_4)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.individual_disarm_button = QtWidgets.QPushButton(Dialog)
        self.individual_disarm_button.setObjectName("individual_disarm_button")
        self.verticalLayout_5.addWidget(self.individual_disarm_button, 0, QtCore.Qt.AlignHCenter)
        self.individual_force_disarm_button = QtWidgets.QPushButton(Dialog)
        self.individual_force_disarm_button.setObjectName("individual_force_disarm_button")
        self.verticalLayout_5.addWidget(self.individual_force_disarm_button, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout.addLayout(self.verticalLayout_5)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.individual_takeoff_altitude_lineEdit = QtWidgets.QLineEdit(Dialog)
        self.individual_takeoff_altitude_lineEdit.setObjectName("individual_takeoff_altitude_lineEdit")
        self.verticalLayout_6.addWidget(self.individual_takeoff_altitude_lineEdit, 0, QtCore.Qt.AlignHCenter)
        self.individual_takeoff_button = QtWidgets.QPushButton(Dialog)
        self.individual_takeoff_button.setObjectName("individual_takeoff_button")
        self.verticalLayout_6.addWidget(self.individual_takeoff_button, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout.addLayout(self.verticalLayout_6)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.individual_land_button = QtWidgets.QPushButton(Dialog)
        self.individual_land_button.setObjectName("individual_land_button")
        self.verticalLayout_8.addWidget(self.individual_land_button, 0, QtCore.Qt.AlignHCenter)
        self.individual_emergency_button = QtWidgets.QPushButton(Dialog)
        self.individual_emergency_button.setObjectName("individual_emergency_button")
        self.verticalLayout_8.addWidget(self.individual_emergency_button, 0, QtCore.Qt.AlignHCenter)
        self.horizontalLayout.addLayout(self.verticalLayout_8)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.individual_move_position_x_lineEdit = QtWidgets.QLineEdit(Dialog)
        self.individual_move_position_x_lineEdit.setObjectName("individual_move_position_x_lineEdit")
        self.horizontalLayout_2.addWidget(self.individual_move_position_x_lineEdit, 0, QtCore.Qt.AlignHCenter)
        self.individual_move_position_y_lineEdit = QtWidgets.QLineEdit(Dialog)
        self.individual_move_position_y_lineEdit.setObjectName("individual_move_position_y_lineEdit")
        self.horizontalLayout_2.addWidget(self.individual_move_position_y_lineEdit, 0, QtCore.Qt.AlignHCenter)
        self.individual_move_position_z_lineEdit = QtWidgets.QLineEdit(Dialog)
        self.individual_move_position_z_lineEdit.setObjectName("individual_move_position_z_lineEdit")
        self.horizontalLayout_2.addWidget(self.individual_move_position_z_lineEdit, 0, QtCore.Qt.AlignRight)
        self.verticalLayout_7.addLayout(self.horizontalLayout_2)
        self.individual_move_button_x = QtWidgets.QPushButton(Dialog)
        self.individual_move_button_x.setObjectName("individual_move_button_x")
        self.verticalLayout_7.addWidget(self.individual_move_button_x, 0, QtCore.Qt.AlignHCenter)
        self.verticalLayout_2.addLayout(self.verticalLayout_7)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.individual_arm_button_2.setText(_translate("Dialog", "Arm"))
        self.individual_force_arm_button_2.setText(_translate("Dialog", "Force Arm"))
        self.individual_disarm_button.setText(_translate("Dialog", "Disarm"))
        self.individual_force_disarm_button.setText(_translate("Dialog", "Force Disarm"))
        self.individual_takeoff_altitude_lineEdit.setPlaceholderText(_translate("Dialog", "Altitude"))
        self.individual_takeoff_button.setText(_translate("Dialog", "Takeoff"))
        self.individual_land_button.setText(_translate("Dialog", "Land"))
        self.individual_emergency_button.setText(_translate("Dialog", "Emergency"))
        self.individual_move_position_x_lineEdit.setPlaceholderText(_translate("Dialog", "X"))
        self.individual_move_position_y_lineEdit.setPlaceholderText(_translate("Dialog", "Y"))
        self.individual_move_position_z_lineEdit.setPlaceholderText(_translate("Dialog", "Z"))
        self.individual_move_button_x.setText(_translate("Dialog", "Move"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
