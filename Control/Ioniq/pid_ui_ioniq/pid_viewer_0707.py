##############
# PID CONTROLLER
# GUI Version of PID Controller
# Copyright ©  2020 DGIST ARTIV All rights reserved
# Author: Seunggi Lee
# Date: 2020.07.07


import sys
import os
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QDesktopWidget
from PyQt5.QtWidgets import QLabel, QVBoxLayout
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from subprocess import Popen, PIPE
import threading
import time
import rclpy  #run this file in python3
import termios
import sys
import tty
import select
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from rclpy.qos import qos_profile_default
# Load UI file from local directory
form_class = uic.loadUiType("pid_viewer.ui")[0]


class MyWindow(QMainWindow, form_class):

    def __init__(self):
        super(QMainWindow, self).__init__()
        self.setupUi(self)

        rclpy.init()
        node = rclpy.create_node('PID_Viewer')
        self.apth = ApplyThread(node)
        self.apth.start()

        #global node
        self.pushButton.clicked.connect(self.applyBtn)
        self.pushButton_4.clicked.connect(self.startDrive)
        self.pushButton_2.clicked.connect(self.actReset)
        self.doubleSpinBox_2.editingFinished.connect(self.applyBtn)

        self.kp = 1.25
        self.ki = 0.75
        self.kd = 1
        # self.desired_speed = 600
        self.desired_speed = 0
        # self.Anti_windup_guard = 70
        self.Anti_windup_guard = 70

    def startDrive(self):
        if self.apth.pubSwitch:
            self.apth.pubSwitch = False
            self.label_17.setText("Cruise mode OFF")
            self.label_17.setStyleSheet("Color : red")
        else:
            self.apth.pubSwitch = True
            self.label_17.setText("Cruise mode ON")
            self.label_17.setStyleSheet("Color : green")

        pass



    def applyBtn(self):

        if self.desired_speed < float(self.doubleSpinBox_2.value()):
            print(self.desired_speed)
            print(float(self.doubleSpinBox_2.value()))
            if self.apth.error_i < 0: self.apth.error_i = 0

        elif self.desired_speed > float(self.doubleSpinBox_2.value()):
            print(self.desired_speed)
            print(float(self.doubleSpinBox_2.value()))
            if self.apth.error_i > 0: self.apth.error_i = 0

        self.apth.error_i = 0


        self.kp = float(self.lineEdit.text())
        self.ki = float(self.lineEdit_2.text())
        self.kd = float(self.lineEdit_3.text())
        # self.desired_speed = 600
        self.desired_speed = float(self.doubleSpinBox_2.value())
        # self.Anti_windup_guard = 70
        self.Anti_windup_guard = float(self.lineEdit_9.text())

####### 추후 주석 처리 하기!!! 매우 중요
        self.apth.pubSwitch = True
        self.label_17.setText("Cruise mode ON")
        self.label_17.setStyleSheet("Color : green")


    def actReset(self):
        self.apth.init()
        self.apth.pubSwitch = False
        self.label_17.setText("Cruise mode OFF")
        self.label_17.setStyleSheet("Color : red")
        pass


class ApplyThread(QThread):

    def init(self):
        self.prev_error = 0

        self.error_p = 0
        self.error_i = 0
        self.error_d = 0
        self.pid_out = 0

        # Act data
        self.handle_set = 0
        self.accel_data = 0
        self.brake_data = 0

        # Ros initialize
        self.current_speed = 0

        accel = Int16()
        brake = Int16()
        steer = Int16()
        for i in range(5):
            accel.data = self.accel_data
            brake.data = self.brake_data
            steer.data = self.handle_set

            self.brakePub.publish(brake)
            self.accelPub.publish(accel)
            self.steerPub.publish(steer)


        self.pubSwitch = False

    def __init__(self, node_, parent=None):
        QThread.__init__(self)
        self.node_ = node_
        self.main = parent
        self.isRun = True

        self.cur_time = time.time()
        self.prev_time = 0
        self.prev_error = 0
        self.del_time = 0
        self.error_p = 0
        self.error_i = 0
        self.error_d = 0
        self.pid_out = 0

        # Act data
        self.handle_set = 0
        self.accel_data = 0
        self.brake_data = 0

        # Ros initialize
        self.current_speed = 0
        self.pubSwitch = False

    def __del__(self):
        print("Command Job Done")
        self.wait()


    def PID(self):

        global myWindow
        # myWindow.desired_speed = Int16()
        # self.cruise_speed_Pub.publish(myWindow.desired_speed.data())
        tempSpeed = Int16()
        tempSpeed.data = int(myWindow.desired_speed)
        self.cruise_speed_Pub.publish(tempSpeed)
        self.cur_time = time.time()
        self.del_time = self.cur_time - self.prev_time;

        self.error_p = myWindow.desired_speed - self.current_speed
        self.error_i += self.error_p * (self.del_time)

        time.sleep(0.005)

    	# Anti wind-up
        if (self.error_i < - myWindow.Anti_windup_guard):
            self.error_i = - myWindow.Anti_windup_guard
        elif (self.error_i > myWindow.Anti_windup_guard):
            self.error_i = myWindow.Anti_windup_guard

        self.error_d = (self.error_p - self.prev_error)/self.del_time

        # pid_out
        self.pid_out = myWindow.kp*self.error_p + myWindow.ki*self.error_i + myWindow.kd*self.error_d

        if self.pid_out > 1e5 : self.pid_out = 1e5
        elif self.pid_out < -1e5 : self.pid_out = -1e5

        # Feedback
        self.prev_error = self.error_p # Feedback current error
        self.prev_time = self.cur_time # Feedback current time

    	# accel_max - accel_dead_zone = 3000 - 800 = 2200
    	# 2200/10 = 220, 220 + 1 = 221
        if self.pid_out > 0:
            for i in range(221):
                if i <= self.pid_out < i+1:
                    return 800 + 10*i, 0

    	# brake_max - brake_dead_zone = 27000 - 3500 = 23500
    	# 23500/10 = 2350, 2350 + 1 = 2351
        elif self.pid_out < 0:
            for i in range(2351):
                if i <= abs(self.pid_out) < i+1:
                    return 0, 3500+10*i

        return 0, 0


    def jointCallback(self, data):
        self.current_speed = data.velocity[0]
        myWindow.lcdNumber.display(self.current_speed)
        myWindow.lcdNumber_2.display(self.current_speed + myWindow.doubleSpinBox.value())
        myWindow.lineEdit_10.setText(str(self.current_speed))

        self.accel_data, self.brake_data = self.PID()

        myWindow.lineEdit_5.setText(str(round(self.error_p, 3)))
        myWindow.lineEdit_6.setText(str(round(self.error_i, 3)))
        myWindow.lineEdit_7.setText(str(round(self.error_d, 3)))
        myWindow.lineEdit_8.setText(str(round(self.pid_out, 3)))

        print("accel: ", self.accel_data)
        print("brake: ", self.brake_data)

        myWindow.lineEdit_11.setText(str(self.accel_data))
        myWindow.lineEdit_12.setText(str(self.brake_data))

        myWindow.lineEdit_14.setText(str(round(self.cur_time, 2)))
        myWindow.lineEdit_15.setText(str(round(self.del_time, 4)))

        print(self.cur_time)
        print(self.del_time)


    def intCallback(self, data):
        self.steer = data.data[3]
        #print(self.steer)

    def floatCallback(self, data):
        self.accel_data, self.brake_data = self.PID()
        self.steer_data = data.data[3]

        myWindow.lineEdit_13.setText(str(self.steer_data))
        # myWindow.lcdNumber_18.display(self.steer_data)


        if (self.pubSwitch):
            accel = Int16()
            brake = Int16()
            steer = Int16()

            accel.data = self.accel_data
            brake.data = self.brake_data
            steer.data = self.handle_set

            self.brakePub.publish(brake)
            self.accelPub.publish(accel)
            self.steerPub.publish(steer)

        '''
        myWindow.lcdNumber_16.display(self.accel_data)
        myWindow.lcdNumber_17.display(self.brake_data)
        myWindow.lcdNumber_18.display(self.steer_data)
        '''

        '''
        self.accel_data = data.data[0]
        self.brake_data = data.data[1]
        self.steer_data = data.data[3]
        myWindow.lcdNumber_16.display(self.accel_data)
        myWindow.lcdNumber_17.display(self.brake_data)
        myWindow.lcdNumber_18.display(self.steer_data)
        '''

    def run(self):
        jointSub = self.node_.create_subscription(JointState,
            "/Joint_state",
            self.jointCallback)

        intSub = self.node_.create_subscription(Int16MultiArray,
            "/Int_Info",
            self.intCallback)

        floatSub = self.node_.create_subscription(Float32MultiArray,
            "/Ioniq_info",
            self.floatCallback)

        self.accelPub = self.node_.create_publisher(Int16, '/dbw_cmd/Accel', qos_profile_default)
        self.brakePub  = self.node_.create_publisher(Int16, '/dbw_cmd/Brake', qos_profile_default)
        self.steerPub = self.node_.create_publisher(Int16, '/dbw_cmd/Steer', qos_profile_default)
        self.cruise_speed_Pub = self.node_.create_publisher(Int16, '/Cruise_speed', qos_profile_default)
        while rclpy.ok():
            rclpy.spin_once(self.node_)
            time.sleep(0.002)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    global myWindow
    myWindow = MyWindow()
    myWindow.show()
    app.exec_()
