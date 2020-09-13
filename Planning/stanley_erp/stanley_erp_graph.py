import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import sys
sys.path.append("../../PathPlanning/fSpline/")

import rclpy
from rclpy.qos import qos_profile_default
import os
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import NavSatFix
import math
import threading
import time
import termios
import sys
import tty
import select
import math
import csv

try:
    import cubic_spline_planner
except:
    raise


i=0
k = 0.4  # control gain
k_s = 0.03
k_e = 1 # control gain(theta_e)
k_total = 0.35
dt = 0.1 # [s] time difference
L = 1.04  # [m] Wheel base of vehicle
# np.radians(20.0) = 0.34906585
max_steer = np.radians(20.0)  # [rad] max steering angle

# 위도, 경도, gpsyaw 초기값 설정
gpsyaw = 0
gpsvel = 0
ioniqsteer = 0
erpsteer = 0
velocity = 0

# utm_x = 0
# utm_y = 0
utm_x = float('NaN')
utm_y = float('NaN')

global_path_x=[0]
global_path_y=[0]

show_animation = True


cx = []
cy = []
ck = []
cyaw =[]
s =[]

last_idx =0
x =0
y =0
yaw =0
v =0
t =0

history_x = []
history_y = []
history_x_mid = 0
history_y_mid = 0

# map update rate, idx is a cycle for path cb 
updateRate = 10
updateIdx = 11


class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0, y=0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = utm_x
        self.y = utm_y
        self.yaw = gpsyaw*(math.pi/180)
        self.yaw = normalize_angle(self.yaw)
        self.v = gpsvel

    def update(self, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """

        # 차가 꺾을 수 있는 각도를 제한
        delta = np.clip(delta, -max_steer, max_steer)
        # delta = theta_e + theta_d

        self.x = utm_x
        self.y = utm_y
        # self.yaw의 단위는 radian이고, gpsyaw의 단위는 degree이므로, gpsyaw를 radian으로
        # 바꿔주기 위해서는 pi/180을 곱해주어야 함.
        # print("self.yaw: ", self.yaw)
        # print("self.v: ", self.v)
        self.yaw = gpsyaw*(math.pi/180)
        self.yaw = normalize_angle(self.yaw)
        self.v = gpsvel


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def execute(node_):
    global steerPub_
    global global_path_x
    global global_path_y
    gpsyawSub = node_.create_subscription(Float64, "/gps_yaw", gpsyawCallback, 0)
    gpsvelSub = node_.create_subscription(Float64, "/gps_spd", gpsvelCallback, 0)
    utmSub = node_.create_subscription(PoseStamped, "/utm_fix", utmCallback, 0)
    erpSub = node_.create_subscription(Float32MultiArray, "/ERP42_info", erpinfoCallback, 0)
    # ioniqSub = node_.create_subscription(Float32MultiArray, "/Ioniq_info", ioniqinfoCallback, 0)
    JointSub = node_.create_subscription(JointState, "/Joint_state", jointCallback, 0)
    pathSub = node_.create_subscription(JointState, "/hdmap/path", pathCallback, 0)
    rclpy.spin(node_)

def execute2():
    global cx
    global cy
    global cyaw
    global ck
    global s
    global global_path_x
    global global_path_y
    global state
    global last_idx
    global x
    global y
    global yaw
    global v
    global t
    global time

    global history_x, history_y


    state = State(x=global_path_x[0], y=global_path_y[0])

    while True:

        if len(cx) >0 and len(cy) > 0 and len(global_path_x)>0 and len(global_path_y)>0 :

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            # t.append(time)
            history_x.append(state.x)
            history_y.append(state.y)

            
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])

                # course가 local path의 역할을 함.
                # course를 그릴 때 cx, cy가 필요함.

                plt.plot(cx, cy, ".y", label="course")
                # plt.scatter(global_path_x, global_path_y)
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(history_x, history_y, "-c", label="history")

                plt.grid(True)
                plt.title("Speed[km/h]:" + str(gpsvel)[:4])
                plt.axis([min(global_path_x)-5, max(global_path_x)+5, min(global_path_y)-5, max(global_path_y)+5])
                # plt.axis("equal")
                plt.pause(0.0005)
                majorLocator   = MultipleLocator(20)
                majorFormatter = FormatStrFormatter('%d')
                minorLocator   = MultipleLocator(5)
                
    pass


def gpsyawCallback(msg):
    global gpsyaw
    gpsyaw = msg.data
    pass

def gpsvelCallback(msg):
    global gpsvel
    gpsvel = msg.data
    pass

def utmCallback(msg):
    global utm_x
    global utm_y
    utm_x = msg.pose.position.x
    utm_y = msg.pose.position.y
    pass

'''
def ioniqinfoCallback(msg):
    global ioniqsteer
    ioniqsteer = msg.data[2]
    pass
'''

def erpinfoCallback(msg):
    global erpsteer
    erpsteer = msg.data[4]
    pass

def jointCallback(msg):
    global velocity
    velocity = msg.velocity[0]
    pass


def pathCallback(msg):
    global cx
    global cy
    global cyaw
    global ck
    global s
    global global_path_x
    global global_path_y
    global state
    global last_idx
    global x
    global y
    global yaw
    global v
    global t
    global time

    global gpsvel
    global state
    global header_time

    global history_x_mid
    global history_y_mid
    global updateIdx, updateRate

    ### Compare Header
    # 0821 Add if path is same as prev one, pass

    # start = time.time()

    if updateIdx > updateRate:
        global_path_x = msg.position
        global_path_y = msg.velocity
        updateIdx = 0

    # print("global_path_x: ", global_path_x)

    '''
    if header_time == msg.header.stamp:
        print("************************SAME PATH ")
        #return 0
    '''
    if len(global_path_x) < 30:

        print("ERROR Rogue Path data", len(global_path_x))
        return 0
    dist = np.sqrt(np.square(history_x_mid - global_path_x[4]) + np.square(history_y_mid -global_path_y[4] ))
    # print("************************* DIST", dist)

    history_x_mid = global_path_x[4]
    history_y_mid = global_path_y[4]
    header_time = msg.header.stamp

    #print("global_path_x: ", global_path_x)
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        global_path_x, global_path_y, ds=0.1)
    # Initial state
    state = State(x=global_path_x[0], y=global_path_y[0])

    last_idx = len(cx) - 1
    # time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    # target_idx, _ = calc_target_index(state, cx, cy)
    updateIdx = 1 + updateIdx

    # print("time: ", time.time() - start)

def main():
    rclpy.init()
    node_ = rclpy.create_node('stanley_method')
    thread_gps = threading.Thread(target=execute, args=(node_,))
    thread_gps2 = threading.Thread(target=execute2)
    thread_gps.start()
    thread_gps2.start()

    thread_gps.join()
    thread_gps2.join()


if __name__ == '__main__':
    main()
