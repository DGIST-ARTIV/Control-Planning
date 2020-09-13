"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import sys
sys.path.append("../../PathPlanning/CubicSpline/")

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
k = 0.03  # control gain
k_e = 1 # control gain(theta_e)
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.7  # [m] Wheel base of vehicle
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
utm_x = 450570
utm_y = 3951650

global_path_x=[0]
global_path_y=[0]

# pid 관련 초기값
cur_time = time.time()
prev_time = 0
prev_error = 0
del_time = 0
error_p = 0
error_i = 0
error_d = 0
pid_out = 0
header_time = 0
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
updateRate = 15
updateIdx = 16


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
        self.x = x
        self.y = y
        self.yaw = gpsyaw*(math.pi/180)
        self.yaw = normalize_angle(self.yaw)
        self.v = gpsvel
        self.target_idx = 0

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
        print("self.yaw: ", self.yaw)
        print("self.v: ", self.v)
        self.yaw = gpsyaw*(math.pi/180)
        self.yaw = normalize_angle(self.yaw)
        self.v = gpsvel


cruise_speed = 3

'''
cruise_speed = 5
def deacceleration(di):
    global cruise_speed
    #global di
    cruise_speed = 5
    deaccel_delta = float(di * 180/(math.pi))
    abs_deaccel_delta = abs(deaccel_delta)
    print("abs_deaccel_delta: ", abs_deaccel_delta)
    if abs_deaccel_delta >= 0.0001:
        # deacceleration
        print("velocity: ", velocity)
        print("원래 cruise_speed: ", cruise_speed)
        cruise_speed = cruise_speed -(0.002*velocity*abs_deaccel_delta)
        print("cruise_speed: ", cruise_speed)
        return cruise_speed


def PID():
    global prev_time
    global error_p
    global error_i
    global error_d
    global prev_error
    global pid_out

    cur_time = time.time()
    del_time = cur_time - prev_time;

    k_p = 10
    k_i = 3
    k_d = 2
    windup_guard = 40.0

    error_p = cruise_speed - gpsvel
    error_i += error_p * (del_time)

    # Anti wind-up
    if (error_i < -windup_guard):
        error_i = -windup_guard
    elif (error_i > windup_guard):
        error_i = windup_guard

    error_d = (error_p - prev_error)/del_time
    print("error_p: ", error_p)
    print("error_i: ", error_i)
    print("error_d: ", error_d)

    pid_out = k_p*error_p + k_i*error_i + k_d*error_d
    print("pid_out: ", pid_out)

    prev_error = error_p # Feedback current error
    prev_time = cur_time # Feedback current time

    # accel_max - accel_dead_zone = 200 - 0 = 200
    # 2200/10 = 220, 220 + 1 = 221
    if pid_out > 0:
        for i in range(2001):
           if i <= 0.5*pid_out < i+1:
                gaspedal = 2*i
                if gaspedal >= 200:
                    gaspedal = 200
                    return int(gaspedal), 0
                else:
                    return int(gaspedal), 0

    # brake_max - brake_dead_zone = 200 - 0 = 200
    # 23500/10 = 2350, 2350 + 1 = 2351
    elif pid_out < 0:
        for i in range(2001):
            if i <= 0.5*abs(pid_out) < i+1:
                return 0, 2*i

    return 0, 0
'''

def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """

    # calc_target_index 함수는 target_idx, error_front_axle 두 개의 값을 return하는 함수
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    # theta e가 통상적으로 사용되는 phi(t)
    # normalize angle 함수는 어떠한 각도를 -pi에서 pi까지의 각도로 정규화 해주는 함수
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    print("cyaw[current_target_idx]: ", cyaw[current_target_idx])
    print("state.yaw: ", state.yaw)
    # theta_d corrects the cross track error
    # np.arctan2(array1, array2) = arctan(array1/array2)
    # error_front_axle  = e(t): e(t)는 cross track error
    # state.v 는 vehicle longitudinal speed
    theta_d = np.arctan2(k * error_front_axle, state.v)
    print("state.v: ", state.v)
    # Steering control
    # delta는 실제로 차가 꺾어야 하는 핸들의 각도를 의미함.
    delta = k_e * theta_e + theta_d

    print("error_front_axle: ", error_front_axle)
    print("theta_e: ", theta_e)
    print("theta_d: ", theta_d)
    print("delta: ", delta)

    return delta, current_target_idx


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


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    # 차량 앞바퀴의 x좌표, y좌표
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    # np.hypot(array1, array2): 요소별 유클리드 거리 계산
    d = np.hypot(dx, dy)
    # np.argmin(d): 배열의 최솟값의 index
    target_idx = np.argmin(d)
    # print(target_idx)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def execute(node_):
    global steerPub_
    global global_path_x
    global global_path_y
    gpsyawSub = node_.create_subscription(Float64, "/gps_yaw", gpsyawCallback, 0)
    gpsvelSub = node_.create_subscription(Float64, "/gps_spd", gpsvelCallback, 0)
    utmSub = node_.create_subscription(PoseStamped, "/utm_fix", utmCallback, 0)
    # erpSub = node_.create_subscription(Float32MultiArray, "/ERP42_info", erpinfoCallback, 0)
    ioniqSub = node_.create_subscription(Float32MultiArray, "/Ioniq_info", ioniqinfoCallback, 0)
    #JointSub = node_.create_subscription(JointState, "/Joint_state", jointCallback, 0)
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

    global accelPub_
    global brakePub_
    global steerPub_
    global history_x, history_y


    state = State(x=global_path_x[0], y=global_path_y[0])

    while True:
        if len(cx) >0 and len(cy) > 0 and len(global_path_x)>0 and len(global_path_y)>0 :

            #state = State(x=global_path_x[0], y=global_path_y[0], yaw=np.radians(20.0), v=0.0)

            accel_ = Int16()
            brake_ = Int16()

            # accel_.data, brake_.data = PID()

            accel_.data = 2000
            accelPub_.publish(accel_)
            brakePub_.publish(brake_)

            target_idx, error_front_axle = calc_target_index(state, cx, cy)

            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            msg = Int16()
            state.update(di)

            # 추후 주석 처리하기~
            if math.isnan(di):
                di = 0.0
            # print("delta: ", di)
            # print("error_front_axle: ", error_front_axle)
            print("target_idx: ", target_idx)

            # down_delta = float(di * 180/(math.pi)/1800)
            # print("down_delta: ", down_delta)

            # 71 = 2000/28.1690
            raw_steer = int(-di*(180/(math.pi))*(525/35.75))
            print("raw_steer: ", raw_steer)

            if -530 < raw_steer < 530:
                msg.data = raw_steer
            elif raw_steer <= -530:
                msg.data = -530
            elif raw_steer >= 530:
                msg.data = 530

            print("Steer: ", msg.data)

            steerPub_.publish(msg)

            # deaccel_speed = deacceleration(di)
            # print("deaccel_speed: ", deaccel_speed)

            print("Accel: ", accel_.data)
            print("Brake: ", brake_.data)

            print("utm_x: ", utm_x)
            print("utm_y: ", utm_y)
            print("-----------------------------------------------")
            #print("limitedSteer: ", msg.data)
            #print("pidout: ", pid_out)

            time += dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
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
                try:
                    plt.plot(cx[target_idx], cy[target_idx], "xk", label="target")
                except:
                    print("plot error")
                #plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(gpsvel)[:4])
                plt.axis([min(global_path_x)-5, max(global_path_x)+5, min(global_path_y)-5, max(global_path_y)+5])
                # plt.axis("equal")
                plt.pause(0.001)
                majorLocator   = MultipleLocator(20)
                majorFormatter = FormatStrFormatter('%d')
                minorLocator   = MultipleLocator(5)
                

            '''
            while last_idx - 10 <= target_idx <= last_idx:
                accel_ = Int16()
                brake_ = Int16()
                accel_.data = 0
                brake_.data = int(15*(8-gpsvel))
                accelPub_.publish(accel_)
                brakePub_.publish(brake_)
                print("Accel: ", accel_.data)
                print("Brake: ", brake_.data)
                if last_idx -5 < target_idx:
                    break
            '''
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

def ioniqinfoCallback(msg):
    global ioniqsteer
    global gpsvel
    gpsvel = msg.data[0]
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
'''

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
    global accelPub_
    global brakePub_
    global steerPub_

    global gpsvel
    global state
    global header_time

    global history_x_mid
    global history_y_mid
    global updateIdx, updateRate

    ### Compare Header
    # 0821 Add if path is same as prev one, pass
    if updateIdx > updateRate:
        global_path_x = msg.position
        global_path_y = msg.velocity
        updateIdx = 0

    # print("global_path_x: ", global_path_x)

    if header_time == msg.header.stamp:
        print("************************SAME PATH ")
        #return 0
    if len(global_path_x) < 30:

        print("ERROR Rogue Path data", len(global_path_x))
        return 0
    dist = np.sqrt(np.square(history_x_mid - global_path_x[4]) + np.square(history_y_mid -global_path_y[4] ))
    print("************************* DIST", dist)

    history_x_mid = global_path_x[4]
    history_y_mid = global_path_y[4]
    header_time = msg.header.stamp

    #print("global_path_x: ", global_path_x)
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        global_path_x, global_path_y, ds=0.1)
    #print('fuck')
    # Initial state
    state = State(x=global_path_x[0], y=global_path_y[0])
    # state = State(x=450570, y=3951646, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    #target_idx, _ = calc_target_index(state, cx, cy)
    updateIdx = 1 + updateIdx


def main():
    global accelPub_
    global brakePub_
    global steerPub_

    rclpy.init()
    node_ = rclpy.create_node('stanley_method')
    accelPub_ = node_.create_publisher(Int16, '/dbw_cmd/Accel', qos_profile_default)
    brakePub_ = node_.create_publisher(Int16, '/dbw_cmd/Brake', qos_profile_default)
    steerPub_ = node_.create_publisher(Int16, '/dbw_cmd/Steer', qos_profile_default)
    thread_gps = threading.Thread(target=execute, args=(node_,))
    thread_gps2 = threading.Thread(target=execute2)
    thread_gps.start()
    thread_gps2.start()


    thread_gps.join()
    thread_gps2.join()

    '''
    f = open('normal_points.csv', 'r', encoding='utf-8')
    rdr = csv.reader(f)

    ax = []
    ay = []

    for line in rdr:
        ax.append(float(line[0]))
        ay.append(float(line[1]))

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        global_path_x, global_path_y, ds=1)
    '''


if __name__ == '__main__':

    main()
