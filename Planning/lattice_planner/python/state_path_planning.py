"""
State lattice planner with model predictive trajectory generator
author: Atsushi Sakai (@Atsushi_twi)
- lookuptable.csv is generated with this script: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ModelPredictiveTrajectoryGenerator/lookuptable_generator.py
Ref:
- State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.187.8210&rep=rep1&type=pdf
"""
import sys
import os
from matplotlib import pyplot as plt
import numpy as np
import math
import pandas as pd
import cv2
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../ModelPredictiveTrajectoryGenerator/")
import scipy.interpolate

try:
    import model_predictive_trajectory_generator as planner
    import motion_model
except:
    raise

table_path = os.path.dirname(os.path.abspath(__file__)) + "/lookuptable.csv"

show_animation = True
t1=time.time()

map = np.zeros((500, 500))


###sample_states
sample_lidar = [[100,100,200,200], [-20, 30,-20,30 ]]

sample_init = (0, 0)

sample_des = (500,-25)

sample_line1 = np.array([(0,250,800),(50,51,50)])

sample_line2 = np.array([ (-100, 150,500),(-50,-53,-49)])

def line_on_map(line, map,type=0):
    f = scipy.interpolate.interp1d(line[0],line[1], kind='quadratic')
    for x in range(max(line[0][0],0),min(line[0][2],500)):
        a=f(x)
        y = 250-int(a)
        map[y][x]=1

def lidar_on_map(lidar, map, type=0):
    for i in range(min(lidar[0]),max(lidar[0])+1):
        for j in range(min(lidar[1]),max(lidar[1])+1):
                y=250-j
                x=i
                map[y][x]=1




line_on_map(sample_line1,map)
line_on_map(sample_line2,map)
lidar_on_map(sample_lidar, map)



def search_nearest_one_from_lookuptable(tx, ty, tyaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):

        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    return lookup_table[minid]


def get_lookup_table():
    data = pd.read_csv(table_path)

    return np.array(data)


def generate_path(target_states, k0):
    # x, y, yaw, s, km, kf
    lookup_table = get_lookup_table()
    result = []
    #tar
    for state in target_states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookup_table)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).reshape(3, 1)

        x, y, yaw, p, cost = planner.optimize_trajectory(target, k0, init_p, map)

        if x is not None:
            print("find good path")
            result.append(
                [x, y, yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish path generation")
    return result


def calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy):
    xc = math.cos(l_heading) * d + math.sin(l_heading) * l_center
    yc = math.sin(l_heading) * d + math.cos(l_heading) * l_center

    states = []
    for i in range(nxy):
        delta = -0.5 * (l_width - v_width) + \
            (l_width - v_width) * i / (nxy - 1)
        xf = xc - delta * math.sin(l_heading)
        yf = yc + delta * math.cos(l_heading)
        yawf = l_heading
        states.append([xf, yf, yawf])

    return states


def lane_state_sampling_test1():
    k0 = 0.0

    l_center = 0.0
    l_heading = np.deg2rad(0.0)
    l_width = 10.0
    v_width = 1.16
    d = 10
    nxy = 10
    states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy)
    result = generate_path(states, k0)
    mind=float("inf")
    t2=time.time()
    for table in result:
        d = math.sqrt((sample_des[0] - table[0][-1])**2+(sample_des[1] - table[1][-1])**2)
        if d<mind:
            mind= d
            x = table[0]
            y = table[1]

    print(time.time()-t1)
    if show_animation:
        plt.plot(x, y, "-b")
        plt.grid(True)
        plt.axis("equal")
        plt.show()


def main():
    #uniform_terminal_state_sampling_test1()
    t1=time.time()
    lane_state_sampling_test1()



if __name__ == '__main__':
    main()
