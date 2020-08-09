import rospy
import threading
import math
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

global_node = [] #input global node
try:
     global node_arr = np.array(global_node).reshape(2,len(global_node)/2)
except:
    print("check global node")
global PI = math.pi
class init_navi:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        Th1 = threading.Thread(target=self.subscribe)
        Th1.daemon = True
        Th1.start()
    def subscribe(self):
        self.utm_subscriber = rospy.Subscriber("utm_fix", Float64MultiArray, self.utm_callback)
        self.gpsy_subscriber = rospy.Subscriber("gps_yaw",Float64,self.gps_callback)
        rospy.spin()
    def utm_callback(self, data):
        self.x = data.data[0]
        self.y = data.data[1]
    def gps_callback(self, data):
        self.yaw = data.data
    def pub(self):
        navi = Navi()
        while not rospy.is_shutdown():
            navi.navigation(self.x, self.y)

class Navi:
    def __init__(self,yaw):
        self.converted_node = np.array([[math.cos(-yaw), -math.sin(-yaw)], [math.sin(-yaw), math.cos(-yaw)]]) @ node_arr
        self.pub_global_path = rospy.Publisher("check_path", MarkerArray, queue_size = 1)
        self.points = Float64MultiArray()
        self.sub_point_p = []
        self.sub_point_n = []
    def navigation(self, x,y):
        cntn1 = 0
        cntn2 = 0
        cntp1 = 0
        cntp2 = 0
        for point in self.converted_node:
            dx = (point[0] - x)
            dy = (point[1] - y)
            d = math.sqrt(dx**2+dy**2)
            if d>1 and d<10:
                if cntn1>0 and cntp1>0:
                    break
                if dx>1 and cntp1 == 0:
                    self.points.append(point[0])
                    self.points.append(point[1])
                    cntp1+=1
                if dx<-1 and cntn1==0:
                    self.points.append(point[0])
                    self.points.append(point[1])
                    cntn1+=1
                cnt+=1
            elif d>=10 and d<20:
                if cntn2>0 and cntp2>0:
                    continue
                if dx>1 and cntp2 == 0:
                    self.sub_point_p.append(point[0])
                    self.sub_point_p.append(point[1])
                    cntp2+=1
                if dx<-1 and cntn2==0:
                    self.sub_point_n.append(point[0])
                    self.sub_point_n.append(point[1])
                    cntn2+=1
            if cntp1 == 0:
                if cntp2>0:
                    self.points.append(self.sub_point_p[len(self.sub_point_p)/2])
                    self.points.append(self.sub_point_p[len(self.sub_point_p)/2+1])
                else:
                    checkp = 1
            if cntn1 == 0:
                if cntp2>0:
                    self.points.append(self.sub_point_n[len(self.sub_point_n)/2])
                    self.points.append(self.sub_point_n[len(self.sub_point_n)/2+1])
                else:
                    checkn = 1

            if len(self.points)<4:
                print("# WARNING: not enough global path")
                return
            else:
                pub_global_path.publish(self.points)
