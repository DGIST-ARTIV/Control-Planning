import sys
import tty
import termios
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import threading
import os
import csv



global path

path=[]
class Check:
    def __init__(self):
        self.lst_tot = []
        self.lst_now =[0,0]
        self.lst_csv_1 = []
        self.lst_csv_2 = []
        self.init_x = 0
        self.init_y = 0
        self.cnt=0
        self.cnt2=0
        self.ma = MarkerArray()
        self.id = 0
        self.m = Marker()
        self.pub = rospy.Publisher("check_path", MarkerArray, queue_size = 1)
        Th1 = threading.Thread(target=self.subscribe)
        Th1.daemon = True
        Th2 = threading.Thread(target=self.on_rviz)
        Th2.daemon = True
        Th1.start()
        Th2.start()

    def on_rviz(self):
        last_location = self.lst_now
        cnt = 0
        while not rospy.is_shutdown():
            while (last_location[0]-self.lst_now[0])**2 +(last_location[1]-self.lst_now[1])**2 < 1.75:
                cnt=1
            last_location = self.lst_now
            self.m.header.frame_id = "map"
            self.m.id = self.id
            self.m.type = Marker.SPHERE
            self.m.action = Marker.ADD
            self.m.scale.x = 1
            self.m.scale.y = 1
            self.m.scale.z = 1
            self.m.color.a = 1.0
            self.m.color.r = 1.0
            self.m.color.g = 1.0
            self.m.color.b = 0.0
            self.m.pose.orientation.w = 1.0
            self.m.pose.position.x = self.lst_now[0] - self.init_x
            self.m.pose.position.y = self.lst_now[1] - self.init_y
            self.m.pose.position.z = 0
            self.ma.markers.append(self.m)
            self.id += 1
            self.pub.publish(self.ma)
            self.lst_csv_1.append([self.lst_now[0], self.lst_now[1]])
            self.cnt2+=1
        print("stop record")
    def subscribe(self):
        self.utm_subscriber = rospy.Subscriber("utm_fix", PoseStamped, self.utm_callback)
        rospy.spin()
    def utm_callback(self, data):
        if self.cnt == 0:
            self.init_x = data.pose.position.x
            self.init_y = data.pose.position.y
        self.lst_now = [data.pose.position.x, data.pose.position.y]
        self.cnt+=1
    def update(self):
        self.lst_tot.append(self.lst_now[0])
        self.lst_tot.append(self.lst_now[1])
        self.lst_csv_2.append([self.lst_now[0], self.lst_now[1]])
        print("update : {}, {}".format(self.lst_now[0], self.lst_now[1]))
    def print_list(self):
        print(self.lst_tot)
    def print_sep(self):
        x = []
        y = []
        print("length : {}".format(len(self.lst_tot)/2))
        for i in range(len(self.lst_tot)/2):
            x.append(self.lst_tot[i*2])
            y.append(self.lst_tot[i*2+1])
        print"x :", x
        print"y :", y
def getkey():
    fd = sys.stdin.fileno()
    original_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
    return ch

rospy.init_node('check_path')
check = Check()

while not rospy.is_shutdown():
    key = getkey()
    if key == 'k':
        check.update()
    if key == 'a':
        check.print_list()
        check.print_sep()
        break

rospy.signal_shutdown("node is shutdown")
file_directory = os.getcwd()
#global name
name1 =str(raw_input("Save 1m point: "))
name2 =raw_input("Save costom point: ")
name1 =name1+".csv"
name2 = name2 + ".csv"
with open(name1,'w') as f:
    print("start saving")
    print "points are", check.cnt2
    wr = csv.writer(f, delimiter = ',')
    for rows in check.lst_csv_1:
        wr.writerow(rows)
with open(name2,'w') as f:
    wr = csv.writer(f, delimiter = ',')
    for rows in check.lst_csv_2:
        wr.writerow(rows)
