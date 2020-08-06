import sys
import tty
import termios
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import threading
import os

class Check:
    def __init__(self):
        self.lst_tot = []
        self.lst_now =[0,0]
        self.init_x = 0
        self.init_y = 0
        self.cnt=0
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
        while not rospy.is_shutdown():
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
            rospy.sleep(1)

    def subscribe(self):
        self.utm_subscriber = rospy.Subscriber("utm_fix", Float64MultiArray, self.utm_callback)
        rospy.spin()
    def utm_callback(self, data):
        if self.cnt == 0:
            self.init_x = data.data[0]
            self.init_y = data.data[1]
        self.lst_now = data.data
        self.cnt+=1
    def update(self):
        self.lst_tot.append(self.lst_now[0])
        self.lst_tot.append(self.lst_now[1])
        print("update :", self.lst_now)
    def print_list(self):
        print(self.lst_tot)
    def print_sep(self):
        x = []
        y = []
        print("length :",len(self.lst_tot))
        for i in range(len(self.lst_tot)/2):
            x.append(self.lst_tot[i*2])
            y.append(self.lst_tot[i*2+1])
        print("x :", x)
        print("y :", y)

rospy.init_node('check_path')
check = Check()
def getkey():
    fd = sys.stdin.fileno()
    original_attributes = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
    return ch

while not rospy.is_shutdown():
    key = getkey()
    if key == 'k':
        check.update()
    if key == 'a':
        check.print_list()
        check.print_sep()
        break
