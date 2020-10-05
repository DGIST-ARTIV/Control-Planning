import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import csv
import matplotlib
import threading
import math
rospy.init_node("carla_glodal_path")
class publisher:
    def __init__(self):
        self.x=0
        self.y=0
        self.nodes = []
        self.node_x = []
        self.node_y = []
        self.path_x = []
        self.path_y = []
        self.hz = rospy.Rate(20)
        self.get_global_path()
        self.path_pub = rospy.Publisher("/hdmap/path", JointState, queue_size = 1)
        self.utm_sub = rospy.Subscriber("utm_fix", PoseStamped, self.utm_CallBack)

        self.id = 0
        self.len = 30
        Th1 = threading.Thread(target=self.publishing)
        Th1.daemon = True
        Th1.start()
        rospy.spin()
    def utm_CallBack(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y

    def get_global_path(self):
        # name =raw_input("input carla path :")
        name = "E6_path1.csv"
        with open(name, "r") as f:
            reader = csv.reader(f, delimiter = ',')
            for rows in reader:
                x = float(rows[0])
                y = float(rows[1])
                self.nodes.append([x,y])
    def publishing(self):
        while self.x==0 or self.y==0:
            rospy.loginfo("Waiting for start")
        while not rospy.is_shutdown():
            id = 0
            msg = JointState()
            for i in self.nodes:
                #print( math.sqrt((i[0]-self.x)**2+(i[1] - self.y)**2), i, self.x, self.y)
                if math.sqrt((i[0]-self.x)**2+(i[1] - self.y)**2) < 5:
                    self.id = id
                    break
                id+=1
            else:
                rospy.loginfo("Path is gone")
                continue
            k=2
            for i in range(self.id+k, self.id + self.len+k):
                x = 0
                y = 0
                try:
                    x = self.nodes[i][0]
                    y = self.nodes[i][1]
                except:
                    rospy.loginfo("# WARNING: Path end")
                    break
                msg.position.append(x)
                msg.velocity.append(y)
            rospy.loginfo("publishing {}".format(self.id))
            rospy.sleep(0.05)
            self.path_pub.publish(msg)

if __name__ == "__main__":
    a = publisher()
