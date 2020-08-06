import rospy
from std_msgs.msg import Float32MultiArray
from pyproj import Proj, transform
# UTM-K
proj_UTMK = Proj(init='epsg:5178') # UTM-K(Bassel)

# WGS1984
proj_WGS84 = Proj(init='epsg:4326') # Wgs84
ay = [35.68318847, 35.68327882, 35.68348881, 35.6837128417, 35.6839673467, 35.68445858, 35.6847729783]
ax = [128.464377685, 128.464517788, 128.464818563, 128.465102358, 128.465387107, 128.465836422, 128.466069103]
n = 5
rospy.init_node('sample_destination')
des_pub = rospy.Publisher("/destination_info", Float32MultiArray,queue_size = 1)

rate = rospy.Rate(10)

#x2, y2 = transform(proj_WGS84, proj_UTMK, x1, y1)



while not rospy.is_shutdown():
    msg = Float32MultiArray()
    '''
    for i in range(len(ax)):
        x2, y2 = transform(proj_WGS84, proj_UTMK, ax[i], ay[i]);
	    print(x2, y2)
        msg.data.append(x2)
        msg.data.append(y2)
    '''
    msg.data.append(1)
    msg.data.append(2)
    msg.data.append(3)
    msg.data.append(4)
    des_pub.publish(msg)
    rate.sleep()
