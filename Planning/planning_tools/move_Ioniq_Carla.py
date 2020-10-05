# Mediating Node for Ioniq, move_Ioniq
# Author: Juho Song (hoya4764@dgist.ac.kr)
# 2020.09.03 released
# version 1.0.3
# github :

# <new update>
# 1. every mode in this file can be used.
# 2. PID function has modified.

# structure of move_car for Ioniq: [car_type = 0.0(Ioniq), mode, speed, accel, brake, steer, gear, angular(Ioniq), status(ERP42), estop(ERP42)]

#for PID
import time

#ros2 module
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

NODENAME = "move_Ioniq"

#subscribed topics
topic_Movecar = '/move_car'
topic_Info = '/Carla_info'
topic_NavPilot = '/mission_manager/nav_pilot'

#topics to be published
topic_MovecarInfo = '/move_car_info'

rootname = '/dbw_cmd'
topic_Accel = '/Accel'
topic_Angular = '/Angular'
topic_Brake = '/Brake'
topic_Steer = '/Steer'
topic_Gear = '/Gear'

class Move_Ioniq():

    def __init__(self):
        # Initializing move_Ioniq node

        #callback data
        self.infomsg = [] # Ioniq_info
        self.cur_vel = 0.0 # Carla_info
        self.move_carmsg = [] #move_car(debug)
        self.move_car_info = "" #move_car_info
        self.prev_mode = 200.0
        self.nav_pilot_switch = 2

        # About PID
        self.cur_time = time.time()
        self.prev_time = 0
        self.del_time = 0
        self.kp = 20
        self.ki = 7
        self.kd = 5
        self.error_p = 0
        self.error_i = 0
        self.error_d = 0
        self.prev_error = 0
        self.antiwindup = 70
        self.prev_desired_vel = 0
        self.mode_data = [None] * 4

        # Initializing Ioniq dbw_cmd_node publisher
        self.accelPub = rospy.Publisher(rootname + topic_Accel, Int16, queue_size = 1)
        self.brakePub = rospy.Publisher(rootname + topic_Brake, Int16,queue_size = 1)
        self.steerPub = rospy.Publisher(rootname + topic_Steer, Int16,queue_size = 1)
        self.gearPub = rospy.Publisher(rootname + topic_Gear, Int16,queue_size = 1)
        self.angularPub = rospy.Publisher(rootname + topic_Angular, Int16,queue_size = 1)

        # Initializing dbw_ioniq_node subscriber
        self.InfoSub = rospy.Subscriber(topic_Info, Float32MultiArray, self.info_callback)

        # Initializing Navigation Pilot switch subscriber
        self.NavPilotSub = rospy.Subscriber(topic_NavPilot, Int16, self.nav_pilot_callback)

        # Initializing move_car topic subscriber
        self.move_carSub = rospy.Subscriber(topic_Movecar,Float32MultiArray, self.move_car_callback)

        # Initializing move_car_info topic publisher (depends on move_car callback)
        self.MovecarInfoPub = rospy.Publisher(topic_MovecarInfo, String, queue_size = 1)
        rospy.spin()
    def nav_pilot_callback(self, msg):
        self.nav_pilot_switch = msg.data

    def info_callback(self, msg):
        self.cur_vel = msg.data[0]

        if self.mode_data[0] == 1.0 or self.mode_data[0] == 2.0 or self.mode_data[0] ==4.0 or self.mode_data[0] ==5.0 or self.mode_data[0] ==6.0: # for PID
            self.mode_data[2], self.mode_data[3] = self.PID(self.mode_data[1])
            self.pub_accel(self.mode_data[2])
            self.pub_brake(self.mode_data[3])
            self.mode_data[1] = self.prev_desired_vel
            rospy.loginfo("cruise mode{}. {}km/h a: {}, b: {}".format(self.mode_data[0], self.mode_data[1], self.mode_data[2],self.mode_data[3]))

    # structure of move_car for Ioniq: [car_type = 0.0(Ioniq), mode, speed, accel, brake, steer, gear, angular(Ioniq), status(ERP42), estop(ERP42)]
    def move_car_callback(self, msg):
        rospy.loginfo("{}".format(msg.data))
        if self.prev_mode == 200.0 and len(msg.data) ==1 and msg.data[0] == 119.0: #case 0. when the reset code is the first published code
            self.prev_mode == 119.0
            rospy.logwarn("You published E-Stop reset code as the first topic!")

        elif self.prev_mode == 119.0 and len(msg.data) ==1 and msg.data[0] == 119.0: #case 1. reset many times repeatly
            self.prev_mode = 119.0
            rospy.logwarn("Don't reset too much!")

        elif self.prev_mode !=0.0 and len(msg.data) == 1 and msg.data[0] == 119.0: #case 2. reset after other modes except E-Stop mode
            self.prev_mode = 119.0
            rospy.logwarn("Reset after other modes except E-Stop mode.")

        elif self.prev_mode !=0.0:

            if len(msg.data) != 10 or msg.data[0] != 0.0: #wrong topics
                rospy.logwarn("You published wrong!!! {}".format(msg.data))
            else:
                mode = msg.data[1]

                if self.nav_pilot_switch == 0 and mode == 6.0: # mission manager mode
                    if self.prev_mode == 119.0:
                        self.emergency_stop_off()
                    steer = msg.data[5]
                    gear = msg.data[6]
                    angular = msg.data[7]

                    self.cruise_control(msg)
                    self.pub_gear(gear)
                    self.pub_angular(angular)
                    self.pub_steer(steer)
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug

                elif mode ==0.0: #E-STOP
                    rospy.logwarn("E-STOP Publishing Actuator with mode{}".format(mode))
                    self.emergency_stop_on()
                    self.mode_data[0] = 0.0
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug
                    self.prev_error = 0
                    self.error_i = 0
                    self.prev_time = 0
                    self.prev_desired_vel = 0

                elif mode == 1.0: #cruise control
                    if self.prev_mode == 119.0:
                        self.emergency_stop_off()
                    self.cruise_control(msg)
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug

                elif mode == 2.0: #cruise control with steering

                    if self.prev_mode == 119.0:
                        self.emergency_stop_off()
                    steer = msg.data[5]
                    angular = msg.data[7]

                    self.cruise_control(msg)
                    self.pub_angular(angular)
                    self.pub_steer(steer)
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug

                elif mode == 3.0: # mode that you can directly publish cmd value (for develper mode.)

                    if self.prev_mode == 119.0:
                        self.emergency_stop_off()
                    self.mode_data[0] = 3.0
                    accel = msg.data[3]
                    brake = msg.data[4]
                    steer = msg.data[5]
                    gear = msg.data[6]
                    angular = msg.data[7]

                    self.pub_accel(accel)
                    self.pub_brake(brake)
                    self.pub_gear(gear)
                    self.pub_angular(angular)
                    self.pub_steer(steer)
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug
                    self.prev_error = 0
                    self.error_i = 0
                    self.prev_desired_vel = 0
                    self.prev_time = 0

                elif mode == 4.0: # mode 1.0 + mode 3.0 (cruise control and direct publish except accel and brake)

                    if self.prev_mode == 119.0:
                        self.emergency_stop_off()
                    steer = msg.data[5]
                    gear = msg.data[6]
                    angular = msg.data[7]

                    self.cruise_control(msg)
                    self.pub_gear(gear)
                    self.pub_angular(angular)
                    self.pub_steer(steer)
                    self.prev_mode = msg.data[1]
                    self.move_carmsg = msg.data #debug
                    rospy.loginfo("{}".format(self.move_carmsg)) #dubug

                elif mode == 5.0: # navigation pilot mode
                    if self.nav_pilot_switch == 0: #switch off
                      rospy.logwarn("nav_pilot_switch is off!")

                    elif self.nav_pilot_switch ==1:
                        if self.prev_mode == 119.0:
                            self.emergency_stop_off()
                        steer = msg.data[5]
                        gear = msg.data[6]
                        angular = msg.data[7]

                        self.cruise_control(msg)
                        self.pub_gear(gear)
                        self.pub_angular(angular)
                        self.pub_steer(steer)
                        self.prev_mode = msg.data[1]
                        self.move_carmsg = msg.data #debug
                        rospy.loginfo("{}".format(self.move_carmsg)) #dubug

        elif self.prev_mode == 0.0:

            if len(msg.data) == 1 and msg.data[0] ==119.0: #escape code [119.0]
                self.prev_mode = 119.0
                rospy.loginfo("Escape!")
            else:
                rospy.loginfo("Stucked in E-Stop! {}".format(msg.data))

        else:
            rospy.logwarn("Not Valid Message!!! {}".format(msg.data))




    # Basic cmd publisher
    def pub(self, topic, val):
        topic_list = ['accel','brake','steer','gear','angular']

        if topic == topic_list[0]:
            val = int(val)
            if not 0 <= val <= 3000:
                raise ValueError("your val for accel, {} is out of range.".format(val))
            accel = Int16()
            accel.data = val
            self.accelPub.publish(accel)

        elif topic == topic_list[1]:
            val = int(val)
            if not 0 <= val <= 29000:
                raise ValueError("your val for brake, {} is out of range.".format(val))
            brake = Int16()
            brake.data = val
            self.brakePub.publish(brake)

        elif topic == topic_list[2]:
            val = int(val)
            if not -440 <= val <= 440:
                raise ValueError("your val for steer, {} is out of range.".format(val))
            steer = Int16()
            steer.data = val
            self.steerPub.publish(steer)

        elif topic == topic_list[3]:
            val = int(val)
            gear_dict = {0:"parking",5:"driving",6:"neutral",7:"reverse"}
            if val not in gear_dict:
                raise ValueError("your val for gear, {} is not valid.".format(val))
            gear = Int16()
            gear.data = val
            self.gearPub.publish(gear)

        elif topic == topic_list[4]:
            val = int(val)
            if not 0 <= val <= 255:
                raise ValueError("your val for angular, {} is out of range.".format(val))
            angular = Int16()
            angular.data = val
            self.angularPub.publish(angular)

    #Functional cmd publishers
    def pub_accel(self, val):
        self.pub('accel', val)

    def pub_brake(self, val):
        self.pub('brake', val)

    def pub_steer(self, val):
        self.pub('steer', val)

    def pub_gear(self, val):
        self.pub('gear', val)

    def pub_angular(self, val):
        self.pub('angular', val)

    #mode manager

    #mode 0. Emergency Stop
    def emergency_stop_on(self, msg):
        self.pub_accel(0.0)
        self.pub_brake(29000.0)

    def emergency_stop_off(self):
        rospy.loginfo("E-Stop Off!")

    #mode 1. Cruise Control
    def PID(self,desired_vel):

        if desired_vel == -1.0:
            desired_vel = self.prev_desired_vel

        # When desired velocity is changed, prev_error and error_i should be reset!
        if desired_vel != self.prev_desired_vel:
            self.prev_error = 0
            self.error_i = 0
            self.prev_time = 0
        self.prev_desired_vel = desired_vel

        # Defining dt(del_time)
        self.cur_time = time.time()
        if self.prev_time ==0:
            self.del_time = 0
        elif self.prev_time!=0:
            self.del_time = self.cur_time - self.prev_time

        # Calculate Errors
        self.error_p = desired_vel - self.cur_vel
        self.error_i += self.error_p * (self.del_time)
        time.sleep(0.005)
        if (self.error_i < - self.antiwindup):
            self.error_i = - self.antiwindup
        elif (self.error_i > self.antiwindup):
            self.error_i = self.antiwindup
        if self.prev_time ==0:
            self.error_d = 0
        elif self.prev_time!=0:
            self.error_d = (self.error_p - self.prev_error)/self.del_time

        # PID Out
        pid_out = self.kp*self.error_p + self.ki*self.error_i + self.kd*self.error_d
        self.pid_out = pid_out
        if pid_out > 1000:
            pid_out = 1000
        elif pid_out < -1000:
            pid_out = -1000

        self.prev_error = self.error_p # Feedback current error
        self.prev_time = self.cur_time # Feedback current time

    	# accel_max - accel_dead_zone = 3000 - 800 = 2200
    	# 2200/10 = 220, 220 + 1 = 221
        '''
        if pid_out > 0:
            for i in range(221):
                if i <= pid_out < i+1:
                    return 800 + 10*i, 0
        '''
        if pid_out > 0:
            for i in range(858):
                if i <= pid_out < i+1:
                    gaspedal = 800 + 10*i
                    if gaspedal >= 3000:
                        gaspedal = 3000
                        return gaspedal, 0
                    else:
                        return gaspedal, 0


    	# brake_max - brake_dead_zone = 29000 - 3500 = 25500 (!!!!!!!should be changed to 29000 if needed!!!!!!!)
    	# 25500/10 = 2550, 2550 + 1 = 2551
        elif pid_out < 0:
            for i in range(2551):
                if i <= abs(pid_out) < i+1:
                    return 0, 2700+10*i

        return 0, 0

    def cruise_control(self,msg):
        self.mode_data = [msg.data[1], msg.data[2], 0.0, 0.0] #[mode, target_speed, 0.0, 0.0]

def main():
    rospy.init_node(NODENAME)
    move_car = Move_Ioniq()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()
