'''
Steer control with PID based on keyboardcontrol
Author: Seunggi Lee 
Date: 2020.06.30
Version: 2.0
# TODO
'''

import rclpy
from rclpy.qos import qos_profile_default
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import  Int16
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import threading
import time
import termios
import sys
import tty
import select


settings = termios.tcgetattr(sys.stdin)

msg = """
2020 ARTIV Ioniq Keyboard Drving Assist by Gwanjun Shin
---------------------------
Accel : 'w'
brake : 's'
fastBrake : 'x' More fast & Impact brake
Steer : Left : 'a' | Right : 'd'
Steer Function On Toggle : 'h'
Steer Function Off Toggle : 'j'
Set + 5 degree (In steer function) : 'u'
Set - 5 degree (In steer function) : 'n'
anything else : emergency Stop
CTRL-C to quit
WARNING! Must operate with driver and more than one assist!
"""

moveBindings = {
	    'a' : (-5), #deg
        'd' : (5), #deg
        'q' : (0)
	            }

speedBindings={
	    'w' : (50), #APS_ACT Feedback
        's' : (1500), #Brake_ACT Feedback ~20000
        'x' : (5000) #Brake_ACT Feedback ~20000 (HIGH)
	           }

steerControl={
		'h' : (1), # mode on
		'j' : (0), # mode off
		'u' : (1), # 
		'n' : (-1) #
	            }

steering_angle = 0
steer_mode = 0
desire_angle = 0

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def execute(node_):
	velSub = node_.create_subscription(Float32MultiArray,
			"/Ioniq_info",
            velCallback)
	rclpy.spin(node_)

def velCallback(floatmsg):
    global steering_angle
    steering_angle = floatmsg.data[3]
    #print(steering_angle)
    pass

def steerfunction(steerPub_):
    global steer_mode
    global steering_angle
    global desire_angle
	global error_i

    steer_ = Int16()

    prev_time = time.time()
    prev_error = 0
    error_i = 0

    while(1):
        if steer_mode == 1:
            
            os.system('cls' if os.name == 'nt' else 'clear')

            cur_time = time.time()
            del_time = cur_time - prev_time;

            # PID coefficient
            k_p = 1.25
            k_i = 0.75
            k_d = 0
            windup_guard = 70.0
			
            error_p = desire_angle - steering_angle
            error_i += error_p * (del_time)
			
			# Anti wind-up
            if (error_i < -windup_guard):
                error_i = -windup_guard
            elif (error_i > windup_guard):
                error_i = windup_guard
            
            error_d = (error_p - prev_error)/del_time
            pid_out = k_p*error_p + k_i*error_i + k_d*error_d
            
            prev_error = error_p # Feedback current error  
            prev_time = cur_time # Feedback current time 

			# angle_max - angle_dead_zone = 440 - 0 = 440
			# 440, 440 + 1 = 441
            if pid_out > 0:
                for i in range(441):
                    if i <= pid_out < i+1:
                        steer_.data = i
			
			# brake_max - brake_dead_zone = 440 - 0 = 440
			# 440, 440 + 1 = 441
            elif pid_out < 0:
                for i in range(441):
                    if i <= abs(pid_out) < i+1:
                        steer_.data = -i
			
			# Change PID coefficient through this values
            print("error_p: ", error_p)
            print("error_i: ", error_i)
            print("error_d: ", error_d)
            print("pidout: ", pid_out)
            print("desire_angle: ", desire_angle)
            print("current_angle: ", steering_angle)
            print("steer_data: ", steer_.data)

            steerPub_.publish(steer_)

            time.sleep(0.1)
     
        else:
            if desire_angle != 0:        
                desire_angle = 0
            steer_.data = 0
            

def main(args=None):
    global steering_angle
    global steer_mode
    global desire_angle
    
    rclpy.init()
    node = rclpy.create_node('steer_PID')
    steerPub = node.create_publisher(Int16, '/dbw_cmd/Steer', qos_profile_default)

    thread_velo = threading.Thread(target=execute, args=(node,))
    thread_velo.start()

    thread_steer = threading.Thread(target=steerfunction, args=(steerPub,))
    thread_steer.start()

    handle_set = 0
    status = 0

    handle_set_MAX = 440


    try:
        print(msg)
        while(1):

            if abs(handle_set) >= handle_set_MAX:
                Hsigned = -1 if handle_set < 0 else 1
                handle_set = Hsigned * handle_set_MAX


            #print('='*30,"\n")


            key = getKey()
            os.system('cls' if os.name == 'nt' else 'clear')
            #print('\n\n\n\n\n','='*30, sep='')

            if key in steerControl.keys():
                if key == "h" :
                    steer_mode = steerControl[key]
                if key == "j" :
                    steer_mode = steerControl[key]                
                if key == "u" :
                    desire_angle += steerControl[key]
					if error_i < 0 : error_i = 0
                if key == "n" :
                    desire_angle += steerControl[key]
					if error_i > 0 : error_i = 0

            elif key in moveBindings.keys():
                if key == "q" :
                    handle_set = 0
                handle_set += moveBindings[key]

            else:
                print("BRACE!! EMERGENCY STOP!!!\n"*3)
                ###MUST be ON THREAD!!!!
                os.system('''for k in {1..3};
                do
                    play -nq -t alsa synth 0.2 sine 544;
                    sleep 0.03;
                    play -nq -t alsa synth 0.2 sine 544;
                    sleep 0.03;
                    play -nq -t alsa synth 0.2 sine 544;
                    sleep 0.03;
                    play -nq -t alsa synth 0.2 sine 544;
                    sleep 0.03;
                done &''')
    
                if (key == '\x03'):
                	break
            print("steer_mode = ", steer_mode , "  , desire_angle = ", desire_angle)
            
            steer = Int16()

            steer.data = handle_set
            
            if steer_mode == 0:
                steerPub.publish(steer)
            
    except Exception as e:
        print(e)

    finally:
        steer = Int16()

        steer = 0

        steerPub.publish(steer)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    os.system('cls' if os.name == 'nt' else 'clear')
    main()
