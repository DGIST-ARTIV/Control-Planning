'''
Cruise control with PID based on keyboardcontrol
Author: Seunggi Lee 
Date: 2020.05.23
Version: 1.0

# TODO

'''



import rclpy
from rclpy.qos import qos_profile_default
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import  Int16
from sensor_msgs.msg import JointState
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
Cruise Function Toggle : 'k'
Set +0.1km/h () : 'i'
Set -0.5km/h () : 'm'
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
cruiseControl={
        'k' : (1), #mode
        'l' : (0), #mode
        'i' : (1), #km/h
        'm' : (-1) #km/h
	           }
velocity = 0
cruise_mode = 0
cruise_speed = 5

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def execute(node_):
    velSub = node_.create_subscription(JointState,
            "/Joint_state",
            velCallback)
    rclpy.spin(node_)


def velCallback(msg):
    global velocity
    #print('velocity', msg.velocity[0])
    velocity = msg.velocity[0]
    #print(msg.data)
    pass

def cruise(accelPub_, brakePub_):
    global velocity
    global cruise_mode
    global cruise_speed
	global error_i
    accel_ = Int16()
    brake_ = Int16()

    prev_time = time.time()
    prev_error = 0
    error_i = 0

    while(1):
        if cruise_mode == 1:
            #accel_ = Int16()
            #brake_ = Int16()
            os.system('cls' if os.name == 'nt' else 'clear')
			
            cur_time = time.time() 
            del_time = cur_time - prev_time;
			
			# PID coefficient
            k_p = 1.25
            k_i = 0.75
            k_d = 0
            windup_guard = 70.0
			
            error_p = cruise_speed - velocity
			error_i += error_p * del_time
			
			# Anti wind-up
            if (error_i < -windup_guard):
                error_i = -windup_guard
            elif (error_i > windup_guard):
                error_i = windup_guard
            
            error_d = (error_p - prev_error)/del_time
            pid_out = k_p*error_p + k_i*error_i + k_d*error_d
            
            prev_error = error_p # Feedback current error  
            prev_time = cur_time # Feedback current time 

			# accel_max - accel_dead_zone = 3000 - 800 = 2200
			# 2200/10 = 220, 220 + 1 = 221
            if pid_out > 0:
                for i in range(221):
                    if i <= pid_out < i+1:
                        accel_.data = 800 + 10*i
                        brake_.data = 0
			
			# brake_max - brake_dead_zone = 27000 - 3500 = 23500
			# 23500/10 = 2350, 2350 + 1 = 2351
            elif pid_out < 0:
                for i in range(2351):
                    if i <= abs(pid_out) < i+1:
                        accel_.data = 0
                        brake_.data = 3500+10*i
			
			# Change PID coefficient through this values
            print("error_p: ", error_p)
            print("error_i: ", error_i)
			print("error_d: ", error_d)
            print("pidout: ", pid_out)
            print("desired_speed: ", cruise_speed)
            print("current_speed: ", velocity)
            print(accel_.data, brake_.data)

            #accel_ = accel_out
            brakePub_.publish(brake_)
            accelPub_.publish(accel_)
            time.sleep(0.1)
     
        else:
            if cruise_speed != 5:        
                cruise_speed = 5
            accel_.data = 800
            brake_.data = 0

def vels(speed,turn, accel, brake):
	return (f"currently:\tpropulsion rate {speed}\t handle set {turn}\n \t\t aceel : {accel}, brake { brake}")

def main(args=None):
    global velocity
    global cruise_mode
    global cruise_speed
    
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    accelPub = node.create_publisher(Int16, '/dbw_cmd/Accel', qos_profile_default)
    brakePub  = node.create_publisher(Int16, '/dbw_cmd/Brake', qos_profile_default)
    steerPub = node.create_publisher(Int16, '/dbw_cmd/Steer', qos_profile_default)

    thread_velo = threading.Thread(target=execute, args=(node,))
    thread_velo.start()

    thread_cruise = threading.Thread(target=cruise, args=(accelPub, brakePub,))
    thread_cruise.start()

    handle_set = 0
    accelACT = 650

    brakeACT = 8500
    status = 0

    accelACT_MAX = 3000
    brakeACT_MAX = 27000
    handle_set_MAX = 440

    propulsion_rate = ((accelACT/accelACT_MAX)-(brakeACT/brakeACT_MAX))/(2)*100

    try:
        print(msg)
        print(vels(propulsion_rate,handle_set, accelACT, brakeACT))
        while(1):
            accelACT = accelACT if accelACT <= accelACT_MAX else accelACT_MAX
            brakeACT = brakeACT if brakeACT <= brakeACT_MAX else brakeACT_MAX

            if abs(handle_set) >= handle_set_MAX:
                Hsigned = -1 if handle_set < 0 else 1
                handle_set = Hsigned * handle_set_MAX

            propulsion_rate = ((accelACT/accelACT_MAX)-(brakeACT/brakeACT_MAX))/(1)*100

            #print('='*30,"\n")


            key = getKey()
           
            #print('\n\n\n\n\n','='*30, sep='')

            if key in cruiseControl.keys():
                if key == "k" :
                    cruise_mode = cruiseControl[key]
                if key == "l" :
                    cruise_mode = cruiseControl[key]                
                if key == "i" :
                    cruise_speed += cruiseControl[key]
                if key == "m" :
                    cruise_speed += cruiseControl[key]


                    
            elif key in moveBindings.keys():
                if key == "q" :
                    handle_set = 0
                handle_set += moveBindings[key]

            elif key in speedBindings.keys():
                if key == "w" :
                    brakeACT = 0
                    accelACT += speedBindings[key]
                if key == "s" :
                    accelACT = 650
                    brakeACT += speedBindings[key]
                if key == "x" :
                    accelACT = 0
                    brakeACT += speedBindings[key]
                if (status == 14):
                    print(msg)
                    status = (status + 1) % 15

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
                brakeACT = brakeACT_MAX
    
                if (key == '\x03'):
                	break
            print("cruise_mode = ", cruise_mode , "  , cruise_speed = ", cruise_speed)
            print(vels(propulsion_rate,handle_set, accelACT, brakeACT))
            accel = Int16()
            brake = Int16()
            steer = Int16()

            accel.data = accelACT
            brake.data = brakeACT

            steer.data = handle_set
            
            if cruise_mode == 0:
                brakePub.publish(brake)
                accelPub.publish(accel)
            steerPub.publish(steer)
            
    except Exception as e:
        print(e)

    finally:
        accel = Int16()
        brake = Int16()
        steer = Int16()

        accel.data = 0
        brake.data = 8500
        steer = 0
	
	
        brakePub.publish(brake)
        accelPub.publish(accel)
        steerPub.publish(steer)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    os.system('cls' if os.name == 'nt' else 'clear')
    main()
