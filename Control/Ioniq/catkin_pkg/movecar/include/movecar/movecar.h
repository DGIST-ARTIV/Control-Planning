#pragma once
#include "movecar/pid.h"
#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "time.h"
#include <boost/thread.hpp>

class movecar{
public:
	vector<float> stanly, pure;
	int flag = 0;
	bool Estop_flag = false;
	bool Estop_switch = false;
	double desire_velocity{0}, now_velocity{0}, desire_steer{0}, desire_angular{0};
	double Angular = 70;
	ros::Publisher acc_pub, brake_pub, steer_pub, angular_pub, state_pub;
	double upper_threathold{50}, lower_threathold{-50};
	double brake_scale = 10;
	double KP{20}, KI{7}, KD{5}, AW{40}, Scale{1};
	int hz = 10;
	double maximum_speed = 100;
	double buff = 5;
	PID vel_pid = PID(KP, KI, KD, AW);;
	boost::thread calc_thread;

	movecar(ros::NodeHandle *nodehandle);
	~movecar();
	void calculation();
	void Estop();

	void publishing(int mode, double data, double steer_data, double angular_data);
	void estop_callback(const std_msgs::Bool msg);

	void maximum_speed_callback(const std_msgs::Int16::ConstPtr& msg);
	void stanly_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void pure_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void info_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void switch_callback(const std_msgs::Int16::ConstPtr& msg);
};