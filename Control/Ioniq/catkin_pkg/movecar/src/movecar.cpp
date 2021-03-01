/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For a temporary license for autonomous vehicle
 *******************************************************/
#include "movecar/movecar.h"

using namespace std;

movecar::movecar(ros::NodeHandle *nodehandle){

	ros::NodeHandle nh(*nodehandle);
	ROS_INFO("start move car");
	ros::param::get("~kp", KP);
	ros::param::get("~ki", KI);
	ros::param::get("~kd", KD);
	ros::param::get("~aw", AW);
	vel_pid.set(KP, KI, KD, AW);
	ros::param::get("~brake_scale", brake_scale);
	ros::param::get("~upper_threathold", upper_threathold);
	ros::param::get("~lower_threathold", lower_threathold);

	ros::param::get("~hz", hz);

	ros::param::get("~max_speed", maximum_speed);
	ros::param::get("~speed_buffer", buff);

	ROS_INFO("PID prefix p : %lf i : %lf d : %lf aw : %lf", KP, KI, KD, AW);
	ROS_INFO("HZ : %d brake scale : %lf uppper_threathold : %lf lower_threathold : %lf", hz, brake_scale, upper_threathold, lower_threathold);
	ROS_INFO("MaxSpeed : %lf Speed Buffer : %lf", maximum_speed, buff);

	ros::Subscriber stanly_sub = nh.subscribe("movecar_stanly", 1, &movecar::stanly_callback, this);
	ros::Subscriber pure_sub = nh.subscribe("/move_car", 1, &movecar::pure_callback, this);

	ros::Subscriber estop_sub =nh.subscribe("/move_car/Estop",1, &movecar::estop_callback, this);

	ros::Subscriber info_sub = nh.subscribe("Ioniq_info", 1, &movecar::info_callback, this);
	ros::Subscriber max_speed_sub = nh.subscribe("/dbw_cmd/MaxSpeed", 1, &movecar::maximum_speed_callback, this);

	acc_pub = nh.advertise<std_msgs::Int16>("/dbw_cmd/Accel", 1);
	brake_pub = nh.advertise<std_msgs::Int16>("/dbw_cmd/Brake", 1);
	steer_pub = nh.advertise<std_msgs::Int16>("/dbw_cmd/Steer", 1);
	angular_pub = nh.advertise<std_msgs::Int16>("/dbw_cmd/Angular", 1);
	state_pub = nh.advertise<std_msgs::Int16>("/dbw_cmd/Status", 1);
	calc_thread = boost::thread(boost::bind(&movecar::calculation, this));
	ros::spin();
}
movecar::~movecar(){
	calc_thread.join();
	cout<<"kill thread"<<endl;
	ROS_INFO("movecar stoped cleary");
}
void movecar::calculation(){
	while(pure.size() == 0 && ros::ok()){
		//cout<<"waiting"<<endl;
	}
	while(ros::ok()){
		std_msgs::Int16 state;
		if(Estop_flag || Estop_switch){
			cout<<"Stuck in Estop, Please reset car"<<endl;
			state.data = 0;
			state_pub.publish(state);
			usleep(1000000/hz);
			continue;
		}
		state.data = 1;
		state_pub.publish(state);
		if(flag){//임의로 pure = 0, stanly = 1 
			desire_velocity = stanly[2];
			desire_steer = stanly[5];
		}
		else{
			desire_velocity = pure[2];
			desire_steer = pure[5];
			desire_angular = pure[7];
		}
		if(desire_velocity >= (maximum_speed - buff)){
			ROS_WARN("Input speed over the maximum speed");
			desire_velocity = maximum_speed - buff;
			vel_pid.reset_pid();
		}
		cout<<"now velocity:"<<now_velocity<<"desire velocity :"<<desire_velocity<< endl;
		
		double data = Scale*vel_pid.out(desire_velocity, now_velocity);
		if(desire_velocity <= 0){
			publishing(4,0,desire_steer,desire_angular);
		}
		else if(data<upper_threathold && data>=lower_threathold){
			publishing(1,0,desire_steer,desire_angular);
		}
		else if(data > upper_threathold){
			publishing(2,data,desire_steer,desire_angular);
		}
		else{
			publishing(3,data,desire_steer,desire_angular);
		}
		usleep(1000000/hz);
	}
}
void movecar::publishing(int mode, double data, double steer_data, double angular_data){
	std_msgs::Int16 acc;
	std_msgs::Int16 brake;
	std_msgs::Int16 steer;
	std_msgs::Int16 angular;
	switch(mode){
		case 0:
			cout<<"Estop"<<endl;
			acc.data = 0;
			brake.data = 28000;
			steer.data = 0;
			angular.data = 0;
			break;
		case 1: //zero zone
			cout<<endl<<"zero!!"<<endl<<endl;
			acc.data = 0;
			brake.data = 0;
			steer.data = steer_data;
			angular.data = angular_data;
			break;
		case 2: // accel
			cout<<endl<<"accel!!"<<endl<<endl;
			acc.data = min(3000, 800 + int(abs(data)));
			brake.data = 0;
			steer.data = steer_data;
			angular.data = angular_data;
			break;
		case 3: //brake
			cout<<endl<<"brake!!"<<endl<<endl;
			acc.data = 0;
			brake.data = min(25000, 3800 + int(abs(data*brake_scale)));
			steer.data = steer_data;
			angular.data = angular_data;
			break;
		case 4:
			cout<<endl<<"Stop"<<endl<<endl;
			acc.data = 0;
			brake.data = min(27000, 4500 + int( sqrt(now_velocity/maximum_speed)*(27000 - 4200)));
			steer.data = steer_data;
			angular_data = angular_data;
			break;
	}
	acc_pub.publish(acc);
	brake_pub.publish(brake);
	steer_pub.publish(steer);
	angular_pub.publish(angular);
}
void movecar::estop_callback(const std_msgs::Bool msg){
	if(Estop_switch){
		cout<<"Already turn on Estop"<<endl;
		return;
	}
	Estop_flag = msg.data;
	if(Estop_flag && !Estop_switch){
		publishing(0,0,0,0);
		usleep(int(1000000.0*now_velocity/7.0)+1000000*3);
		cout<<"time : "<<now_velocity/7<<endl;
		publishing(1,0,0,0);
		std_msgs::Int16 data;
		data.data = 0;
		state_pub.publish(data);
		Estop_switch = true;
	}
}
void movecar::maximum_speed_callback(const std_msgs::Int16::ConstPtr& msg){
	maximum_speed = double(msg->data);
}
void movecar::stanly_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	stanly = msg->data;
	return;
}
void movecar::pure_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	pure = msg->data;
	return;
}
void movecar::info_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	now_velocity = msg->data[19];
	if(!msg->data[5]){
		cout<<"reset move_car"<<endl;
		vel_pid.reset_pid();
		Estop_flag = false;
		Estop_switch = false;
	}
	return;
}
void movecar::switch_callback(const std_msgs::Int16::ConstPtr& msg){
	flag = msg->data;
	return;
}