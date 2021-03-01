// HLK Switch
// Author : Yeo Ho Yeong (ARTIV), hoyeong23@dgist.ac.kr
// Date : 2020.12.06
// Default : Auto mode, GPS stable, Not MPTG

#include <bits/stdc++.h>
#include <typeinfo>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/NavSatFix.h"

struct Switch{
	int auto_switch = 1; // 0 : Auto OFF, 1 : Auto On
	int mptg_switch = 1; // 0 : MPTG, 1 : Not MPTG
	int mission_switch; // 0 : Pure Pursuit, 1 : Stanley
	int lane_stab_switch; // 0 : Stable, 1 : Unstable
	int gps_stab_switch;  // 0 : Stable, 1 : Unstable
	int mission_info_switch; // 0 : Free, 1 : Vision Only
	int hlk_switch; //(Last Switch) 0 : None, 1 : MPTG, 2 : Pure Pursuit(Vision), 3 : Pure Pursuit(GPS), 4 : Stanley, 5 : Lane Change
};

Switch swit;

void autoCallback(const std_msgs::Int16::ConstPtr& msg){
	swit.auto_switch = 1; // 0 : Auto OFF, 1 : Auto On
}

void mptgCallback(const std_msgs::Int16::ConstPtr& msg){
	swit.mptg_switch = 1; // 0 : MPTG, 1 : Not MPTG
}

void switchCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	int switch_len = msg->data.size();
	int switch_check = 0;
	int s;


	for(s = 1; s < switch_len; s += 3){

		if(msg->data[s] == 1){

			if(msg->data[s+1] >= 0){
				switch_check = 1;
				break;
			}
		}
		if(msg->data[s] == 0){
			if(msg->data[s-1] ==1){	continue; }

			else if (msg->data[s-1] == 2 || msg->data[s-1] == 10){
				switch_check = 2; // Pure Pursuit GPS Only

			}

			else if(msg->data[s+1] == 0){
				switch_check = 0;
				break;
			}
		}
	}

	swit.mission_switch = switch_check; // 0 : Pure Pursuit, 1 : Stanley
}

void lanestabCallback(const std_msgs::Int16::ConstPtr& msg){
	swit.lane_stab_switch = msg->data;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
	double covar = msg->position_covariance[0];

	if (covar < 7){
		swit.gps_stab_switch = 0; // Stable
	}
	else{
		swit.gps_stab_switch = 0; // Unstable
	}
}

void missioninfoCallback(const std_msgs::Float32MultiArray msg)
{
	for(int i =0; i<msg.data.size()/3; i++)
	{
		if(msg.data[3*i+1] == 10)
		{

			if (msg.data[3*i+2] >= 0){
				swit.mission_info_switch = 1; // Pure Pursuit Vision Only
			}
			else{
				swit.mission_info_switch = 0; // None
			}
		}
		else
		{
			swit.mission_info_switch = 0;
		}
	}
}

void last_switch(){
	if (swit.auto_switch == 0){ // None
		swit.hlk_switch = 0;
		ROS_INFO("NONE MODE[1]");
	}
	else{
		if (swit.mptg_switch == 0){ // MPTG
			swit.hlk_switch = 1;
			ROS_INFO("MPTG MODE");
		}
		else{ // Not MPTG
			if (swit.mission_info_switch == 0){ // Free Mode
				if (swit.mission_switch == 0){ // Pure Pursuit

					if(swit.mission_info_switch == 1){
						swit.hlk_switch = 2;
					}
					else{
						if (swit.lane_stab_switch == 0){ // Lane Stable
							swit.hlk_switch = 2; // Pure Pursuit(Vision)
							//ROS_INFO("PURE PURSUIT VISION");
						}
						else{ // Lane Unstable
							if (swit.gps_stab_switch == 0){ // GPS Stable
								swit.hlk_switch = 3; // Pure Pursuit(GPS)
								//ROS_INFO("PURE PURSUIT GPS");
							}
							else{ // GPS Unstable
								swit.hlk_switch = 0; // None
								ROS_INFO("NONE MODE[2]");
							}
						}
					}

				}
				else if (swit.mission_switch == 2){
					swit.hlk_switch = 3; //GPS Only
				}
				else{ // Stanley
					if (swit.gps_stab_switch == 0){ // GPS Stable
						swit.hlk_switch = 4; // Stanley
						//ROS_INFO("STANLEY MODE");
					}
					else{ // GPS Unstable
						swit.hlk_switch = 0; // None
						ROS_INFO("NONE MODE[3]");
					}
				}
			}
			else{ // Vision Only Mode
				swit.hlk_switch = 2; // Pure Pursuit(Vision)
				//ROS_INFO("VISION ONLY MODE");
			}
		}
	}
}

void publishing(ros::Publisher switch_pub, std_msgs::Int16 msg){
	while(ros::ok()){
		last_switch();
		usleep(1000);
		msg.data = 3;//swit.hlk_switch;
		switch_pub.publish(msg);
	}
}

void subscribing()
{
	ros::spin();
}

int main(int argc, char * argv[]){
	std::cout<<"Turnning On the HLK_Switch..."<<std::endl;
	ros::init(argc, argv, "HLK_Switch");
	ros::NodeHandle n;
	ros::Subscriber auto_sub = n.subscribe("Auto_driving", 1, autoCallback);
	ros::Subscriber mptg_sub = n.subscribe("planner/flag", 1, mptgCallback);
	ros::Subscriber mission_sub = n.subscribe("hdmap/switch_info", 1, switchCallback);
	ros::Subscriber missioninfo_sub = n.subscribe("hdmap/mission_info", 1, missioninfoCallback);
	ros::Subscriber lane_sub = n.subscribe("lane_stab", 1, lanestabCallback);
	ros::Subscriber gps_sub = n.subscribe("gps_fix", 1, gpsCallback);
	ros::Publisher switch_pub = n.advertise<std_msgs::Int16>("hlk_switch", 1);

	std_msgs::Int16 msg;
	std::thread t1(publishing, switch_pub, msg);
	std::thread t2(subscribing);
	t1.join();
	t2.join();
	return 0;
}
