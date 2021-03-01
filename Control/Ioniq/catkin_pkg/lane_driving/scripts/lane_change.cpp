// Pure Pursuit Tracker : Vision's point data -> Tracking
// Author : Yeo Ho Yeong (ARTIV), hoyeong23@dgist.ac.kr
// Date : 2020.12.04

#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#define _USE_MATH_DEFINES

using namespace cv;

const float origin_x = 150.0, origin_y = 300.0, dist_cam = 10.0, anti = 0.07, angular_input = 90.0;
const float rear_x = origin_x, rear_y = origin_y + dist_cam;
const float dt = 0.05;
const int vision_len = 31, gps_len = 30;

double I_gain = 0.0;
double global_x, global_y;

float prev_error, global_yaw, Kp, Ki, Kd; // Ki -> Vision, Ki_ -> GPS
float global_v = 30, prev_steer = 0, del_val = 0, vel_lim = 30;

int steer_input, speed_input, error_cnt, global_len, hlk_val = 5;
int x_coord_arr[31] = {}, y_coord_arr[31] = {};

geometry_msgs::Pose global_pose;

void update_v(double angle);
double update_Lf(double vel);
double calc_distance(double point_x, double point_y);
int find_smallest_idx(double (*arr));
int find_smallest_idx_vision(double (*arr));
double PID_control(double error);
void search_target_index(int& val_1, double& val_2, double (*arr_1), double (*arr_2));
void pure_pursuit_steer_control(double& val_1, double& val_2, int& val_3, int (*arr_1), int (*arr_2));
double update_steer(double cur_steer);
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
void vellimCallback(const std_msgs::Float32::ConstPtr& msg);
void visionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void ioniqCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void hlkCallback(const std_msgs::Int16::ConstPtr& msg);
void publishing(ros::Publisher cmd_cruise, ros::Publisher cmd_nav, std_msgs::Float32MultiArray msg1, std_msgs::Int16 msg2);
void subscribing();

void update_v(double angle){ // Delta range : -28 ~ 28 (degrees)
	double abs_angle = abs(angle);
	speed_input = 26;

	if (hlk_val == 2){
		if (abs_angle > 40/18){ // in curve
			if (del_val < 0){
				del_val = 0;
			}
			speed_input -= 10/(1 + exp(del_val - 6)); // 25 -> 15km/h (Deacceleration)
			del_val -= 0.3;
		}
		else{
			if (del_val > 12){
				del_val = 12;
			}
			speed_input -= 10/(1 + exp(del_val - 6)); // 15 -> 25km/h (Acceleration)
			del_val += 0.5;
		}
	}
	else if (hlk_val == 3){
		if (del_val < 0){
			del_val = 0;
		}
		speed_input -= 13/(1 + exp(del_val - 6)); // 25 -> 12km/h (Deacceleration)
		del_val -= 0.5;
	}

	if (speed_input > vel_lim){
		speed_input = vel_lim;
	}
}

double update_Lf(double vel){ // velocity region : 0 ~ 80km/h
	double result_Lf;

	result_Lf = (vel - 20) / 15;
	result_Lf = 1/(1 + exp(-result_Lf)) - 0.5;
	result_Lf = 190 + result_Lf * 100;

	if (result_Lf < 20){
		result_Lf = 20;
	}
	else if(result_Lf > 250){
		result_Lf = 250;
	}

	return result_Lf;
}

double calc_distance(double point_x, double point_y){
	double dx, dy;
	dx = rear_x - point_x;
	dy = rear_y - point_y;

	return hypot(dx, dy);
}

int find_smallest_idx(double (*arr)){
	int i, len_val = 10, smallest_idx = 0;
	double smallest_val = arr[0];

	for (i = 0; i < len_val; i++){
		if (arr[i] < smallest_val){
			smallest_val = arr[i];
			smallest_idx = i;
		}
	}

	return smallest_idx;
}

int find_smallest_idx_vision(double (*arr)){
	int i, smallest_idx = 0;
	double smallest_val = arr[0];

	for (i = 0; i < vision_len; i++){
		if (arr[i] < smallest_val){
			smallest_val = arr[i];
			smallest_idx = i;
		}
	}

	return smallest_idx;
}

double PID_control(double error){ // PID Control for Vision
	double P_gain, D_gain, PID_gain;
	P_gain = error;
	I_gain += error * dt;
	D_gain = (error - prev_error) / dt;

	if (I_gain > anti){
		I_gain = anti;
	}
	else if (I_gain < -anti){
		I_gain = -anti;
	}
	PID_gain = Kp * P_gain + Ki * I_gain + Kd * D_gain;
	prev_error = error;

	return PID_gain;
}

void search_target_index(int& val_1, double& val_2, double (*arr_1), double (*arr_2)){ 
	// Find shortest point and index for look ahead point
	int i, target_idx, add_to_index, target_idx_of_arr;
	double Lf_, del_x, del_y;
	double int_x_arr[10] = {}, int_y_arr[10] = {};
	double hypot_arr_Lf[31] = {};
	double hypot_int_arr[10] = {};

	Lf_ = update_Lf(global_v);

	if (hlk_val == 5){ // Tracking by Vision
		for (i = 0; i < vision_len; i++){
			hypot_arr_Lf[i] = (abs(calc_distance(x_coord_arr[i], y_coord_arr[i]) - Lf_));
		}

		target_idx = find_smallest_idx_vision(hypot_arr_Lf);

		if (calc_distance(x_coord_arr[target_idx], y_coord_arr[target_idx]) > Lf_){ 
			// if target point is more far than circle's radius
			add_to_index = target_idx + 1;
			del_x = abs(x_coord_arr[target_idx] - x_coord_arr[add_to_index]) / 11.0;
			del_y = abs(y_coord_arr[target_idx] - y_coord_arr[add_to_index]) / 11.0;

			// Point interpolation
			if (x_coord_arr[target_idx] > 150){
				for (i = 0; i < 10; i++){
					int_x_arr[i] = x_coord_arr[add_to_index] + del_x * (i+1);
					int_y_arr[i] = y_coord_arr[add_to_index] - del_y * (i+1);
				}
			}
			else{
				for (i = 0; i < 10; i++){
					int_x_arr[i] = x_coord_arr[target_idx] + del_x * (i+1);
					int_y_arr[i] = y_coord_arr[target_idx] + del_y * (i+1);
				}
			}
		}
		else{ 
			// if target point is less far than circle's radius
			add_to_index = target_idx - 1;
			del_x = abs(x_coord_arr[target_idx] - x_coord_arr[add_to_index]) / 11.0;
			del_y = abs(y_coord_arr[target_idx] - y_coord_arr[add_to_index]) / 11.0;

			// Point interpolation
			if (x_coord_arr[target_idx] > 150){
				for (i = 0; i < 10; i++){
					int_x_arr[i] = x_coord_arr[target_idx] + del_x * (i+1);
					int_y_arr[i] = y_coord_arr[target_idx] - del_y * (i+1);
				}
			}
			else{
				for (i = 0; i < 10; i++){
					int_x_arr[i] = x_coord_arr[add_to_index] + del_x * (i+1);
					int_y_arr[i] = y_coord_arr[add_to_index] + del_y * (i+1);
				}
			}
		}

		for (i = 0; i < 10; i++){
			hypot_int_arr[i] = abs(calc_distance(int_x_arr[i], int_y_arr[i]) - Lf_);
		}
		
		target_idx_of_arr = find_smallest_idx(hypot_int_arr);
		val_1 = target_idx_of_arr;
		val_2 = Lf_;
		for (i = 0; i < 10; i++){
			arr_1[i] = int_x_arr[i];
			arr_2[i] = int_y_arr[i];
		}
	}
}

void pure_pursuit_steer_control(double& val_1, double& val_2, int& val_3, int (*arr_1), int (*arr_2)){
	int i, target_index;
	double Lf, alpha, delta, pid_result, del_x, del_y;
	double int_x_arr[10] = {};
	double int_y_arr[10] = {};

	search_target_index(target_index, Lf, int_x_arr, int_y_arr);

	if (hlk_val == 5){ // for vision
		del_x = abs(int_x_arr[target_index] - rear_x);
		del_y = abs(int_y_arr[target_index] - rear_y);

		if (int_x_arr[target_index] - rear_x > 0){
			alpha = atan2(del_x, del_y);
		}
		else{ // target_x - rear_x < 0
			alpha = -atan2(del_x, del_y);
		}

		delta = atan2(2 * dist_cam * sin(alpha) / Lf, 1.0);

		if (delta >= 30 * M_PI / 180){ // Limit Angle of Front Wheel : -30 ~ +30
			delta = 30 * M_PI / 180;
		}
		else if (delta <= -30 * M_PI / 180){
			delta = -30 * M_PI / 180;
		}

		pid_result = PID_control(delta);
		val_1 = pid_result * 180 / M_PI; // Desired Angle value of Front Wheel
		val_2 = Lf;
		val_3 = target_index;
		for (i = 0; i < 10; i++){
			arr_1[i] = int_x_arr[i];
			arr_2[i] = int_y_arr[i];
		}
	}
}

double update_steer(double cur_steer){
	if (abs(cur_steer - prev_steer) > 5){
		if (error_cnt < 6){
			cur_steer = prev_steer;
			error_cnt += 1;
		}
		else{
			error_cnt = 0;
			I_gain = 0;
		}
	}
	else{
		error_cnt = 0;
	}

	prev_steer = cur_steer;

	return cur_steer;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){  
// Update ioniq's velocity
	global_v = msg->velocity[0];
}

void vellimCallback(const std_msgs::Float32::ConstPtr& msg){
	vel_lim = msg->data;
}

void visionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
// Update Center Points
	int i, tar_idx;
	double steer_pd, steer, Lh, result;
	char text_input_1[] = "Steer : ";
	int int_x_arr[10] = {}, int_y_arr[10] = {};

	for (i = 0; i < msg->data.size()/2; i++){ // Add x, y points to coords vectors
		x_coord_arr[i] = msg->data[2*i];
		y_coord_arr[i] = msg->data[2*i+1];
	}

	// Plotting Result
	Mat pursuit_img = Mat::zeros(300, 400, CV_8UC3); // Plot Blank View
	for (i = 0; i < vision_len; i++){ // Plot Vision Points
		circle(pursuit_img, Point(x_coord_arr[i], y_coord_arr[i]), 3, Scalar(0, 255, 0), FILLED, LINE_8);
	}

	if (hlk_val == 5){ // Tracking for Vision data
		pure_pursuit_steer_control(steer, Lh, tar_idx, int_x_arr, int_y_arr);
		update_v(steer);
		steer_pd = update_steer(steer * 18.0);
		steer_input = (int)round(steer_pd);
		putText(pursuit_img, "VISION MODE", Point(100, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 2);
		for (i = 0; i < 10; i++){
			circle(pursuit_img, Point(int_x_arr[i], int_y_arr[i]), 3, Scalar(100, 100, 100), FILLED, LINE_8);
		}
		circle(pursuit_img, Point(int_x_arr[tar_idx], int_y_arr[tar_idx]), 5, Scalar(255, 255, 0), FILLED, LINE_8);
	}

	putText(pursuit_img, "Steer : " + std::to_string(steer_input), Point(100, 100), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 2);

	if(steer_input > 0){
		circle(pursuit_img, Point(rear_x, rear_y), Lh, Scalar(0, 0, 255), 2);
	}
	else{
		circle(pursuit_img, Point(rear_x, rear_y), Lh, Scalar(255, 0, 0), 2);
	}
	imshow("Result", pursuit_img);
	waitKey(1);
}

void ioniqCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	float auto_switch = msg->data[5];
	float APM_switch = msg->data[6];

	if (auto_switch == 0.0 || APM_switch == 0.0){
		I_gain = 0;
	}
}

void hlkCallback(const std_msgs::Int16::ConstPtr& msg){
	hlk_val = msg->data;

	if (hlk_val != 5){
		I_gain = 0;
	}
}

void publishing(ros::Publisher cmd_cruise, ros::Publisher cmd_nav, std_msgs::Float32MultiArray msg1, std_msgs::Int16 msg2){
	int i;

	std::vector<float> move_car_data(10, 0.0);
	while(ros::ok()){
		usleep(1000);
		if (hlk_val == 5){
			move_car_data[0] = 0.0;
			move_car_data[1] = 6.0;
			move_car_data[2] = speed_input;
			move_car_data[5] = steer_input;
			move_car_data[6] = 5.0;
			move_car_data[7] = angular_input;
			msg1.data = move_car_data;
			msg2.data = 0;
			cmd_cruise.publish(msg1);
			cmd_nav.publish(msg2);
		}
	}
}

void subscribing(){
	ros::spin();
}

int main(int argc, char * argv[]){
	std::cout<<"Starting Pure Pursuit..."<<std::endl;
	ros::init(argc, argv, "pure_tracker");
	ros::NodeHandle n;
	ros::Subscriber joint_sub = n.subscribe("Joint_state", 1, jointCallback);
	ros::Subscriber ioniq_sub = n.subscribe("Ioniq_info", 1, ioniqCallback);
	ros::Subscriber vision_sub = n.subscribe("enetsad/centerPt", 1, visionCallback);
	ros::Subscriber vellim_sub = n.subscribe("smartcruise/desired_velocity", 1, vellimCallback);
	ros::Subscriber hlk_sub = n.subscribe("hlk_switch", 1, hlkCallback);
	ros::Publisher cmd_cruise = n.advertise<std_msgs::Float32MultiArray>("/move_car", 1);
	ros::Publisher cmd_nav = n.advertise<std_msgs::Int16>("/mission_manager/nav_pilot", 1);

	std_msgs::Float32MultiArray msg_1;
	std_msgs::Int16 msg_2;
	std_msgs::Int16 msg_3;
	std::cout<<"Starting Threading..."<<std::endl;
	std::thread t1(publishing, cmd_cruise, cmd_nav, msg_1, msg_2);
	std::thread t2(subscribing);
	std::cout<<"Please Input Points, Eunbin!!!"<<std::endl;
	t1.join();
	t2.join();
	return 0;
}
