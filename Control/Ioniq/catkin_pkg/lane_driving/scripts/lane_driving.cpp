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
const float dt = 0.05, dt_ = 0.1;
const int vision_len = 31, gps_len = 30;
int max_speed = 100;
double velocity_term;

double I_gain = 0.0, I_gain_ = 0.0;
double global_x, global_y;
double global_x_arr[30] = {}, global_y_arr[30] = {};
double global_virtual;

double prev_error, global_yaw, Kp, Ki, Kd, Kp_, Ki_, Kd_; // Ki -> Vision, Ki_ -> GPS
//float LD_A, LD_B, LD_C;
double global_v = 30, prev_steer = 0, del_val = 0, virtual_steer = 0;
double vel_lim = 100; 
int steer_input, error_cnt, global_len;
double desire_vel = 100, curve_vel = 0;
float speed_input = 0;
int global_switch = 0, hlk_val = 2, lane_stab = 0;
int x_coord_arr[31] = {}, y_coord_arr[31] = {};

geometry_msgs::Pose global_pose;

double min_filter = 3;


void update_v(double angle);
double update_Lf(double vel);
double calc_distance(double point_x, double point_y);
int find_smallest_idx(double (*arr));
int find_smallest_idx_gps(double (*arr));
int find_smallest_idx_vision(double (*arr));
double PID_control(double error);
double PID_control_(double error);
void search_target_index(int& val_1, double& val_2, double (*arr_1), double (*arr_2));
void pure_pursuit_steer_control(double& val_1, double& val_2, int& val_3, int (*arr_1), int (*arr_2));
void checkState(void);
double update_steer(double cur_steer);
void switchCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void mapCallback(const sensor_msgs::JointState::ConstPtr& msg);
void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void yawCallback(const std_msgs::Float64::ConstPtr& msg);
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
void vellimCallback(const std_msgs::Float64::ConstPtr& msg);
void visionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void ioniqCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void hlkCallback(const std_msgs::Int16::ConstPtr& msg);
void publishing(ros::Publisher cmd_cruise, ros::Publisher cmd_nav, ros::Publisher lane_stab_pub, std_msgs::Float32MultiArray msg1, std_msgs::Int16 msg2, std_msgs::Int16 msg3);
void subscribing();

void update_v(double angle){ // Delta range : -28 ~ 28 (degrees)
	double abs_angle = 1000*abs(angle)*M_PI / 180; //--> ioniq handle steer, not wheel angle
	//std::cout<<abs_angle<<std::endl;
	//speed_input = 26;
	double filter = max(min_filter, abs(desire_vel - global_v) / 40);
	//std::cout<<filter<<std::endl;
	
	if(desire_vel<=1){
		speed_input = 0;
	}
	else if(desire_vel >= global_v){
		speed_input = min(desire_vel, speed_input + filter);
	}
	else{
		speed_input = max(desire_vel, speed_input - filter*4);
	}
	

	if (hlk_val == 2){
		if (abs_angle >=25){// in curve
			//if (del_val < 0){
			//	del_val = 0;
			//}
			//speed_input -= 8/(1 + exp(del_val - 6)); // 25 -> 15km/h (Deacceleration)
			//if(speed_input >= max_speed-13){speed_input -= velocity_term;}
			//else if(speed_input < max_speed-13) {speed_input += velocity_term;}
			//speed_input = 60;
			//del_val -= 0.3;
			curve_vel -= velocity_term;
		}
		else if (abs_angle < 25 && abs_angle>=10){
			curve_vel -= velocity_term/2;
		}

		else if (abs_angle < 10 && abs_angle>=0)
		{
			//if (del_val > 12){
			//	del_val = 12;
			//}
			//speed_input -= 8/(1 + exp(del_val - 6)); // 15 -> 25km/h (Acceleration)
			curve_vel = 0;
			//if(speed_input >= max_speed-5){speed_input -= velocity_term;}
			//else if(speed_input < max_speed-5){speed_input += velocity_term;}
			//speed_input = 70;
			//del_val += 0.5;
		}

	}
	else if (hlk_val == 3){
		std::cout<<"3"<<std::endl;
		if (del_val < 0){
			del_val = 0;
		}
		speed_input -= 8/(1 + exp(del_val - 6)); // 25 -> 12km/h (Deacceleration)
		//while (desire_vel > 60){desire_vel -=1;}
		del_val -= 0.5;
	}

	//if (speed_input > vel_lim){
	//	speed_input = vel_lim;
	//}
	else
	{
	std::cout<<"None"<<std::endl;
	//speed_input = 80;
	}

	if (vel_lim < speed_input){speed_input=vel_lim;}
	else{}
	
}

double update_Lf(double vel){ // velocity region : 0 ~ 80km/h
	double result_Lf;

	result_Lf = (vel - 20) / 15;
	result_Lf = 1/(1 + exp(-result_Lf)) - 0.5;

	result_Lf = 190 + result_Lf * 100;
        //result_Lf = LD_A*vel*vel + LD_B*vel +LD_C;

	if (result_Lf < 20){
		result_Lf = 20;
	}
	else if(result_Lf > 290){
		result_Lf = 290;
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

int find_smallest_idx_gps(double (*arr)){
	int i, smallest_idx = 0;
	double smallest_val = arr[0];

	for (i = 0; i < gps_len; i++){
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


double PID_control_(double error){ // PID Control for GPS
	double P_gain_, D_gain_, PID_gain_;

	P_gain_ = error;
	I_gain_ += error * dt_;
	D_gain_ = (error - prev_error) / dt_;

	if (I_gain_ > anti){
		I_gain_ = anti;
	}
	else if (I_gain_ < -anti){
		I_gain_ = -anti;
	}

	PID_gain_ = Kp_ * P_gain_ + Ki_ * I_gain_ + Kd_ * D_gain_;
	prev_error = error;

	return PID_gain_;
}


void search_target_index(int& val_1, double& val_2, double (*arr_1), double (*arr_2)){
	// Find shortest point and index for look ahead point
	int i, target_idx, add_to_index, target_idx_of_arr;
	double Lf_, del_x, del_y;
	double int_x_arr[10] = {}, int_y_arr[10] = {};
	double hypot_arr_Lf[31] = {};
	double hypot_int_arr[10] = {};
	Lf_ = update_Lf(global_v);

	if (hlk_val == 2){ // Tracking by Vision
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
	else if (hlk_val == 3){ // for GPS
		for (i = 0; i < gps_len; i++){
			hypot_arr_Lf[i] = (abs(calc_distance(global_x_arr[i], global_y_arr[i]) - Lf_));
		}

		target_idx = find_smallest_idx_gps(hypot_arr_Lf);

		if (calc_distance(global_x_arr[target_idx], global_y_arr[target_idx]) > Lf_){
			// if target point is more far than circle's radius
			add_to_index = target_idx + 1;
			del_x = abs(global_x_arr[target_idx] - global_x_arr[add_to_index]) / 11.0;
			del_y = abs(global_y_arr[target_idx] - global_y_arr[add_to_index]) / 11.0;

			if (global_y_arr[target_idx] > 150){
				for (i = 0; i < 10; i++){
					int_x_arr[i] = global_x_arr[add_to_index] + del_x * (i+1);
					int_y_arr[i] = global_y_arr[add_to_index] - del_y * (i+1);
				}
			}
			else{
				for (i = 0; i< 10; i++){
					int_x_arr[i] = global_x_arr[target_idx] + del_x * (i+1);
					int_y_arr[i] = global_y_arr[target_idx] + del_y * (i+1);
				}
			}
		}
		else{
			// if target point is less far than circle's radius
			add_to_index = target_idx - 1;
			del_x = abs(global_x_arr[target_idx] - global_x_arr[add_to_index]) / 11.0;
			del_y = abs(global_y_arr[target_idx] - global_y_arr[add_to_index]) / 11.0;

			if (global_x_arr[target_idx] > 150){
				for (i = 0; i < 10; i++){
					int_x_arr[i] = global_x_arr[target_idx] + del_x * (i+1);
					int_y_arr[i] = global_y_arr[target_idx] - del_y * (i+1);
				}
			}
			else{
				for (i = 0; i < 10; i++){
					int_x_arr[i] = global_x_arr[add_to_index] + del_x * (i+1);
					int_y_arr[i] = global_y_arr[add_to_index] + del_y * (i+1);
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

	if (hlk_val == 2){ // for vision
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
	else if (hlk_val == 3){
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

		pid_result = PID_control_(delta);
		val_1 = pid_result * 180 / M_PI; // Desired Angle value of Front Wheel
		val_2 = Lf;
		val_3 = target_index;
		for (i = 0; i < 10; i++){
			arr_1[i] = int_x_arr[i];
			arr_2[i] = int_y_arr[i];
		}
	}
}

void checkState(void){
	int nearest_path_idx = 0;
	int nearest_vision_idx = 0;
	double nearest_path_val = abs(global_y_arr[0] - 300);
	double nearest_vision_val = abs(y_coord_arr[0] - 300);
	double vision_del, path_del, vision_path_del;
	int check_val;

	for (int i = 0; i < vision_len; i++){
		vision_del = abs(y_coord_arr[i] - 300);
		if (vision_del < nearest_vision_val){
			nearest_vision_idx = i;
			nearest_vision_val = vision_del;
		}
	}

	for (int i = 0; i < gps_len; i++){
		path_del = abs(global_y_arr[i] - 300);
		if (path_del < nearest_path_val){
			nearest_path_idx = i;
			nearest_path_val = path_del;
		}
	}

	vision_del = abs(x_coord_arr[nearest_vision_idx] - 150);
	path_del = abs(global_x_arr[nearest_path_idx] -150);

	if (abs(vision_del - path_del) < 20){
		check_val = 0; // vision mode
	}
	else{
		if (vision_del <= path_del){
			check_val = 0; // vision mode
			I_gain_ = 0;
		}
		else{
			check_val = 1; // gps mode
			I_gain = 0;
		}
	}

	lane_stab = check_val; // 0 : Stable, 1 : Unstable
}

double update_steer(double cur_steer){
	double virtual_steer, bdy_val = 6.0;

	if (cur_steer - prev_steer > bdy_val){
		virtual_steer = prev_steer + bdy_val;
	}
	else if (prev_steer - cur_steer > bdy_val){
		virtual_steer = prev_steer - bdy_val;
	}
	else{
		virtual_steer = cur_steer;
	}
	error_cnt = 0;
	prev_steer = virtual_steer;
	global_virtual = virtual_steer;

	return virtual_steer;
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
			else if(msg->data[s+1] == 0){
				switch_check = 0;
				break;
			}

		}

	}

	if(switch_check == 0){ // Pure Pursuit Mode
		global_switch = 0;
	}
	else{ // Stanley Mode
		global_switch = 1;
	}
}

void mapCallback(const sensor_msgs::JointState::ConstPtr& msg){
	for(int i = 0; i < gps_len; i++){ // Rotation Transformation
		global_x_arr[i] = origin_x - 10*(cos(-global_yaw)*(msg->position[i] - global_x) - sin(-global_yaw)*(msg->velocity[i] - global_y));
		global_y_arr[i] = origin_y + 10*(sin(-global_yaw)*(msg->position[i] - global_x) + cos(-global_yaw)*(msg->velocity[i] - global_y));
	}
}

void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	global_x = msg->pose.position.x;
	global_y = msg->pose.position.y;
}


void yawCallback(const std_msgs::Float64::ConstPtr& msg){
	if (abs(msg->data) <200){
	global_yaw = (msg->data + 90)*M_PI/180;
	}
	else{}
}


void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
// Update ioniq's velocity
	global_v = msg->velocity[0];
}

void desire_vel_Callback(const std_msgs::Float64::ConstPtr& msg){
	desire_vel = msg->data;
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
	Mat pursuit_img = Mat::zeros(300, 600, CV_8UC3); // Plot Blank View
	for (i = 0; i < vision_len; i++){ // Plot Vision Points
		circle(pursuit_img, Point(x_coord_arr[i], y_coord_arr[i]), 3, Scalar(0, 255, 0), FILLED, LINE_8);
	}

	for (i = 0; i < gps_len; i++){ // Plot GPS Points
		circle(pursuit_img, Point(global_x_arr[i], global_y_arr[i]), 3, Scalar(0, 0, 255), FILLED, LINE_8);
	}
	circle(pursuit_img, Point(150, 300), 5, Scalar(255, 255, 255), FILLED, LINE_8);

	checkState();

	if (hlk_val == 2){ // Tracking for Vision data
		pure_pursuit_steer_control(steer, Lh, tar_idx, int_x_arr, int_y_arr);
		update_v(steer);

		steer_pd = update_steer(steer * 18.0);
		steer_input = (int)round(steer_pd);
		if (abs(steer_input) >=25){
			if (steer_input > 0){steer_input+=2;}
			else{steer_input-=2;}		
		}
		else if (abs(steer_input) <25 && abs(steer_input)>15){
			if (steer_input > 0){steer_input+=1;}
			else{steer_input-=1;}
		}
		I_gain_ = 0;
	}
	else if (hlk_val == 3){ // Tracking for GPS data
		pure_pursuit_steer_control(steer, Lh, tar_idx, int_x_arr, int_y_arr);
		update_v(steer);
		steer_pd = update_steer(steer * 18.0);
		steer_input = (int)round(steer_pd);
	}

	if (hlk_val == 2){ // Tracking Vision
		putText(pursuit_img, "VISION MODE", Point(100, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 2);
		for (i = 0; i < 10; i++){
			circle(pursuit_img, Point(int_x_arr[i], int_y_arr[i]), 3, Scalar(100, 100, 100), FILLED, LINE_8);
		}
		circle(pursuit_img, Point(int_x_arr[tar_idx], int_y_arr[tar_idx]), 5, Scalar(255, 255, 0), FILLED, LINE_8);
	}

	else if (hlk_val == 3){ // Plotting GPS
		putText(pursuit_img, "GPS MODE", Point(100, 50), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255), 2);
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
	line(pursuit_img, Point(origin_x + 300, origin_y), Point(origin_x + 300 + 300*sin(global_virtual*M_PI/180.0), origin_y - 300*cos(global_virtual*M_PI/180.0)), Scalar(0, 255, 0));
	imshow("Result", pursuit_img);
	waitKey(1);
}

void ioniqCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	float auto_switch = msg->data[5];
	float APM_switch = msg->data[6];

	if (auto_switch == 0.0){
		std::cout<<"reset"<<std::endl;
		I_gain = 0;
		speed_input = global_v;
	}
}

void hlkCallback(const std_msgs::Int16::ConstPtr& msg){
	hlk_val = msg->data;
}


void publishing(ros::Publisher cmd_cruise, ros::Publisher cmd_nav, ros::Publisher lane_stab_pub, std_msgs::Float32MultiArray msg1, std_msgs::Int16 msg2, std_msgs::Int16 msg3){
	int i;

	std::vector<float> move_car_data(10, 0.0);
	while(ros::ok()){
		usleep(10000);

		if (hlk_val == 2 || hlk_val == 3){
			move_car_data[0] = 0.0;
			move_car_data[1] = 6.0;
			move_car_data[2] = speed_input + curve_vel;
			move_car_data[5] = steer_input;
			move_car_data[6] = 5.0;
			move_car_data[7] = 100.0;
			msg1.data = move_car_data;
			msg2.data = 0;
			cmd_cruise.publish(msg1);
			cmd_nav.publish(msg2);
		}
		msg3.data = lane_stab;
		lane_stab_pub.publish(msg3);
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
	ros::Subscriber map_sub = n.subscribe("hdmap/path", 1, mapCallback);
	ros::Subscriber utm_sub = n.subscribe("utm_fix", 1, utmCallback);
	ros::Subscriber yaw_sub = n.subscribe("gps_yaw", 1, yawCallback);
	ros::Subscriber vision_sub = n.subscribe("enetsad/centerPt", 1, visionCallback);
	ros::Subscriber mission_sub = n.subscribe("hdmap/switch_info", 1, switchCallback);
	ros::Subscriber desire_vel_sub = n.subscribe("/scc/velocity", 1, desire_vel_Callback);
	ros::Subscriber hlk_sub = n.subscribe("hlk_switch", 1, hlkCallback);

	ros::Publisher cmd_cruise = n.advertise<std_msgs::Float32MultiArray>("/move_car", 1);
	ros::Publisher cmd_nav = n.advertise<std_msgs::Int16>("/mission_manager/nav_pilot", 1);
	ros::Publisher lane_stab_pub = n.advertise<std_msgs::Int16>("lane_stab", 1);

	ros::param::get("~Kp_value", Kp);
	ros::param::get("~Ki_value", Ki);
	ros::param::get("~Kd_value", Kd);
	ros::param::get("~Kp_value_", Kp_);
	ros::param::get("~Ki_value_", Ki_);
	ros::param::get("~Kd_value_", Kd_);
	ros::param::get("~desire_vel", desire_vel);
	ros::param::get("~velocity_term", velocity_term);
	ros::param::get("~min_filter", min_filter);
	//ros::param::get("/pure_pursuit_for_vision/LD_A", LD_A);
	//ros::param::get("/pure_pursuit_for_vision/LD_B", LD_B);
	//ros::param::get("/pure_pursuit_for_vision/LD_C", LD_C);

	std::cout<<"Kp : "<<Kp<<std::endl;
	std::cout<<"Ki : "<<Ki<<std::endl;
	std::cout<<"Kd : "<<Kd<<std::endl;
	std::cout<<"Kp_ : "<<Kp_<<std::endl;
	std::cout<<"Ki_ : "<<Ki_<<std::endl;
	std::cout<<"Kd_ : "<<Kd_<<std::endl;

	std_msgs::Float32MultiArray msg_1;
	std_msgs::Int16 msg_2;
	std_msgs::Int16 msg_3;
	std::cout<<"Starting Threading..."<<std::endl;
	std::thread t1(publishing, cmd_cruise, cmd_nav, lane_stab_pub, msg_1, msg_2, msg_3);
	std::thread t2(subscribing);
	std::cout<<"Please Input Points, Eunbin!!!"<<std::endl;
	t1.join();
	t2.join();
	return 0;
}
