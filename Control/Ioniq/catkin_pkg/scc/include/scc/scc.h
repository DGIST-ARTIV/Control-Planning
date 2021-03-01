#pragma once
#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

#include "tracking_msg/TrackingObjectArray.h"

#include "geometry_msgs/Point.h"


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "scc/logger.h"

#define PI 3.141592
using namespace std;
using namespace cv;
using namespace std::chrono;
 

void reset_map(Mat& map);
struct State {
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;// in obstracel is length
	double v = 0.0;
};
struct Obstacle{
	double x{0}, y{0};
	double v_x{0}, v_y{0};
	double R{0};
	double length{0};
	int state=0;
};

double pi_2_pi(double angle);
State update(State state, double v, double delta, double dt, double L);
Point upper_points(double x, double y, double yaw, double length);
Point lower_points(double x, double y, double yaw, double length);
class SCC{
public:
	ros::Publisher movecar_pub, closest_distance_pub, Estop_pub, velocity_pub;
	ros::Publisher line_pub, closest_object_pub;

	vector<Obstacle> obstacles;
	Obstacle closest_object;
	high_resolution_clock::time_point now_time = high_resolution_clock::now();
	high_resolution_clock::time_point prev_time = now_time;
	Mat line_map = Mat::zeros(500, 500, CV_8UC1);
	Mat obstacle_map = Mat::zeros(500, 500, CV_8UC1);
	Mat debug_map = Mat::zeros(500, 500, CV_8UC3);
	double hz = 100;
	double closest_distance = 10000;
	double desire_velocity = 30/3.6;
	double max_velocity = 100;
	double now_velocity{0}, now_steer{0};
	double max_accelation = 100/3.6/9.1;
	double scale = 3;
	double dt = 1/hz;//sampling_time;
	double filter_velocity = desire_velocity / hz;
	double threthold = 10;
	double arrival_time = 3;
	double wheel_base = 1.2;
	logger LOG;

	double min_distance = 10;
	double upper_obstacle_filter = 5;
	double lower_obstacle_filter = 1;
	bool Debug = true;
	SCC(ros::NodeHandle *nodehandle);
	void publishing();
	void draw_line();
	void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void object_callback(const tracking_msg::TrackingObjectArray& msg);
	void velocity_callback(const std_msgs::Float64::ConstPtr& msg);
	void max_velocity_callback(const std_msgs::Int16::ConstPtr& msg);
	void AEB();
	void move_car(double velocity);
	void move_car_data(double velocity, bool flag);
};