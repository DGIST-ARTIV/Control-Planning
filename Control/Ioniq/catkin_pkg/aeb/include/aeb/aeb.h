#include "ros/ros.h"
#include <bits/stdc++.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

#include "tracking_msg/TrackingObjectArray.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;

struct object{
	int state = 0;

	double x{0}, y{0};
	double size{0};
	double v_x{0}, v_y{0};
	vector<Point> bev;
	int id{0};
};

void draw_object(Mat &map, vector<Point> &p, int id);
int check_map(Mat &map, double x, double y,double scale);
class AEB{
public:
	double now_velocity{0};
	double min_distance{3};
	double wheel_base = 2;

	bool flag = false;
	double scale = 10;
	double filter=  7;
	ros::Publisher Estop_pub;
	Mat obstacle_map = Mat::zeros(500, 500, CV_8UC1);
	Mat debug_map = Mat::zeros(500, 500, CV_8UC1);
	vector<object> obstacles;

	AEB(ros::NodeHandle *nodehandle);
	void publishing();
	void state_callback(const std_msgs::Float32MultiArray::ConstPtr &msg);
	void object_callback(const tracking_msg::TrackingObjectArray &msg);
	void bracking();
};