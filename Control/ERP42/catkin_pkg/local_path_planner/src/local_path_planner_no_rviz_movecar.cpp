/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.1.0(ERP42 ver)
 * Be inspired by Python Robotics StateLatticePlanner(https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/StateLatticePlanner)
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For ERP42 version
 *******************************************************/

#include <bits/stdc++.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <tracking_msg/TrackingObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
/*
#include "../matplotlibcpp.h"
#define _USE_MATH_DEFINES
namespace plt = matplotlibcpp;
*/
using namespace std;
using namespace cv;

Mat visualization_map;
vector<double> plot_cost;
const int max_iter = 50;
double h[3] = { 0.5, 0.02, 0.02 };
const double cost_th = 0.15;
const double C1 = 0.1; //map cost constant
const double C2 = 10;
//p is ve0 ,default = ioniq
double L = 2.0;
const double ds = 0.2;
const double PI = acos(-1);
std::string lookup_table_path  = "/home/mj/catkin_ws/src/local_path_planner/src/lookuptable.csv";
double velocity = 15;
double Kp_alpha = 15;
double Kp_beta = -3;
double dt = 0.05;
double d0 = 20;
double line_length = 20;
double Alpha = 14.67/525;
int max_steer = 525;
int warning_count = 0;
int warning_threshold = 10;
int car_type = 0;// 0 : ioniq 1 : ERP42
int cmd_type = 0;// 0 : dbw_cmd 1 : move_car
int obstacle_range = 5; //obstacle range
int obstacle_size = 8; //obstacle Size
int obstacle_set = 0;// = 0 state
int debuging = 1;// 1=debug
ros::Publisher path_array_pub;
ros::Publisher path_pub;
ros::Publisher obstracle_bev_pub;
ros::Publisher line_pub;
ros::Publisher des_pub;
ros::Publisher move_car_pub;
ros::Publisher acc_pub;
ros::Publisher steer_pub;
ros::Publisher break_pub;

struct State {
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;// in obstracel is length
	double v = 0.0;
};
struct Dynamic_Obstacle{
	double xc, yc;
	double v_x, v_y;
	double R;
};
int check_map(Mat map, double x, double y) {
	int xx = round(x * 10);
	int yy = 250 - round(y * 10);
	if (yy < 0 || yy>499 || xx < 0 || xx>499) return 0;
	else {
    return int(map.data[500*yy+xx]);
  }
}
double pi_2_pi(double angle) {
	while (angle < -PI) angle += 2 * PI;
	while (angle > PI) angle -= 2 * PI;
	return angle;
}
State update(State state, double v, double delta, double dt, double L) {
	state.v = v;
  state.x += state.v * cos(state.yaw) * dt;
	state.y += state.v * sin(state.yaw) * dt;
	state.yaw += state.v / L * tan(delta) * dt;
	state.yaw = pi_2_pi(state.yaw);
  return state;
}
void generate_trajectory(double s, double km, double kf, double k0, vector<double>& x, vector<double>& y, vector<double>& yaw, double v){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	double tk[3] = {0.0, time/2.0, time};
	double kk[3] = {k0, km, kf};
	vector<double> kp;
	double invA[2][2] = {{time, -time/2}, {-time*time, time*time/4}};
	double detA = -time*time*time/4;
	double A = (invA[0][0]*(km-k0) + invA[0][1] * (kf-k0)) / detA;
	double B = (invA[1][0]*(km-k0) + invA[1][1] * (kf-k0)) / detA;
	double C = k0;
	for(double t = 0.0; t<time; t += time/n){
		kp.push_back( A*t*t + B*t + C );
	}
	double dt = time/n;
	State state;
	x.push_back(state.x);
	y.push_back(state.y);
	yaw.push_back(state.yaw);
	for(double ikp : kp){
		state = update(state, v, ikp, dt, L);
		x.push_back(state.x);
		y.push_back(state.y);
		yaw.push_back(state.yaw);
	}
}
vector<double> generate_last_state(double s, double km, double kf, double k0, double v) {
	double n = s / ds;
	double time = s / v;
	double cost = 0;
	vector<double> x; vector<double> y; vector<double> yaw;
	double tk[3] = { 0.0, time / 2.0, time };
	double kk[3] = { k0, km, kf };
	vector<double> kp;
	double invA[2][2] = { {time, -time / 2}, {-time * time, time * time / 4} };
	double detA = -time * time * time / 4;
	double A = (invA[0][0] * (km - k0) + invA[0][1] * (kf - k0)) / detA;
	double B = (invA[1][0] * (km - k0) + invA[1][1] * (kf - k0)) / detA;
	double C = k0;
	for (double t = 0.0; t < time; t += time / n) {
		kp.push_back(A * t * t + B * t + C);
	}
	double dt = time / n;
	State state;
	x.push_back(state.x);
	y.push_back(state.y);
	yaw.push_back(state.yaw);
  for (double ikp : kp) {
		state = update(state, v, ikp, dt, L);
		x.push_back(state.x);
		y.push_back(state.y);
		yaw.push_back(state.yaw);
	}
	vector<double> res(3);
	res[0] = state.x;
	res[1] = state.y;
	res[2] = state.yaw;
	return res;
}
vector<double> calc_diff2(State target, double x, double y, double yaw) {
	vector<double> v;
	v.push_back(target.x - x);
	v.push_back(target.y - y);
	v.push_back(target.yaw - yaw);
	return v;
}
void carc_j(State target, double* p, double* h, double k0, double(*j)[3],double v) {
	double xp, xn;
	double yp, yn;
	double yawp, yawn;
	vector<double> dn, dp;
	double d1[3], d2[3], d3[3];
	vector<double> k;
	k = generate_last_state(p[0] + h[0], p[1], p[2], k0,v);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0] - h[0], p[1], p[2], k0,v);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
	for (int i = 0; i < 3; i++) {
		d1[i] = (dp[i] - dn[i]) / (2.0 * h[0]);
	}
	k = generate_last_state(p[0], p[1] + h[1], p[2], k0,v);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1] - h[1], p[2], k0,v);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
	for (int i = 0; i < 3; i++) {
		d2[i] = (dp[i] - dn[i]) / (2.0 * h[1]);
	}
	k = generate_last_state(p[0], p[1], p[2] + h[2], k0,v);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1], p[2] - h[2], k0,v);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
	for (int i = 0; i < 3; i++) {
		d3[i] = (dp[i] - dn[i]) / (2.0 * h[2]);
	}
	for (int i = 0; i < 3; i++) {
		j[i][0] = d1[i];
		j[i][1] = d2[i];
		j[i][2] = d3[i];
	}
	return;
}
double selection_learning_param(double* dp, double* p, double k0, State target, double v) {
	double mincost = 100000000.0;
	double mina = 1.0;
	double maxa = 2.0;
	double da = 0.5;
	double a[2] = { 1.0, 1.5 };
	double tp[3];
	vector<double> b;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			tp[j] = p[j] + a[i] * dp[j];
		}
		b = generate_last_state(tp[0], tp[1], tp[2], k0,v);
		double x = b[0];
		double y = b[1];
		double yaw = b[2];
		vector<double> dc;
		dc = calc_diff2(target, b[0], b[1], b[2]);
		double cost = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[1] * dc[1]);
		if (cost <= mincost && a[i] != 0.0) {
			mina = a[i];
			mincost = cost;
		}
	}
	return mina;
}
int f_inv(double(*a)[3], double(*inv)[3]) {
	double determinant = a[0][0] * a[1][1] * a[2][2] - a[0][0] * a[1][2] * a[2][1]
		+ a[0][1] * a[1][2] * a[2][0] - a[0][1] * a[1][0] * a[2][2]
		+ a[0][2] * a[1][0] * a[2][1] - a[0][2] * a[1][1] * a[2][0];
	if (determinant == 0.0) {
		//cout << "\nNo inverse matrix exists.\n" << endl;
		return 1;
	}
	for (int i = 0; i < 3; i += 1) {
		for (int j = 0; j < 3; j += 1) {
			inv[j][i] = 1.0 / determinant *
				(a[(i + 1) % 3][(j + 1) % 3] * a[(i + 2) % 3][(j + 2) % 3] - a[(i + 1) % 3][(j + 2) % 3] * a[(i + 2) % 3][(j + 1) % 3]);
		}
	}
	return 0;
}
double optimize_trajectory_costmap(State target, double k0, double* p, double* xp, double* yp, double* yawp, double v, Mat &map, vector<Dynamic_Obstacle> obstacles) {
	for (int i = 0; i < max_iter; i++) {
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		generate_trajectory(p[0], p[1], p[2], k0, x, y, yaw, v);
		vector<double> dc;
		dc = calc_diff2(target, x[x.size() - 1], y[y.size() - 1], yaw[yaw.size() - 1]);
		double cost1 = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[2] * dc[2]);
		if (cost1 <= cost_th) {
			double time = p[0]/v;
			double n = p[0]/ds;
			double dt = time/n;
			int cost = 0, c = 0;
			double c2 = 0;
      for(int i=0; i<x.size(); i++){
				c = check_map(map, x[i],y[i]);
				for(int j=0; i<obstacles.size(); i++){
					double d = sqrt((obstacles[j].xc+obstacles[j].v_x*dt*i - x[i])*(obstacles[j].xc+obstacles[j].v_x*dt*i - x[i])+(obstacles[j].yc+obstacles[j].v_y*dt*i - y[i])*(obstacles[i].yc+obstacles[i].v_y*dt*i - y[i]));
					if (d<obstacles[i].R){c2 = 11111;break;}
					else if((d-obstacles[i].R)<0.7){c2+= d-obstacles[i].R;}
				}
				if(c==255 || c2 == 11111){for(int j=0; j<x.size(); j++) circle(visualization_map, Point(x[j]*10, 250 - y[j]*10), 1, Scalar(150), FILLED, LINE_8);
				 	*xp = 0;
				 	return 0;
				}
        cost += C1*c+C2*c2;
      }
      for(int j=0; j<x.size(); j++) circle(visualization_map, Point(x[j]*10, 250 - y[j]*10), 2, Scalar(200), FILLED, LINE_8);
			*xp = x[x.size() - 1];
			*yp = y[y.size() - 1];
			*yawp = yaw[yaw.size() - 1];
			return cost;
		}
		double j[3][3];
		double invJ[3][3], dp[3];
		double* dpp = dp;
		int chekk;
		try {
			carc_j(target, p, h, k0, j,v);
    			chekk = f_inv(j, invJ);
      for(int i=0; i<3; i++){
      }
			if (chekk) { throw chekk; }
			for (int i = 0; i < 3; i++) {
				dp[i] = -invJ[i][0] * dc[0] - invJ[i][1] * dc[1] - invJ[i][2] * dc[2];
			}
		}
		catch (int a) {
			*xp = 0;
			//cout << "cannot calc path LinAlgError : " << k0<<","<<v<<endl;
			return 0;
		}
		double alpha = selection_learning_param(dpp, p, k0, target, v);
		for (int i = 0; i < 3; i++) {
			p[i] += alpha * dp[i];
		}
	}
	*xp = 0;
	//cout << "can't find path in this state : " << target.x<<","<<target.y<<endl;
	return 0;
}
void lidar_on_map(int (*lidar)[4], Mat &map, int type = 0) {
	//cout << "lidar_on_map" << endl;
	//type = 0 ->states,
	int k = obstacle_size;
  vector<Point> contour;
  contour.push_back(Point(lidar[0][0]-k, lidar[1][0]-k));
  contour.push_back(Point(lidar[0][1]+k, lidar[1][1]-k));
  contour.push_back(Point(lidar[0][2]+k, lidar[1][2]+k));
  contour.push_back(Point(lidar[0][3]-k, lidar[1][3]+k));
  const Point *pts1 = (const cv::Point*) Mat(contour).data;
  int npts1 = Mat(contour).rows;
  Scalar a(255);
  fillPoly(map, &pts1, &npts1, 1,Scalar(255,255,255));
  contour.clear();
	//printf("obstacle_range : %d",obstacle_range);
  for(int i=1; i<=obstacle_range; i++){
    vector<Point> contour1;
  	contour1.push_back(Point(lidar[0][0]-i-k, lidar[1][0]-i-k));
  	contour1.push_back(Point(lidar[0][1]+i+k, lidar[1][1]-i-k));
    contour1.push_back(Point(lidar[0][2]+i+k, lidar[1][2]+i+k));
    contour1.push_back(Point(lidar[0][3]-i-k, lidar[1][3]+i+k));
	  const Point *pts1 = (const cv::Point*) Mat(contour1).data;
	  int npts1 = Mat(contour1).rows;
	  polylines(map, &pts1, &npts1, 1, true, Scalar(int(255/i)), 1);
    contour.clear();
  }
}
vector< vector<double> > get_lookup_table() {
	FILE* fpi;
	fpi = fopen(lookup_table_path.c_str(), "r");
	vector< vector<double> > res;
	char tmp[300];
	int cnt = 1;
	for (int i = 0; i < 7; i++) {
		if (feof(fpi)) break;
		fscanf(fpi, "%s", tmp);
	}
	while (1) {
		cnt++;
		double v0, k0, x, y, yaw, km, kf, sf;
		fscanf(fpi, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &v0, &x, &y, &yaw, &sf, &km, &kf);
		if (feof(fpi)) break;
		vector<double> t;
		t.push_back(v0);
		t.push_back(x);
		t.push_back(y);
		t.push_back(yaw);
		t.push_back(sf);
		t.push_back(km);
		t.push_back(kf);
		res.push_back(t);
	}
	fclose(fpi);
	return res;
}
vector<double> search_nearest_one_from_lookuptable(double tx, double ty, double tyaw, vector< vector<double> >& lookup_table, double v) {
	double mind = 1e11;
	int minid = -1;
	int first = 0;
	int k = lookup_table.size();
	int last = k;
  double dv = round(v);
	for (int i = k - 1; i >= 0; i--) {
		if (lookup_table[i][0] == dv) {
      cout<<"check"<<endl;
			last = i + 1;
			for (int j = i; j >= 0; j--) {
				if (lookup_table[j][0] == dv - 1) {
					first = j;
          break;
				}
			}
		}
    break;
	}
	//printf("first last : %d %d %lf\n", first, last, dv);
	for (int i = first; i < last - 1; i++) {
		double dx = tx - lookup_table[i][1];
		double dy = ty - lookup_table[i][2];
		double dyaw = tyaw - lookup_table[i][3];
		double d = sqrt(dx * dx + dy * dy + dyaw * dyaw);
		if (d <= mind) {
			minid = i;
			mind = d;
		}
	}
	return lookup_table[minid];
}
vector< vector<double> > generate_path_costmap(vector< vector<double> >& target_states, double k0, double v, Mat &map,vector< vector<double> > &lookup_table, vector<Dynamic_Obstacle>& obstacles) {
	vector< vector<double> > result;
	int k = target_states.size();
	for (int i = 0; i < k; i++) {
		vector<double> bestp;
		double tx, ty, tyaw, tv;
		tv = target_states[i][0];
		tx = target_states[i][1];
		ty = target_states[i][2];
		tyaw = target_states[i][3];
		bestp = search_nearest_one_from_lookuptable(tx, ty, tyaw, lookup_table, tv);
		State target;
		target.x = tx;
		target.y = ty;
		target.yaw = tyaw;
		target.v = tv;
		double init_p[3] = { sqrt(tx * tx + ty * ty), bestp[5], bestp[6] };
		double x, y, yaw, cost;
		double p[3];
		cost = optimize_trajectory_costmap(target, k0, init_p, &x, &y, &yaw, v/3.6, map, obstacles);// km/h -> m/s check!!
		if (x) {
			vector<double> t;
			t.push_back(x);
			t.push_back(y);
			t.push_back(yaw);
			t.push_back(init_p[0]);
			t.push_back(init_p[1]);
			t.push_back(init_p[2]);
			t.push_back(cost);
			result.push_back(t);
		}
	}
	return result;
}
vector< vector<double> > calc_lane_states(double l_center, double l_heading, double l_width, double d, double v) {
	double dx = sin(l_heading) * l_width/10;
	double dy = cos(l_heading) * l_width/10;
	double y0 = l_center;
	double x0 = d;
	double yawf = l_heading;
	vector< vector<double> > states;
  vector<double> x;
  vector<double> y;
  visualization_msgs::MarkerArray line;
  line.markers.resize(19);
	for (int i = 0; i < 19; i++) {
		double xf = x0 + (4.5-i*0.5)*dx;
		double yf = y0 - (4.5-i*0.5)*dy;
		vector<double> t;
		t.push_back(v);
		t.push_back(xf);
    x.push_back(xf);
    y.push_back(yf);
		t.push_back(yf);
		t.push_back(yawf);
		states.push_back(t);
	}
	return states;
}
void reset_map(Mat& map){
	map = Scalar(0);
}
void show_map(char *msg){
	/*Point2f pc(visualiztion_map.cols/2. , visualiztion_map.rows/2.);
	Mat r = getRotationMatrix2D(pc,90,1.0);
	Mat d;
	warpAffine(visualiztion_map,d,r,visualiztion_map.size());
*/
	putText(visualization_map,msg, Point(0, 50), 1, 1, Scalar(255));
	imshow("visualization_map", visualization_map);
	waitKey(1);
	reset_map(visualization_map);

}
class Planning {
public:

	vector<double> x_last, y_last, yaw_last;
	ros::Subscriber state_sub;
	ros::Subscriber destination_sub;
  ros::Subscriber location_sub;
  ros::Subscriber lidal_sub;
  ros::Subscriber gps_yaw_sub;
	double v{5}, steer{0};
	double x_current = 0;
  double count= 0 ;
  double y_current = 0;
  double yaw_current =  0;
	vector< vector<double> > lookup_table;
	double xd[3], yd[3];
	vector< vector < float> >  lidar;
	vector<double> x_destination;
	vector<double> y_destination;
  Mat local_map = Mat::zeros(500, 500, CV_8UC1);
	vector<Dynamic_Obstacle> dynamic_obstacle;
	Planning(ros::NodeHandle * nodeHandle){
		lookup_table = get_lookup_table();
		reset_map(local_map);
		ROS_INFO("initializing path planner");
		ros::NodeHandle nh(*nodeHandle);
		acc_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Accel",1);
		steer_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Steer", 1);
		break_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Brake", 1);
		move_car_pub = nh.advertise<std_msgs::Float32MultiArray>("/move_car", 1);
    state_sub = nh.subscribe("/ERP42_info", 1, &Planning::state_callback, this);
    gps_yaw_sub = nh.subscribe("/opencr/imu_yaw", 1, &Planning::yaw_callback, this);
    lidal_sub = nh.subscribe("/lidar/tracking_objects", 1, &Planning::lidar_obstracle_callback, this);
    //ros::Subscriber line_sub = nh.subscribe("/", 1, line_callback);
    location_sub = nh.subscribe("/utm_fix", 1, &Planning::location_callback, this);
    destination_sub = nh.subscribe("/hdmap/path",1,&Planning::destination_callback, this);
	}

	void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
		v = msg->data[0];
		v = max(0.5, v);
		steer = msg->data[5];
	}
  void lidar_obstracle_callback(const tracking_msg::TrackingObjectArray& msg) {
    reset_map(local_map);
		dynamic_obstacle.clear();
		dynamic_obstacle.resize(0);
		int size = msg.size;
		int cnt_state = 0;
		int cnt_dynamic = 0;
    //cout<<"get lidar, size : "<<size<<endl;
    visualization_msgs::MarkerArray ma;
		int l[2][4];
		if(obstacle_set == 0){
			for(int i=0; i<size; i++){
				cnt_state+=1;
				double min_x = 1;
				//state obstracle
				for (int j = 0; j < 4; j++) {
					l[0][j] = int(msg.array[i].bev.data[j * 2]*10);
					min_x = min(min_x,double(msg.array[i].bev.data[j * 2]));
					l[1][j] = 250 - int(msg.array[i].bev.data[j * 2 + 1]*10);
				}
				if(min_x<0.5){continue;}
				lidar_on_map(l, local_map, msg.array[i].state);
			}
		}
		else{
			for (int i = 0; i < size; i++) {
				if(msg.array[i].state == 0){
					cnt_state+=1;
					//state obstracle
					for (int j = 0; j < 4; j++) {
						l[0][j] = int(msg.array[i].bev.data[j * 2]*10);
						l[1][j] = 250 - int(msg.array[i].bev.data[j * 2 + 1]*10);
					}
					lidar_on_map(l, local_map, msg.array[i].state);
				}
				else{
					cnt_dynamic +=1;
					Dynamic_Obstacle obstacle;
					obstacle.xc = (msg.array[i].bev.data[0] + msg.array[i].bev.data[2])/2;
					obstacle.yc = (msg.array[i].bev.data[1] + msg.array[i].bev.data[7])/2;
					obstacle.R = sqrt((msg.array[i].bev.data[0] - obstacle.xc)*(msg.array[i].bev.data[0] - obstacle.xc)+(msg.array[i].bev.data[1] - obstacle.yc)*(msg.array[i].bev.data[1] - obstacle.yc));
					obstacle.v_x = msg.array[i].velocity.x;
					obstacle.v_y = msg.array[i].velocity.y;
					dynamic_obstacle.push_back(obstacle);
				}
			}
		}
		char str[100];
		sprintf(str, "Obstacles state :%d, dynamic : %d", cnt_state, cnt_dynamic);
		putText(local_map, str, Point(200,10), 1, 1, Scalar(255));
		if(debuging){
			imshow("local_map update", local_map);
			waitKey(1);
		}
	}
  void yaw_callback(const std_msgs::Float64::ConstPtr& msg){
    //cout<<"yaw callback"<<endl;
    yaw_current = -msg->data;
  }
	void location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)	{
  //  cout<<"get current location"<<endl;
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
	}
	void move_car(double x, double y, double yaw, double yaw_init, int type){
		double alpha = pi_2_pi(atan(y/x) - yaw_init);
		double beta = pi_2_pi(yaw - yaw_init -  alpha);
		double w = (Kp_alpha * alpha + Kp_beta*beta)*dt;
		int steer =  -max(-max_steer, min(max_steer, int(w * 180/PI/Alpha)));
		//printf("%d %d %lf\n",max_steer, steer,double(abs(steer))/max_steer);
		int acc = max(0,int(double(velocity) - double(abs(steer))/max_steer*velocity/2.0));
		std_msgs::Int16 acc_;
		std_msgs::Int16 steer_;
		std_msgs::Int16 break_;
		acc_.data =  acc*10;
		steer_.data = steer;


	  char str[100], str1[100];
		sprintf(str, "acc : %d, steer : %d, alpha : %lf, beta : %lf", acc, steer,alpha,beta);
		sprintf(str1, "x:%lf, y:%lf, yaw_init:%lf, yaw:%lf", x,y,yaw_init,yaw);
		putText(visualization_map, str, Point(0, 400), 1, 1, Scalar(255));
		putText(visualization_map, str1, Point(0, 450), 1, 1, Scalar(255));
/*
		acc_pub.publish(acc_);
		steer_pub.publish(steer_);
		*/
		if(car_type == 0){
			if(cmd_type == 1){
				if(type == 0){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 0; //car_type
					move.data[1] = 2; //mode
					move.data[2] = acc; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = steer;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
				}
				else if(type == 1){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 0; //car_type
					move.data[1] = 2; //mode
					move.data[2] = 0; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = steer;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
				}
				else if(type == 2){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 0; //car_type
					move.data[1] = 0; //mode
					move.data[2] = 0; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = 0;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
					ROS_INFO("Estop!!");
				}
			}
			else{
				if (type == 0){
					break_.data = 0;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
				}
				else if(type == 1){
					acc_.data = 0;
					break_.data = 0;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
				}
				else if(type == 2){
					acc_.data = 0;
					steer_.data = 0;
					break_.data = 200;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
					ROS_INFO("ESTOP");
				}
			}
		}
		else{
			if(cmd_type == 1){
				if(type == 0){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 1; //car_type
					move.data[1] = 6; //mode
					move.data[2] = acc; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = steer;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
				}
				else if(type == 1){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 1; //car_type
					move.data[1] = 6; //mode
					move.data[2] = 0; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = steer;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
				}
				else if(type == 2){
					std_msgs::Float32MultiArray move;
					move.data.resize(10);
					move.data[0] = 1; //car_type
					move.data[1] = 6; //mode
					move.data[2] = 0; // desired_speed
					move.data[3] = 0;
					move.data[4] = 0;
					move.data[5] = 0;
					move.data[6] = 0;
					move.data[7] = 0;
					move.data[8] = 0;
					move.data[9] = 0;
					move_car_pub.publish(move);
					ROS_INFO("Estop!!");
				}
			}
			else{
				if (type == 0){
					break_.data = 0;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
				}
				else if(type == 1){
					acc_.data = 0;
					break_.data = 0;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
				}
				else if(type == 2){
					acc_.data = 0;
					steer_.data = 0;
					break_.data = 200;
					acc_pub.publish(acc_);
					steer_pub.publish(steer_);
					break_pub.publish(break_);
					ROS_INFO("ESTOP");
				}
			}
		}
	}
	void destination_callback(const sensor_msgs::JointState::ConstPtr& msg)	{
	    visualization_map = local_map.clone();
	    clock_t start = clock();
		float d[3][2];
	    double dd[6];
	    int size = msg->position.size();
	    int l = size;
	    vector< vector<double> > dest;
		//global path plotting

		for(int i = 0; i< size; i++){
			vector<double> t;
			t.push_back(cos(yaw_current*PI/180)*(msg->position[i]-x_current)-sin(yaw_current*PI/180)*(msg->velocity[i]-y_current));
		 	t.push_back(sin(yaw_current*PI/180)*(msg->position[i]-x_current)+cos(yaw_current*PI/180)*(msg->velocity[i]-y_current));
		 	dest.push_back(t);

		}
		for(int i = 0; i < size; i++){
			if((sqrt(dest[i][0]*dest[i][0] + dest[i][1]*dest[i][1])>d0) && dest[i][0] >= 0){
		 		l = i;
		 		break;
		 	}
		}
		if(debuging){
			Mat path_debug = Mat::zeros(1000, 1000, CV_8UC1);
			char b[100];
			sprintf(b,"Path size = %d", l);
			putText(path_debug, b, Point(500,10), 1, 1, Scalar(255));
			for(int i=0; i<100; i++){
				circle(path_debug, Point(i*10, 500),1, Scalar(255), 1);
				circle(path_debug, Point(500, i*10),1, Scalar(255), 1);
			}
			for(int i=0; i<size; i++){
				int intensity = 120+ int(double(i)/size*125);
				circle(path_debug, Point(int(500 + dest[i][0]*10), int(500 - dest[i][1]*10)), 3, Scalar(intensity), 1);
			}
			imshow("debug path", path_debug);
			waitKey(1);
		}
		if(l>=size-3){
			cout << "Rogue input data size " << endl;
			warning_count = warning_threshold;
			move_car(1,1,1,1,2);
			show_map("Rogue input data size");
			return;
		}
		for(int i = 0; i<3; i++){
			d[i][0] = dest[l+i][0];
			d[i][1] = dest[i+l][1];
		}
		for(int i=0; i<3; i++){
			circle(visualization_map, Point(d[i][0]*10, 250 - d[i][1]*10), 3, Scalar(255, 255, 255), 1);
		}
		double arr[3][3] = {
			{d[0][0]*d[0][0]*d[0][0],d[0][0]*d[0][0],d[0][0]},
			{d[1][0]*d[1][0]*d[1][0],d[1][0]*d[1][0],d[1][0]},
			{d[2][0]*d[2][0]*d[2][0],d[2][0]*d[2][0],d[2][0]}
		};
		double inv[3][3];
		int check = f_inv(arr,inv);
		double A = inv[0][0]*d[0][1]+inv[0][1]*d[1][1]+inv[0][2]*d[2][1];
		double B = inv[1][0]*d[0][1]+inv[1][1]*d[1][1]+inv[1][2]*d[2][1];
		double C = inv[2][0]*d[0][1]+inv[2][1]*d[1][1]+inv[2][2]*d[2][1];
		if(check){
			warning_count = warning_threshold;
			printf("Along Global node");
			move_car(1,1,1,1,2);
			show_map("Along Global node");
			return;
		}
		char a[100];
		sprintf(a, "d0 : %lf, id : %d", d0, l);
		putText(visualization_map, a, Point(0, 150), 1, 1, Scalar(255));
		//main callback do path planning
		// here will set destination
		double k0 = 0; //check this value, this value is for ERP42

		double l_center = max(-25.0, min(25.0, A*d0*d0*d0+B*d0*d0+C*d0));
		double l_heading = max(-PI/2, min(PI/2, atan(3*A*d0*d0+2*B*d0+C)));
		double l_width = line_length;

		double cost=0;
		vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width,  d0, v);
		vector< vector<double> > result = generate_path_costmap(states, k0,v, local_map, lookup_table, dynamic_obstacle);
		double mind = 1e9;
		vector <double> vm;
	//	printf("result size : %ld\n", result.size());
    if(result.size()==0){
      cout<<"no path generated"<<endl;
			if (count == 0){return;}
			int k = x_last.size();
			for(int i=0; i<k; i++){
				circle(visualization_map, Point(x_last[i]*10, 250 - y_last[i]*10), 1, Scalar(255, 255, 255), -1);
			}
			warning_count= min(10000000, warning_count+1);
			char str[100];
			if(warning_count<warning_threshold){
				sprintf(str,  "warning!! this path will have a problem : %d %%", double(warning_count)*100.0/warning_threshold);
				move_car(x_last[x_last.size()-1], y_last[y_last.size()-1], yaw_last[yaw_last.size()-1], k0, 1);
			}
			else {
				sprintf(str, "dangerous!! this path will incorrect");
				move_car(x_last[x_last.size()-1], y_last[y_last.size()-1], yaw_last[yaw_last.size()-1], k0, 2);
			}
			show_map(str);
      return;
    }
    double des_x = d0;
    double des_y =max(-25.0, min(25.0, A*d0*d0*d0+B*d0*d0+C*d0));
    //cout<<des_x<<" , "<<des_y<<endl;
		for (int i = 0; i < result.size(); i++) {
			vector<double> table = result[i];
			double k = sqrt((table[0]-des_x)*(table[0]-des_x)+(table[1]-des_y)*(table[1]-des_y)) + table[6];// lencth + trajectory cost
			if(k<mind){
				mind = k;
				vm = table;
			}
		}
    vector<double> xt;
		vector<double> yt;
		vector<double> yawt;
		generate_trajectory(vm[3], vm[4], vm[5], k0,xt,yt,yawt,v/3.6);
		for(int i=0; i<xt.size(); i++){circle(visualization_map, Point(xt[i]*10, 250 - yt[i]*10), 3, Scalar(255, 255, 255), -1);}
		char str[100];
		if(warning_count>=warning_threshold){
			std_msgs::Float32MultiArray move;
			move.data.resize(1);
			move.data[0] = 119; //reset move_car
			move_car_pub.publish(move);
		}
    	clock_t end =clock();
		sprintf(str, "find good path, result size : %ld , %lf",result.size(), (double)(end - start)/CLOCKS_PER_SEC);
		move_car(xt[xt.size()-1], yt[yt.size()-1], yawt[yawt.size()-1], k0, 0);
		show_map(str);
		x_last = xt;
		y_last = yt;
		yaw_last = yawt;
		warning_count = 0;
		count=1;
	}
};
int main(int argc, char* argv[]) {
	cout<<"Run Local_path"<<endl;
	ros::init(argc, argv, "local_path_planner");
	ros::NodeHandle nh;
	ros::param::get("~velocity", velocity);
	ros::param::get("~Kp_alpha", Kp_alpha);
	ros::param::get("~Kp_beta", Kp_beta);
	ros::param::get("~dt", dt);
	ros::param::get("~distance", d0);
	ros::param::get("~line_length", line_length);
	ros::param::get("~car_type", car_type);
	ros::param::get("~cmd_type", cmd_type);
	ros::param::get("~warning_threshold", warning_threshold);
	ros::param::get("~lookup_table_path", lookup_table_path);
	ros::param::get("~obstacle_size", obstacle_size);
	ros::param::get("~obstacle_range", obstacle_range);
	ros::param::get("~obstacle_set", obstacle_set);
	ros::param::get("~debug_path", debuging);
	ROS_INFO("velocity : %lf", velocity);
	ROS_INFO("Kp_alpha : %lf Kp_beta : %lf dt : %lf", Kp_alpha, Kp_beta, dt);
	ROS_INFO("distance : %lf line_length : %lf warning_threshold : %d Debuging : %d", d0,line_length, warning_threshold, debuging);
	ROS_INFO("%s", lookup_table_path.c_str());
	ROS_INFO("Obstacle info Size: %d Range : %d set : %d", obstacle_size, obstacle_range, obstacle_set);
	if(car_type == 0){
		ROS_INFO("Car type : Ioniq");
		Alpha = 14.67/525;
		L = 2.0;
		max_steer = 540;
	}
	else{
		ROS_INFO("Car type : ERP42");
		Alpha = 27.0/2000;
		L = 1.0;
		max_steer = 2000;
	}
	if(cmd_type == 0){
		ROS_INFO("Cmd type : dbw_cmd");
	}
	else{
		ROS_INFO("Cmd type : move_car");
	}
	Planning plan(&nh);
  ros::spin();
	return 0;
}
