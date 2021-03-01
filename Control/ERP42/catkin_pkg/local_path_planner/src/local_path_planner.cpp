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
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <tracking_msg/TrackingObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//#include"matplotlib-cpp-master/matplotlibcpp.h"
//namespace plt = matplotlibcpp;
using namespace std;
using namespace cv;
Mat visualiztion_map;

const int max_iter = 50;
double h[3] = { 0.5, 0.02, 0.02 };
const double cost_th = 0.15;
const double con = 0.1; //map cost constant
//p is ve0
const double L = 1.0;
const double ds = 0.2;
const double PI = acos(-1);
ros::Publisher path_array_pub;
ros::Publisher path_pub;
ros::Publisher obstracle_bev_pub;
ros::Publisher line_pub;
ros::Publisher des_pub;
ros::Publisher l_pub;
vector<double> x_last, y_last;
int warning_count = 0;
struct State {
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double v = 0.0;
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
		cout << "\nNo inverse matrix exists.\n" << endl;
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
double optimize_trajectory_costmap(State target, double k0, double* p, double* xp, double* yp, double* yawp, double v, Mat &map) {
	for (int i = 0; i < max_iter; i++) {
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		generate_trajectory(p[0], p[1], p[2], k0, x, y, yaw, v);
		vector<double> dc;
		dc = calc_diff2(target, x[x.size() - 1], y[y.size() - 1], yaw[yaw.size() - 1]);
		double cost1 = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[2] * dc[2]);
		if (cost1 <= cost_th) {
      int cost = 0, c = 0;
      for(int i=0; i<x.size(); i++){
        c = check_map(map, x[i],y[i]);
        if(c==255){
          for(int j=0; j<x.size(); j++){
            circle(visualiztion_map, Point(x[j]*10, 250 - y[j]*10), 1, Scalar(150), FILLED, LINE_8);
          }
          *xp = 0;
          return 0;
        }
        cost += c;
      }
      for(int j=0; j<x.size(); j++){
        circle(visualiztion_map, Point(x[j]*10, 250 - y[j]*10), 2, Scalar(200), FILLED, LINE_8);
      }
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
			cout << "cannot calc path LinAlgError : " << k0<<","<<v<<endl;
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
int k_min(double a, double b) {
	if (a >= b) return int(b);
	else return int(a);
}
int k_max(double a, double b) {
	if (a <= b) return int(b);
	else return int(a);
}
/*void line_on_map(double(*line)[3], double(*map)[500], int type = 0) {
	//type 1 -> break line
	//type 0 -> solid line
	double c = 0.1; //break line cost
	cout << "line_on_map" << endl;
	double A, B, C;
	double inv[3][3];
	double a[3][3] = {
		{line[0][0] * line[0][0], line[0][0], 1.0},
		{line[0][1] * line[0][1], line[0][1], 1.0},
		{line[0][2] * line[0][2], line[0][2], 1.0}
	};
	if (f_inv(a, inv) == 1) {
		printf("interpolation fail\n");
		return;
	}
	A = inv[0][0] * line[1][0] + inv[0][1] * line[1][1] + inv[0][2] * line[1][2];
	B = inv[1][0] * line[1][0] + inv[1][1] * line[1][1] + inv[1][2] * line[1][2];
	C = inv[2][0] * line[1][0] + inv[2][1] * line[1][1] + inv[2][2] * line[1][2];
	double k1 = line[0][0];
	double k2 = line[0][2];
	if (type == 0) {
		for (int x = k_max(k1, 0); x < k_min(k2, 500); x++) {
			double tmp = A * x * x + B * x + C;
			int y = 250 - int(tmp);
			if (y < 0 || y>499) {
				continue;
			}
			map[y][x] = 1;
			map[min(y + 1, 499)][x] = 1; 	map[min(y + 2, 499)][x] = 1;	map[min(y + 3, 499)][x] = 1;	map[min(y + 4, 499)][x] = 1;	map[min(y + 5, 499)][x] = 1;
			map[max(y - 1, 0)][x] = 1;	map[max(y - 2, 0)][x] = 1;	map[max(y - 3, 0)][x] = 1;	map[max(y - 4, 0)][x] = 1;	map[max(y - 5, 0)][x] = 1;
		}
	}
	else if (type == 1) {
		for (int x = k_max(k1, 0); x < k_min(k2, 500); x++) {
			double tmp = A * x * x + B * x + C;
			int y = 250 - int(tmp);
			if (y < 0 || y>499) {
				continue;
			}
			map[y][x] = c;
			map[min(y + 1, 499)][x] = c; 	map[min(y + 2, 499)][x] = c;	map[min(y + 3, 499)][x] = c;	map[min(y + 4, 499)][x] = c;	map[min(y + 5, 499)][x] = c;
			map[max(y - 1, 0)][x] = c;	map[max(y - 2, 0)][x] = c;	map[max(y - 3, 0)][x] = c;	map[max(y - 4, 0)][x] = c;	map[max(y - 5, 0)][x] = c;
		}
	}
}*/
void lidar_on_map(int (*lidar)[4], Mat &map, int type = 0) {
	//cout << "lidar_on_map" << endl;
  vector<Point> contour;
  contour.push_back(Point(lidar[0][0]-5, lidar[1][0]-5));
  contour.push_back(Point(lidar[0][1]+5, lidar[1][1]-5));
  contour.push_back(Point(lidar[0][2]+5, lidar[1][2]+5));
  contour.push_back(Point(lidar[0][3]-5, lidar[1][3]+5));
  const Point *pts1 = (const cv::Point*) Mat(contour).data;
  int npts1 = Mat(contour).rows;
  Scalar a(255);
  fillPoly(map, &pts1, &npts1, 1,Scalar(255,255,255));
  contour.clear();
  for(int i=1; i<=5; i++){
    vector<Point> contour;
  	contour.push_back(Point(lidar[0][0]-i-5, lidar[1][0]-i-5));
  	contour.push_back(Point(lidar[0][1]+i+5, lidar[1][1]-i-5));
    contour.push_back(Point(lidar[0][2]+i+5, lidar[1][2]+i+5));
    contour.push_back(Point(lidar[0][3]-i-5, lidar[1][3]+i+5));
	  const Point *pts1 = (const cv::Point*) Mat(contour).data;
	  int npts1 = Mat(contour).rows;
	  polylines(map, &pts1, &npts1, 1, true, Scalar(int(120/i)), 1);
    contour.clear();
  }
}
vector< vector<double> > get_lookup_table() {
	FILE* fpi;
	fpi = fopen("/home/mj/catkin_ws/src/local_path_planner/src/lookuptable.csv", "r");
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
vector< vector<double> > generate_path_costmap(vector< vector<double> >& target_states, double k0, double v, Mat &map,vector< vector<double> > &lookup_table) {
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
		cost = optimize_trajectory_costmap(target, k0, init_p, &x, &y, &yaw, v/3.6, map);// km/h -> m/s check!!
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
    line.markers[i].header.frame_id = "velodyne";
    line.markers[i].header.stamp = ros::Time();
    line.markers[i].id = i;
    line.markers[i].type = visualization_msgs::Marker::ARROW;
    line.markers[i].action = visualization_msgs::Marker::ADD;
    line.markers[i].pose.position.x = xf;
    line.markers[i].pose.position.y = yf;
    line.markers[i].pose.position.z = 0;
    line.markers[i].pose.orientation.x = 0.0;
    line.markers[i].pose.orientation.y = 0.0;
    line.markers[i].pose.orientation.z = yawf;
    line.markers[i].pose.orientation.w = 1.0;
    line.markers[i].scale.x = 1;
    line.markers[i].scale.y = 0.1;
    line.markers[i].scale.z = 0.1;
    line.markers[i].color.a = 1.0;
    line.markers[i].color.r = 1.0f;
    line.markers[i].color.g = 1.0f;
    line.markers[i].color.b = 1.0f;
		t.push_back(v);
		t.push_back(xf);
    x.push_back(xf);
    y.push_back(yf);
		t.push_back(yf);
		t.push_back(yawf);
		states.push_back(t);
	}
  line_pub.publish(line);
	return states;
}
void reset_map(Mat& map){
	map = Scalar(0);
}
void show_map(char *msg){
	Point2f pc(visualiztion_map.cols/2. , visualiztion_map.rows/2.);
	Mat r = getRotationMatrix2D(pc,90,1.0);
	Mat d;
	warpAffine(visualiztion_map,d,r,visualiztion_map.size());
	putText(d,msg, Point(0, 50), 1, 1, Scalar(255));
	imshow("visualiztion_map", d);
	waitKey(1);
	reset_map(visualiztion_map);
}
class Planning {
public:
	ros::Subscriber state_sub;
	ros::Subscriber destination_sub;
  ros::Subscriber location_sub;
  ros::Subscriber lidal_sub;
  ros::Subscriber gps_yaw_sub;
	double v{5}, steer{0};
	double x_current = 450610.2000564152;
  double y_current = 3951724.661419577;
  double yaw_current =  -13.81;
	vector< vector<double> > lookup_table;
	double xd[3], yd[3];
	vector< vector < float> >  lidar;
  Mat local_map = Mat::zeros(500, 500, CV_8UC1);

	Planning(ros::NodeHandle * nodeHandle){
		lookup_table = get_lookup_table();
		reset_map(local_map);
		ros::NodeHandle nh(*nodeHandle);
		path_array_pub = nh.advertise<std_msgs::Float64MultiArray>("/local_path", 1);
    path_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_path", 1);
    line_pub = nh.advertise<visualization_msgs::MarkerArray>("line", 1);
    des_pub = nh.advertise<visualization_msgs::Marker>("des", 1);
    l_pub = nh.advertise<visualization_msgs::Marker>("l",1);

    state_sub = nh.subscribe("/Ioniq_info", 1, &Planning::state_callback, this);
    gps_yaw_sub = nh.subscribe("/gps_yaw", 1, &Planning::yaw_callback, this);
    lidal_sub = nh.subscribe("/lidar/tracking_objects", 1, &Planning::lidar_obstracle_callback, this);
    //ros::Subscriber line_sub = nh.subscribe("/", 1, line_callback);
    location_sub = nh.subscribe("/utm_fix", 1, &Planning::location_callback, this);
    destination_sub = nh.subscribe("/destination_info",1,&Planning::destination_callback, this);
    /*thread t1(Planning::subscribing, &nh);
    thread t2(Planning::destination_sub, &nh);
    t1.join();
    t2.join();*/
	}
 /*void subscribing(ros::NodeHandle * nodeHandle){
    ros::NodeHandle nh(*nodeHandle);
    state_sub = nh.subscribe("/Ioniq_info", 1, &Planning::state_callback, this);
    gps_yaw_sub = nh.subscribe("/gps_yaw", 1, &Planning::yaw_callback, this);
    lidal_sub = nh.subscribe("/lidar/tracking_objects", 1, &Planning::lidar_obstracle_callback, this);
    //ros::Subscriber line_sub = nh.subscribe("/", 1, line_callback);
    location_sub = nh.subscribe("/utm_fix", 1, &Planning::location_callback, this);
    ros::spin();
  }
  void destination_sub(ros::NodeHandle *nodeHandle){
    ros::NodeHandle nh(*nodeHandle);
    destination_sub = nh.subscribe("/destination_info",1,&Planning::destination_callback, this);
    ros::spin();
  }*/
	void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
		//cout<<"state_callback!!!"<<endl;
		v = msg->data[23];
		steer = msg->data[3];
  //cout<<"!!!!!!!!!!!!!!!!!!"<<v<<", "<<steer<<endl;
	}
  void lidar_obstracle_callback(const tracking_msg::TrackingObjectArray& msg) {
    reset_map(local_map);
		int size = msg.size;
    //cout<<"get lidar, size : "<<size<<endl;
    visualization_msgs::MarkerArray ma;
		int l[2][4];
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < 4; j++) {
				l[0][j] = int(msg.array[i].bev.data[j * 2]*10);
				l[1][j] = 250 - int(msg.array[i].bev.data[j * 2 + 1]*10);
			}
			lidar_on_map(l, local_map, msg.array[i].state);
		}
    //imshow("local_map update", local_map);
    //waitKey(1);
	}
  void yaw_callback(const std_msgs::Float64::ConstPtr& msg){
    cout<<"yaw callback"<<endl;
    yaw_current = -msg->data;
  }
	void location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)	{
    //cout<<"get current location"<<endl;
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    cout<<x_current<<","<<y_current<<endl;
	}
	void destination_callback(const std_msgs::Float32MultiArray& msg)	{
    visualiztion_map = local_map.clone();
    clock_t start = clock();
		float d[3][2];
    double dd[6];
    int cnt = 0;
		FILE* fpi;
		fpi = fopen("/home/mj/Desktop/user_points.csv", "r");
		vector< vector<double> > res;
		char tmp[300];
		for (int i = 0; i < 3; i++) {
			if (feof(fpi)) break;
			fscanf(fpi, "%lf,%lf", &dd[i*2],&dd[i*2+1]);
		}

/*
    for(int i=0; i<2; i++){

        d[i][0] = cos(yaw_current*PI/180)*(msg.data[i*2]-x_current)-sin(yaw_current*PI/180)*(msg.data[i*2+1]-y_current);
        d[i][1] = sin(yaw_current*PI/180)*(msg.data[i*2]-x_current)+cos(yaw_current*PI/180)*(msg.data[i*2+1]-y_current);

    }//need to update more accurate
*/
/*
    d[0][0] = cos(yaw_current*PI/180)*(dd[0]-x_current)-sin(yaw_current*PI/180)*(dd[1]-y_current);
    d[0][1] = sin(yaw_current*PI/180)*(dd[0]-x_current)+cos(yaw_current*PI/180)*(dd[1]-y_current);
    d[1][0] = cos(yaw_current*PI/180)*(dd[2]-x_current)-sin(yaw_current*PI/180)*(dd[3]-y_current);
    d[1][1] = sin(yaw_current*PI/180)*(dd[2]-x_current)+cos(yaw_current*PI/180)*(dd[3]-y_current);
		d[2][0] = cos(yaw_current*PI/180)*(dd[4]-x_current)-sin(yaw_current*PI/180)*(dd[5]-y_current);
		d[2][1] = sin(yaw_current*PI/180)*(dd[4]-x_current)+cos(yaw_current*PI/180)*(dd[5]-y_current);
*/

    d[0][0] = 7;
    d[0][1] = 0;
    d[1][0] = 40;
    d[1][1] = 0;
		d[2][0] = 60;
		d[2][1] = 0;

    cout<<"1 : "<<dd[0]<<","<<dd[1]<<endl;
    cout<<"2 : "<<dd[3]<<","<<dd[4]<<endl;
 visualization_msgs::MarkerArray path;


		std_msgs::Float64MultiArray path_array;
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
			printf("Along Global node");
			return;
		}
		//main callback do path planning
		// here will set destination
		//double k0 = steer/525*36*PI/180; //check this value, this value is for ERP42
		double k0 = steer/100;//for ERP42
		double d0 = 3;
		double l_center =max(-25.0, min(25.0, A*d0*d0*d0+B*d0*d0+C*d0));
		double l_heading = atan(3*A*d0*d0+2*B*d0+C);
		double l_width = 5.0;
		double v_width = 1.16;

		double cost=0;
		// this need to ask, can private variable use in private function?? can!!!
		vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width,  d0, v);
		vector< vector<double> > result = generate_path_costmap(states, k0,v, local_map, lookup_table);
		double mind = 1e9;
		vector <double> vm;
	//	printf("result size : %ld\n", result.size());
    if(result.size()==0){
      cout<<"no path generated"<<endl;
			int k = x_last.size();
			for(int i=0; i<k; i++){
				circle(visualiztion_map, Point(x_last[i]*10, 250 - y_last[i]*10), 1, Scalar(255, 255, 255), -1);
				path_array.data.push_back(x_last[i]);
			}
			for(int i=0; i<k; i++){
				path_array.data.push_back(y_last[i]);
			}
      path_array_pub.publish(path_array);
			warning_count+=1;
			char str[100];
			if(warning_count<50){sprintf(str,  "warning!! this path will have a problem : %d %%", warning_count);}
			else sprintf(str, "dangerous!! this path will incorrect");
			show_map(str);
      return;
    }
    double des_x = d0;
    double des_y = A*d0*d0*d0 + B*d0*d0+C*d0;
    //cout<<des_x<<" , "<<des_y<<endl;
		for (int i = 0; i < result.size(); i++) {
			vector<double> table = result[i];
			double k = sqrt((table[0]-des_x)*(table[0]-des_x)+(table[1]-des_y)*(table[1]-des_y)) + table[6]* con;// lencth + trajectory cost
			if(k<mind){
				mind = k;
				vm = table;
			}
		}
    vector<double> xt;
		vector<double> yt;
		vector<double> yawt;
    visualization_msgs::Marker dest, l;
    l.type = visualization_msgs::Marker::POINTS;
    l.scale.x = 5;
    l.color.r = 0.0f;
    l.color.g = 0.0f;
    l.color.b = 1.0f;
    l.color.a = 1.0;
    l.id = 10;
		generate_trajectory(vm[3], vm[4], vm[5], k0,xt,yt,yawt,v/3.6);
    l.header.frame_id = dest.header.frame_id = "velodyne";
    l.header.stamp = dest.header.stamp = ros::Time();
    dest.id = 1;
    l.ns = dest.ns = "basic_shapes";
    dest.type = visualization_msgs::Marker::ARROW;
    l.action = dest.action = visualization_msgs::Marker::ADD;
    dest.pose.position.x = des_x;
    dest.pose.position.y = des_y;
    dest.pose.position.z = 1;
    dest.pose.orientation.x = 0;
    dest.pose.orientation.y = 0;
    dest.pose.orientation.z = l_heading;
    l.pose.orientation.w = dest.pose.orientation.w = 1.0;
    dest.scale.x = 1;
    dest.scale.y = 0.1;
    dest.scale.z = 0.1;
    dest.color.a = 1.0;
    dest.color.r = 0.0f;
    dest.color.g = 1.0f;
    dest.color.b = 0.0f;
    des_pub.publish(dest);
    for(int i=0; i<2; i++){
        geometry_msgs::Point p;
        p.x = d[i][0];
        p.y = d[i][1];
        p.z = 1;
        l.points.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 1;
    l.points.push_back(p);
    l_pub.publish(l);
    path.markers.resize(xt.size());
		for(int i=0; i<xt.size(); i++){
     circle(visualiztion_map, Point(xt[i]*10, 250 - yt[i]*10), 3, Scalar(255, 255, 255), -1);
     path.markers[i].header.frame_id = "velodyne";
     path.markers[i].header.stamp = ros::Time();
     path.markers[i].id = i;
     path.markers[i].type = visualization_msgs::Marker::ARROW;
     path.markers[i].action = visualization_msgs::Marker::ADD;
     path.markers[i].pose.position.x = xt[i];
     path.markers[i].pose.position.y = yt[i];
     path.markers[i].pose.position.z = 0;
     path.markers[i].pose.orientation.x = 0.0;
     path.markers[i].pose.orientation.y = 0.0;
     path.markers[i].pose.orientation.z = yawt[i];
     path.markers[i].pose.orientation.w = 1.0;
     path.markers[i].scale.x = 1;
     path.markers[i].scale.y = 0.1;
     path.markers[i].scale.z = 0.1;
     path.markers[i].color.a = 1.0;
     path.markers[i].color.r = 1.0f;
     path.markers[i].color.g = 0.0f;
     path.markers[i].color.b = 0.0f;
     path_array.data.push_back(xt[i]);
		}
		for(int i=0; i<xt.size(); i++){path_array.data.push_back(yt[i]);}
		char str[100];
		sprintf(str, "find good path, result size : %ld",result.size());
		show_map(str);
    clock_t end =clock();
    path_pub.publish(path);
    path_array_pub.publish(path_array);
		x_last = xt;
		y_last = yt;
		warning_count = 0;
    //printf("\n\n frame: %lf \n\n", 1/((double)(end-start)/CLOCKS_PER_SEC));
 }
};
int main(int argc, char* argv[]) {
	cout<<"main"<<endl;
	ros::init(argc, argv, "local_path_planner");
	ros::NodeHandle nh;
	Planning plan(&nh);
  ros::spin();
	return 0;
}
