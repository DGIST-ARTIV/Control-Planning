/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 0.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
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



using namespace std;
using namespace cv;

std::string parking_path = "/home/artiv/mission_ros1_ws/src/ERP42_parking/path/parking2-1.csv";
Mat visualization_map = Mat::zeros(500, 500, CV_8UC1);
int parking_num = 1;
double velocity{3}, Kp_alpha{7}, Kp_beta{-3}, dt{0.1}, d0{1}, back_speed{5},curve_speed{6};
double Alpha = 27.0/2000;
int cmd_type = 1;
int debuging = 1;
int stop_time{10}, back_time{5}, curve_time{4};
int max_steer = 2000;
const double PI = acos(-1);

ros::Publisher path_array_pub;
ros::Publisher path_pub;
ros::Publisher move_car_pub;
ros::Publisher acc_pub;
ros::Publisher steer_pub;
ros::Publisher break_pub;
ros::Publisher gear_pub;
ros::Publisher MissionStatus;
ros::Publisher NavStatus;

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
vector< vector<double> > get_path(int num) {
	FILE* fpi;
	fpi = fopen(parking_path.c_str(), "r");
	if (fpi < 0)
		{
		ROS_FATAL_STREAM("csv file open error!") ;
		}
	vector< vector<double> > res;
	char tmp[300];
	int cnt = 1;
	while (1) {
		cnt++;
		double x, y;
		fscanf(fpi, "%lf,%lf",  &x, &y);
		if (feof(fpi)) break;
		vector<double> t;
		t.push_back(x);
		t.push_back(y);
		res.push_back(t);
	}
	fclose(fpi);
	return res;
}
void reset_map(Mat& map){
  map = Scalar(0);
}
void show_map(char *msg){

  putText(visualization_map,msg, Point(0, 50), 1, 1, Scalar(255));
  imshow("visualization_map", visualization_map);
  waitKey(1);
  reset_map(visualization_map);
}
double pi_2_pi(double angle) {
  while (angle < -PI) angle += 2 * PI;
  while (angle > PI) angle -= 2 * PI;
  return angle;
}
class Parking{
public:
  double x_current{0},y_current{0},yaw_current{0}, steer{0};
  vector< vector<double> > path;
  vector< vector<double> > convert_path;
  int parking_num{0}, count{0}, len{0};
  ros::Subscriber state_sub;
  ros::Subscriber location_sub;
  ros::Subscriber yaw_sub;
  Parking(ros::NodeHandle *nodeHandle){
    ROS_INFO("initializing parking");
    path = get_path(parking_num);
    len = path.size();
    ros::NodeHandle nh(*nodeHandle);
    count = 1;
    ROS_INFO("Path length = %d", len);
    acc_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Accel", 1);
		steer_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Steer", 1);
		break_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Brake", 1);
    gear_pub = nh.advertise<std_msgs::Int16>("dbw_cmd/Gear", 1);

		move_car_pub = nh.advertise<std_msgs::Float32MultiArray>("/move_car", 1);
    MissionStatus = nh.advertise<std_msgs::Int16>("/mission_manager/mission_status", 1);
    NavStatus = nh.advertise<std_msgs::Int16>("/mission_manager/nav_pilot", 1);
    std_msgs::Int16 nav;
    nav.data = 0;
    NavStatus.publish(nav);
    ROS_INFO("Set mission mode move_car");
    sleep(1);
    ROS_INFO("Done");

    state_sub = nh.subscribe("/ERP42_info", 1, &Parking::state_callback, this);
    yaw_sub = nh.subscribe("/opencr/imu_yaw", 1, &Parking::yaw_callback, this);
    location_sub = nh.subscribe("/utm_fix", 1, &Parking::location_callback, this);
  }
  void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    steer = msg->data[5];
  }
  void yaw_callback(const std_msgs::Float64::ConstPtr& msg){
    yaw_current = -msg->data;
  }
	void location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)	{
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    if(count){
      convert_path.clear();
      convert_path.resize(0);
      double d[3][2];
  		for(int i = 0; i< len; i++){
        vector<double> t;
        double xt = cos(yaw_current*PI/180)*(path[i][0]-x_current)-sin(yaw_current*PI/180)*(path[i][1]-y_current);
        double yt = sin(yaw_current*PI/180)*(path[i][0]-x_current)+cos(yaw_current*PI/180)*(path[i][1]-y_current);
  			t.push_back(xt);
  		  t.push_back(yt);
        convert_path.push_back(t);
        //printf("11 %d %lf %lf %lf %lf \n",i, path[i][0], path[i][1], convert_path[i][0],convert_path[i][1]);
  		}
      int id = len+1;
      for(int i=0;  i<len; i++){
        if(convert_path[i][0] > 0 && sqrt(convert_path[i][0]*convert_path[i][0]+convert_path[i][1]*convert_path[i][1])>d0+1){
          id = i;
          break;
        }
      }
      if(id == len+1){
        ROS_WARN("Parking is gone TT");
        return;
      }
      ROS_INFO("id : %d, target %d",id, len - 4);
      if(id == len-3 ){
        ROS_INFO("In the Parking lot");
        std_msgs::Int16 acc_;
        std_msgs::Int16 steer_;
        std_msgs::Int16 break_;
        std_msgs::Int16 gear_;
        //go back
        ROS_INFO("Estop!");
        for(int i=0; i<stop_time && ros::ok(); i++){
          if(cmd_type == 1){
            std_msgs::Float32MultiArray move;
            move.data.resize(10);
            move.data[0] = 1; //car_type
            move.data[1] = 0; //mode
            move.data[2] = 0; // desired_speed
            move.data[3] = 0;
            move.data[4] = 0;
            move.data[5] = 0;
            move.data[6] = 2;
            move.data[7] = 0;
            move.data[8] = 0;
            move.data[9] = 0;
            move_car_pub.publish(move);
          }
          else{
            acc_.data = 0;
            steer_.data = 0;
            break_.data = 200;
            gear_.data = 0;
            acc_pub.publish(acc_);
            steer_pub.publish(steer_);
            break_pub.publish(break_);
            gear_pub.publish(gear_);
          }
          printf("%d\n", i);
          sleep(1);
        }
        if(cmd_type == 1){
	  for(int i=0; i<10 && ros::ok(); i++){
	
          std_msgs::Float32MultiArray move;
          move.data.resize(1);
          move.data[0] =119;
          move_car_pub.publish(move);
          usleep(100000);
}
        }
        ROS_INFO("Go Back");
        for(int i=0; i<back_time&&ros::ok(); i++){
          if(cmd_type == 1){
            std_msgs::Float32MultiArray move;
            move.data.resize(10);
            move.data[0] = 1; //car_type
            move.data[1] = 6; //mode
            move.data[2] = back_speed; // desired_speed
            move.data[3] = 0;
            move.data[4] = 0;
            move.data[5] = 0;
            move.data[6] = 2;
            move.data[7] = 0;
            move.data[8] = 0;
            move.data[9] = 0;
            move_car_pub.publish(move);
          }
          else{
            acc_.data = 50;
            steer_.data = 0;
            break_.data = 0;
            gear_.data = 2;
            acc_pub.publish(acc_);
            steer_pub.publish(steer_);
            break_pub.publish(break_);
            gear_pub.publish(gear_);
          }
          sleep(1);
          printf("%d\n", i);
        }
        ROS_INFO("Stop!");
        for(int i=0; i<3 && ros::ok(); i++){
          if(cmd_type == 1){
            std_msgs::Float32MultiArray move;
            move.data.resize(10);
            move.data[0] = 1; //car_type
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
          }
          else{
            acc_.data = 0;
            steer_.data = 0;
            break_.data = 200;
            gear_.data = 0;
            acc_pub.publish(acc_);
            steer_pub.publish(steer_);
            break_pub.publish(break_);
            gear_pub.publish(gear_);
          }
          sleep(1);
          printf("%d\n", i);
        }
        for(int i=0; i<10 && ros::ok(); i++){
          std_msgs::Float32MultiArray move;
          move.data.resize(1);
          move.data[0] =119;
          move_car_pub.publish(move);
	  usleep(100000);
	}
	for(int i=0; i<curve_time && ros::ok(); i++){
          if(cmd_type == 1){
            std_msgs::Float32MultiArray move;
            move.data.resize(10);
            move.data[0] = 1; //car_type
            move.data[1] = 6; //mode
            move.data[2] = curve_speed; // desired_speed
            move.data[3] = 0;
            move.data[4] = 0;
            move.data[5] = -1000;
            move.data[6] = 0;
            move.data[7] = 0;
            move.data[8] = 0;
            move.data[9] = 0;
            move_car_pub.publish(move);
          }
          else{
            acc_.data = 60;
            steer_.data = -1000;
            break_.data = 0;
            gear_.data = 0;
            acc_pub.publish(acc_);
            steer_pub.publish(steer_);
            break_pub.publish(break_);
            gear_pub.publish(gear_);
          }
          sleep(1);
          printf("%d\n", i);
        }
        for(int i=0; i<2 && ros::ok(); i++){

          std_msgs::Int16 navstate;
          std_msgs::Int16 missionstate;
          navstate.data = 1;

          missionstate.data = 3;

          NavStatus.publish(navstate);
          MissionStatus.publish(missionstate);
          sleep(1);  
          printf("%d\n", i);
        }

        ROS_INFO("Done Parking node I will go to bad");
        exit(0);
      }

  		for(int i=0; i<3; i++){
        d[i][0] = convert_path[i+id][0];
        d[i][1] = convert_path[i+id][1];
  			circle(visualization_map, Point(d[i][0]*10, 250 - d[i][1]*10), 3, Scalar(255), 1);
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
  			printf("Along Parking Path");
  			movecar(1,1,1,1,1);
  			return;
  		}

  		char a[100], str[100];
  		sprintf(a, "d0 : %lf, id : %d", d0, id);
  		putText(visualization_map, a, Point(0, 150), 1, 1, Scalar(255));
  		//main callback do path planning
  		// here will set destination
  		double k0 = -steer*PI/180; //check this value, this value is for ERP42

  		double l_center = max(-25.0, min(25.0, A*d0*d0*d0+B*d0*d0+C*d0));
  		double l_heading = max(-PI/2, min(PI/2, atan(3*A*d0*d0+2*B*d0+C)));
  		movecar(d0,l_center,l_heading,k0,0);
      sprintf(str, "find good path");
      show_map(str);
    }
	}
  void movecar(double x, double y, double yaw, double yaw_init, int type){
    double alpha = pi_2_pi(atan(y/x) - yaw_init);
    double beta = pi_2_pi(yaw - yaw_init -  alpha);
    double w = (Kp_alpha * alpha + Kp_beta*beta)*dt;
    int steer =  -max(-max_steer, min(max_steer, int(w * 180/PI/Alpha)));
    //printf("%d %d %lf\n",max_steer, steer,double(abs(steer))/max_steer);
    int acc = max(0,int(double(velocity) - double(abs(steer))/max_steer*velocity/2));
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
    if(cmd_type == 1){
      if(type == 0){
        std_msgs::Float32MultiArray move;
        move.data.resize(10);
        move.data[0] = 1; //car_type
        move.data[1] = 6; //mode
        move.data[2] = double(acc); // desired_speed
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
        steer_.data = 0;
        break_.data = 50;
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
      }
    }
  }
};

int main(int argc, char* argv[]){
  ros::init(argc, argv, "Parking");
	ros::NodeHandle nh;
  ros::param::get("~velocity", velocity);
  ros::param::get("~Kp_alpha", Kp_alpha);
  ros::param::get("~Kp_beta", Kp_beta);
  ros::param::get("~dt", dt);
  ros::param::get("~distance", d0);
  ros::param::get("~cmd_type", cmd_type);
  ros::param::get("~parking_path", parking_path);
  ros::param::get("~debuging", debuging);
  ros::param::get("~stop_time", stop_time);
  ros::param::get("~back_time", back_time);
  ros::param::get("~back_speed", back_speed);
  ros::param::get("~curve_speed", curve_speed);
  ros::param::get("~curve_time", curve_time);
  ROS_INFO("velocity : %lf", velocity);
  ROS_INFO("Kp_alpha : %lf Kp_beta : %lf dt : %lf", Kp_alpha, Kp_beta, dt);
  ROS_INFO("distance : %lf Debuging : %d", d0, debuging);
  ROS_INFO("stop_time : %d back_time : %d back_speed : %lf", stop_time, back_time, back_speed);

  ROS_INFO("%s", parking_path.c_str());

  Parking plan(&nh);
  ros::spin();

  return 0;
}
