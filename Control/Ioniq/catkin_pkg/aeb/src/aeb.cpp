/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For a temporary license for autonomous vehicle
 *******************************************************/
#include "aeb/aeb.h"
void reset_map(Mat& map){
	map = Scalar(0);
}
AEB::AEB(ros::NodeHandle *nodehandle){
	ros::NodeHandle nh(*nodehandle);
	ros::Subscriber state_sub = nh.subscribe("/Ioniq_info",1 , &AEB::state_callback, this);
	ros::Subscriber object_sub = nh.subscribe("/lidar/tracking_objects", 1, &AEB::object_callback, this);
	Estop_pub = nh.advertise<std_msgs::Bool>("/move_car/Estop", 1);
	thread thread(&AEB::publishing, this);
	ros::spin();
}

void AEB::publishing(){
	while(ros::ok()){
		if(flag){
			cout<<"already did E-stop"<<endl;
			usleep(10000);
			continue;
		}

		double scan_length = now_velocity*now_velocity/12 + 3;
		cout<<"l : "<<scan_length<<endl;
		for(double i=0; i<scan_length*scale; i+=1){
			double l = wheel_base*(scan_length*scale + i)/scan_length/2;
			for(double j = l; j>=-l; j-=1){
				int id = check_map(obstacle_map, double(i),double(j),1);
				//cout<<"id "<<id<<endl;
				circle(debug_map, Point(i, 250 - j), 1, Scalar(255), FILLED, LINE_8);
				
				if(id){
					cout<<id<<endl;
					double distance = double(i)/scale;
					if(obstacles[id].state == 1){
						if(obstacles[id].v_x >=0) continue;
						else{
							double t = (now_velocity + obstacles[id].v_x)/now_velocity;
							if(distance <= now_velocity*t -4*t*t){
								bracking();
							}
							else continue;
						}
					}
					bracking();
				}
			}
		}
		usleep(10000);
	}
}
void AEB::bracking(){
	std_msgs::Bool data;
	data.data = true;
	Estop_pub.publish(data);

}
void AEB::state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	now_velocity = msg->data[19]/3.6;
	if(msg->data[5]) flag = false;
}

void AEB::object_callback(const tracking_msg::TrackingObjectArray& msg){
	reset_map(obstacle_map);
	reset_map(debug_map);
	obstacles.clear();
	int size = msg.array.size();
	int id = 0;
	for(int i=0; i<size; i++){
		object obstacle;
		double x = msg.array[i].point.x;
		double y = msg.array[i].point.y;
		double s = sqrt(pow(msg.array[i].bev.data[0] - msg.array[i].bev.data[4], 2) + pow(msg.array[i].bev.data[1] - msg.array[i].bev.data[5], 2));
		if(s>filter) continue;
		else{
			obstacle.x = x;
			obstacle.y = y;
			obstacle.size = s;
			obstacle.id = id;
			obstacle.v_x = msg.array[i].velocity.x;
			obstacle.state = msg.array[i].state;
			id++;
			for(int j=0; j<4; j++){
				Point p;
				p.x = msg.array[i].bev.data[j*2] * scale;
				p.y = 250 - msg.array[i].bev.data[j*2+1] * scale;
				obstacle.bev.push_back(p);
			}
			draw_object(obstacle_map, obstacle.bev, obstacle.id);
			draw_object(debug_map, obstacle.bev, 255);
			obstacles.push_back(obstacle);
		}
		//cout<<"id : "<<id<<endl;
	}

	imshow("obstacle map", debug_map);
	waitKey(1);
}
void draw_object(Mat& map, vector<Point>& p, int id){
	const Point *pts = (const Point*) Mat(p).data;
	int npts = Mat(p).rows;
	fillPoly(map, &pts, &npts,1,Scalar(id));
}
int check_map(Mat& map, double x, double y, double scale) {
	int xx = round(x * scale);
	int yy = 250 - round(y * scale);
	if (yy < 0 || yy>499 || xx < 0 || xx>499) return 0;
	else return int(map.data[500*yy+xx]);
}
