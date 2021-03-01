/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For a temporary license for autonomous vehicle
 *******************************************************/
#include "scc/scc.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

int check_map(Mat map, double x, double y, double scale) {
	int xx = int(x * scale);
	int yy = 250 - int(y * scale);
	if (yy < 0 || yy>499 || xx < 0 || xx>499) return 0;
	else {return int(map.data[500*yy+xx]);}
}

void reset_map(Mat& map){
	map = Scalar(0);
}

double pi_2_pi(double angle) {
	while (angle < -PI) angle += 2 * PI;
	while (angle > PI) angle -= 2 * PI;
	return angle;
}
State update(State state, double v, double delta, double dt, double L) { // dt : sampling scale L : wheel base
	state.v = v;
    state.x += state.v * cos(state.yaw) * dt;
	state.y += state.v * sin(state.yaw) * dt;
	state.yaw += state.v / L * tan(delta) * dt;
	state.yaw = pi_2_pi(state.yaw);
  return state;
}
Point upper_points(double x, double y, double yaw, double length, double scale = 1){
	Point point;
	point.y = scale*((250.0/scale - y) + cos(yaw) * length);
	point.x = scale*(x - sin(yaw) * length);
	return point; 
}
Point lower_points(double x, double y, double yaw, double length, double scale = 1){
	Point point;
	point.y = scale*((250.0/scale - y) - cos(yaw) * length);
	point.x = scale*(x + sin(yaw) * length);
	return point;
}

SCC::SCC(ros::NodeHandle *nodehandle){
	ros::NodeHandle nh(*nodehandle);
	ros::param::get("~debug", Debug);
	ros::param::get("~hz", hz);
	ros::param::get("~max_velocity", max_velocity);
	ros::param::get("~scale", scale);
	ros::param::get("~min_distance", min_distance);
	ros::param::get("~upper_obstacle_filter", upper_obstacle_filter);
	ros::param::get("~lower_obstacle_filter", lower_obstacle_filter);
	ros::param::get("~threthold",threthold);
	ros::param::get("~arrival_time",arrival_time);

	ros::param::get("~wheel_base", wheel_base);

	LOG.flag = Debug;
	ROS_INFO("hz : %lf max_velocity : %lf scale : %lf min_distance : %lf upper_filter : %lf lower filter %lf", hz, max_velocity,scale, min_distance,upper_obstacle_filter, lower_obstacle_filter);
	ROS_INFO("threthold : %lf, arrival_time : %lf wheel_base : %lf", threthold, arrival_time, wheel_base);
	ros::Subscriber object_sub = nh.subscribe("/lidar/tracking_objects", 1, &SCC::object_callback, this);
	ros::Subscriber state_sub = nh.subscribe("/Ioniq_info", 1, &SCC::state_callback, this);
	ros::Subscriber desired_velocity_sub = nh.subscribe("/scc/desire_velocity", 1, &SCC::velocity_callback, this);
	ros::Subscriber max_velocity_sub = nh.subscribe("/dbw_cmd/MaxSpeed", 1, &SCC::max_velocity_callback, this);

	//movecar_pub = nh.advertise<std_msgs::Float32MultiArray>("/move_car", 1);
	velocity_pub = nh.advertise<std_msgs::Float64>("/scc/velocity", 1);
	closest_distance_pub = nh.advertise<std_msgs::Float64MultiArray>("/scc/closest_distance", 1);
	Estop_pub = nh.advertise<std_msgs::Bool>("/move_car/Estop", 1);


	thread thread(&SCC::publishing, this);
	ros::spin();

}
void SCC::draw_line(){
	reset_map(line_map);
	double distance = 0;
	State state;
	state.v = now_velocity;
	vector<Point> points;
	vector<State> line_state;
	Point p_i(0, 0);
	p_i = upper_points(0,0,0,wheel_base,scale);
	points.push_back(p_i);
	p_i = lower_points(0,0,0,wheel_base,scale);
	int cnt = 0;
	while(ros::ok() && cnt<9 && distance <=150){
		double prev_yaw = state.yaw;
		state = update(state, now_velocity,now_steer*PI/180, 1, 2.8);
		if(abs(state.yaw - prev_yaw)>PI/2) break;
		distance = sqrt(state.x*state.x + state.y*state.y);
		line_state.push_back(state);
	}
	int size = line_state.size();
	for(int i=0; i<size; i++){
		Point p;
		p = upper_points(line_state[i].x, line_state[i].y,line_state[i].yaw,wheel_base*(size + i*3)/size, scale);
		points.push_back(p);
	}
	for(int i=size-1; i>=0; i--){
		Point p;
		p = lower_points(line_state[i].x, line_state[i].y,line_state[i].yaw,wheel_base*(size + i*3)/size, scale);
		points.push_back(p);		
	}
	points.push_back(p_i);
	const Point *pts1 = (const cv::Point*) Mat(points).data;
	int npts1 = Mat(points).rows;
	fillPoly(line_map, &pts1, &npts1, 1,Scalar(255,255,255));
	fillPoly(debug_map, &pts1, &npts1, 1,Scalar(0,255,0));
	
	points.clear();
	
	//imshow("line map", line_map);
	//waitKey(1);
}
void SCC::state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	now_steer = -msg->data[3]*14.67/525;
	now_velocity = max(0.5, msg->data[19]/3.6);
	//LOG.INFO(format_string("now v : %lf steer : %lf", now_velocity, now_steer));
	draw_line();
}
void SCC::object_callback(const tracking_msg::TrackingObjectArray& msg){
	prev_time = high_resolution_clock::now();
	obstacles.clear();
	obstacles.resize(0);
	double distance = 10000;
	obstacle_map = line_map.clone();
	int size = msg.array.size();
	for(int i=0; i<size; i++){
		if(true){
			double x = msg.array[i].point.x;
			double y = msg.array[i].point.y;
			double s = sqrt(pow(msg.array[i].bev.data[0] - msg.array[i].bev.data[4], 2) + pow(msg.array[i].bev.data[1] - msg.array[i].bev.data[5], 2)); 
			if(s>upper_obstacle_filter || s<lower_obstacle_filter){
				 circle(debug_map, Point(x*scale, 250 - y*scale), 5, Scalar(0,0,255), FILLED, LINE_8);
				 continue;
			}

			circle(obstacle_map, Point(x*scale, 250 - y*scale), 10, Scalar(255), FILLED, LINE_8);
			circle(debug_map, Point(x*scale, 250 - y*scale), 10, Scalar(125,0,0), FILLED, LINE_8);
			
			if(check_map(line_map, x, y, scale)>0){
				double d = sqrt(x*x + y*y);
				if(distance >= d){
					circle(debug_map, Point(x*scale, 250 - y*scale), 10, Scalar(0,255,255), FILLED, LINE_8);
					distance = d;
					closest_object.x = x;
					closest_object.y = y;
					closest_object.v_x = msg.array[i].velocity.x + now_velocity;
					closest_object.v_y = msg.array[i].velocity.y;
					closest_object.length = d;
					closest_object.state = msg.array[i].state;
				}
			}
			else continue;
		}
		else if(msg.array[i].type_id == 2){
			double x = msg.array[i].point.x;
			double y = msg.array[i].point.y;
			circle(obstacle_map, Point(x*scale, 250 - y*scale), 10, Scalar(255), FILLED, LINE_8);
			if(check_map(line_map, x, y, scale)){
				double d = sqrt(x*x + y*y);
				if(distance >= d){
					distance = d;
					closest_object.x = x;
					closest_object.y = y;
					closest_object.v_x = msg.array[i].velocity.x + now_velocity;
					closest_object.v_y = msg.array[i].velocity.y;
					closest_object.length = d;
					closest_object.state = msg.array[i].state;
				}
			}
			else continue;
		}
		else continue;
	}
	imshow("Debug", debug_map);

	//imshow("obstacle map", obstacle_map);
	waitKey(1);
	reset_map(debug_map);
	closest_distance = distance;
	return;
}
void SCC::max_velocity_callback(const std_msgs::Int16::ConstPtr& msg){
	max_velocity = double(msg->data)/3.6;
}
void SCC::velocity_callback(const std_msgs::Float64::ConstPtr& msg){  
	desire_velocity = msg->data/3.6;
	filter_velocity = desire_velocity / hz;
}
void SCC::AEB(){
		std_msgs::Bool data;
		data.data = true;
		ROS_FATAL("type1 - Auto Emergenct stop time : %lf", now_velocity/7.5);
		putText(debug_map,format_string("Auto Emergenct stop time : %lf", now_velocity/7.5), Point(0, 250),1,1,Scalar(0,0,255));
		Estop_pub.publish(data);
		return;
}

void SCC::move_car(double velocity){
	/*double cmd_velocity = 0;
	if(velocity >= now_velocity){
		cmd_velocity = min(now_velocity+filter_velocity/arrival_time, velocity);
	}
	else{
		cmd_velocity = max(now_velocity-filter_velocity, velocity);
	}*/
	std_msgs::Float64 vel;

	if(velocity <=0){
		if(closest_object.state){
			vel.data = closest_object.v_x;// - threthold*now_velocity/desire_velocity;
			putText(debug_map,format_string("closest vehicle vel : %lf",closest_object.v_x*3.6) , Point(250,10), 1, 1, Scalar(255,255,255));
		}
		else{
			vel.data = 0;
		}
		
		putText(debug_map,"WARNING : Too close front vehicle", Point(0, 250),1,1, Scalar(0,0,255));
	}
	else{
		vel.data = velocity*3.6;
		putText(debug_map,format_string("now : %lf cmd : %lf",now_velocity*3.6, vel.data) , Point(0,10), 1, 1, Scalar(255,255,255));
		LOG.WARNING(format_string("cmd velocity : %lf", vel.data*3.6));
	}
	velocity_pub.publish(vel);
}

void SCC::publishing(){
	while(ros::ok()){
		double time = ros::Time::now().toSec();
		std_msgs::Float64MultiArray distance;
		distance.data.push_back(time);
		now_time = high_resolution_clock::now();
		duration<double> time_span = duration_cast<duration<double>>(now_time - prev_time);
		double temp_desire_velocity = min(desire_velocity, now_velocity+threthold/3.6);
		//cout<<temp_desire_velocity<<endl;
		if(time_span.count()>1.0){
			reset_map(obstacle_map);
			reset_map(line_map);
			reset_map(obstacle_map);
			closest_distance = 10000;
			LOG.INFO("Reset map");
		}

		LOG.INFO(format_string("closest_distance : %lf", closest_distance));
		putText(debug_map,format_string("closest_distance : %lf", closest_distance) , Point(0,450), 1, 1, Scalar(255,255,255));
		if(closest_distance == 10000){
			distance.data.push_back(-1);
			LOG.INFO(format_string("Play desired velocity : %lf", desire_velocity*3.6));
			move_car(temp_desire_velocity);
		}
		else{
			distance.data.push_back(closest_distance);
			double S = max(min(now_velocity*(0.2+2.9*now_velocity/36.1), now_velocity*2), min_distance);
			double s = max(S, now_velocity*now_velocity/16);
			
			double v = max( -1.0/3.6, temp_desire_velocity*(1 -  pow(s/closest_distance, 2)));
			//double v = max( -1.0/3.6, temp_desire_velocity*(1 -  pow(min_distance/closest_distance, 2)));
			
			putText(debug_map,format_string("w1 : %lf w2 : %lf", s/closest_distance, pow(now_velocity/max_velocity/2,4)), Point(0,430),1,1,Scalar(255,255,255));
			move_car(v);
			LOG.INFO(format_string("Calc S : %lf s : %lf V : %lf",S, s,v*3.6)); 
			putText(debug_map,format_string("Calc S : %lf s : %lf V : %lf",S, s,v*3.6) , Point(0,465), 1, 1, Scalar(255,255,255));
		}
		closest_distance_pub.publish(distance);
		usleep(1000000/hz);
	}
}