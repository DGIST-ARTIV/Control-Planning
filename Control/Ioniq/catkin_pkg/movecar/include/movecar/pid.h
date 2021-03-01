#ifndef pid_h
#define pid_h

#include <bits/stdc++.h>
#include "time.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
using namespace std;


class PID{
public:
	double now_data{0}, prev_data{0};
	double error{0}, i_error{0}, d_error{0}, prev_error{0};
	double kp{0}, ki{0}, kd{0}, aw{0};

	time_t now_t{clock()}, prev_t{clock()};

	double hz = 100;

	PID(double kp_, double ki_, double kd_, double aw_){
		kp = kp_;
		ki = ki_;
		kd = kd_;
		aw = aw_;
	}
	void set(double kp_, double ki_, double kd_, double aw_){
		kp = kp_;
		ki = ki_;
		kd = kd_;
		aw = aw_;
	}
	double out(double target, double now){
		now_t = clock();
		double dt = 1.0/hz;
		error = target - now;
		d_error = (error - prev_error)/dt;
		i_error += error*dt;
		i_error = min(aw, max(-aw, i_error));

		prev_error = error;
		prev_t = now_t;
		printf("error : %lf i_error : %lf, d_error : %lf\n", error, i_error, d_error);
		printf("dt : %lf p : %lf i : %lf d : %lf tot : %lf \n", dt, kp*error, ki*i_error,kd*d_error,kp*error+ki*i_error+kd*d_error );
		return kp*error+ki*i_error+kd*d_error;
	}
	void reset_pid(){
		error = 0;
		prev_error = 0;
		i_error = 0;
		d_error = 0;
		return;
	}
};

# endif