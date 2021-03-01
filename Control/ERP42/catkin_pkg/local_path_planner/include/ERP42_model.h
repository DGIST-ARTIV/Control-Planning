#ifndef motion_model
#define motion_model

#include<bits/stdc++.h>
​
using namespace std;
​
double L = 1.0;
double ds = 0.2;
double PI = acos(-1);
​
double mapk[500][500];
​
int max_iter = 50;
​
double cost_th = 0.15;
​struct State {
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
#endif
