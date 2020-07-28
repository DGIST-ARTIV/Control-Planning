#include<bits/stdc++.h>

using namespace std;

double L = 1.0;
double ds = 0.2;
double v = 5.0/3.6;
double PI = acos(-1);

double mapk[500][500];

int max_iter = 50;

double cost_th = 0.1;

struct State{
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double v = 0.0;
};

bool check_map(double (*map)[500], double x, double y){
	int xx = round(x*10);
	int yy = 250 - round(y*10);
	if(yy<0 || yy>499 || xx<0 || xx>499) return false;
	else if( map[yy][xx]==1 ) return true;
	else return false;
}

double pi_2_pi(double angle){
	while(angle < -PI) angle += 2*PI;
	while(angle > PI) angle -= 2*PI;
	return angle;
}

State update(State state, double v, double delta, double dt, double L){
	state.v = v;
	state.x += state.v * cos(state.yaw) * dt;
	state.y += state.v * sin(state.yaw) * dt;
	state.yaw += state.v/L * tan(delta) * dt;
	state.yaw = pi_2_pi(state.yaw);
	
	return state;
}

void generate_trajectory(double s, double km, double kf, double k0, vector<double>& x, vector<double>& y, vector<double>& yaw){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	
	double tk[3] = {0.0, time/2.0, time};
	double kk[3] = {k0, km, kf};
	
	vector<double> kp;
	
	/*
	 A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
	        = -time*time*time/4
	*/
	
	double invA[2][2] = {{time, -time/2}, {-time*time, time*time/4}};
	double detA = -time*time*time/4;
	double A = (invA[0][0]*(km-k0) + invA[0][1] * (kf-k0)) / detA;
	double B = (invA[1][0]*(km-k0) + invA[1][1] * (kf-k0)) / detA;
	double C = k0;
	
	for(double t = 0.0; t<time; t += time/n){
		kp.push_back( A*t*t + B*t + C );
	}
	
	int dt = time/n;
	
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

/*void generate_trajectory_costmap(double s, double km, double kf, double k0, double (*map)[500], vector<double>& x, vector<double>& y, vector<double>& yaw){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	
	double tk[3] = {0.0, time/2.0, time};
	double kk[3] = {k0, km, kf};
	
	vector<double> kp;
	
	
	 A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
	        = -time*time*time/4
	
	
	double invA[2][2] = {{time, -time/2}, {-time*time, time*time/4}};
	double detA = -time*time*time/4;
	double A = (invA[0][0]*(km-k0) + invA[0][1] * (kf-k0)) / detA;
	double B = (invA[1][0]*(km-k0) + invA[1][1] * (kf-k0)) / detA;
	double C = k0;
	
	for(double t = 0.0; t<time; t += time/n){
		kp.push_back( A*t*t + B*t + C );
	}
	
	int dt = time/n;
	
	State state;
	x.push_back(state.x);
	y.push_back(state.y);
	yaw.push_back(state.yaw);
	
	for(double ikp : kp){
		state = update(state, v, ikp, dt, L);
		x.push_back(state.x);
		y.push_back(state.y);
		yaw.push_back(state.yaw);
		if(check_map(map, state.x, state.y)) cnt++;
		cost = cost + map[250-round(state.y*10)][round(state.x*10)];
	}
	
	
}*/

vector<double> generate_last_state(double s, double km, double kf, double k0, vector<double>& x, vector<double>& y, vector<double>& yaw){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	
	double tk[3] = {0.0, time/2.0, time};
	double kk[3] = {k0, km, kf};
	
	vector<double> kp;
	
	/*
	 A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
	        = -time*time*time/4
	*/
	
	double invA[2][2] = {{time, -time/2}, {-time*time, time*time/4}};
	double detA = -time*time*time/4;
	double A = (invA[0][0]*(km-k0) + invA[0][1] * (kf-k0)) / detA;
	double B = (invA[1][0]*(km-k0) + invA[1][1] * (kf-k0)) / detA;
	double C = k0;
	
	for(double t = 0.0; t<time; t += time/n){
		kp.push_back( A*t*t + B*t + C );
	}
	
	int dt = time/n;
	
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
	
	vector<double> res;
	res.resize(3);
	res[0] = state.x;
	res[1] = state.y;
	res[2] = state.yaw;
	
	return res;
}


int main(){
	cout<<"hello world!"<<endl;
	
}
