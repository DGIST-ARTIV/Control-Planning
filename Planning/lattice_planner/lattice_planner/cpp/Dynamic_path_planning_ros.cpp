#include <bits/stdc++.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/Float64MultiArray.hpp"
#include "sensor_msgs/msg/NavSatFix.hpp"
#include "geometry_msgs/msg/PointStamped.hpp"
#include "geometry_msgs/msg/PoseArray.hpp"
#include "std_msgs/msg/JointState.hpp"
using namespace std;

double mapk[500][500];
const int max_iter = 50;
const double h[3] = { 0.5, 0.02, 0.02 }; //check
const double cost_th = 0.15;
const double con = 0.1;// map cost constant
//p is ve0
const double L = 1.0;
const double ds = 0.2;
const double PI = acos(-1);

struct State {
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double v = 0.0;
};
bool check_map(double(*map)[500], double x, double y) {
	cout << "check_map" << endl;
	int xx = round(x * 10);
	int yy = 250 - round(y * 10);
	if (yy < 0 || yy>499 || xx < 0 || xx>499) return false;
	else if (map[yy][xx] == 1) return true;
	else return false;
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
void generate_trajectory(double s, double km, double kf, double k0, vector<double>& x, vector<double>& y, vector<double>& yaw, double v) {
	double n = s / ds;
	double time = s / v;
	double cost = 0;
	double tk[3] = { 0.0, time / 2.0, time };
	double kk[3] = { k0, km, kf };
	vector<double> kp;
	/*
	 A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
			= -time*time*time/4
	*/
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
}
double generate_trajectory_costmap(double s, double km, double kf, double k0, double (*map)[500], vector<double>& x, vector<double>& y, vector<double>& yaw, double v){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	double tk[3] = {0.0, time/2.0, time};
	double kk[3] = {k0, km, kf};
	int cnt = 0;
	vector<double> kp;
	/* A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
			= -time*time*time/4*/
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
	double k;
	for(double ikp : kp){
		state = update(state, v, ikp, dt, L);
		x.push_back(state.x);
		y.push_back(state.y);
		yaw.push_back(state.yaw);
		k = check_map(map, state.x, state.y) cnt++;
		if(k==1){
			cnt++;
		}
		cost += k;
	}
	if(cnt>0){
		return 11111;
	}
	return cost;
}
double* generate_last_state(double s, double km, double kf, double k0) {
	double n = s / ds;
	double time = s / v;
	double cost = 0;
	vector<double> x; vector<double> y; vector<double> yaw;
	double tk[3] = { 0.0, time / 2.0, time };
	double kk[3] = { k0, km, kf };
	vector<double> kp;
	/*
	 A = {{time*time/4, time/2}, {time*time, time}}
	 det(A) = time*time*time/4 - time*time*time/2
			= -time*time*time/4
	*/
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
	double res[3];
	res[0] = state.x;
	res[1] = state.y;
	res[2] = state.yaw;
	double* m = res;
	return m;
}
vector<double> calc_diff2(State target, double x, double y, double yaw) {
	vector<double> v;
	v.push_back(target.x - x);
	v.push_back(target.y - y);
	v.push_back(target.yaw - yaw);
	return v;
}
void carc_j(State target, double* p, double* h, double k0, double(*j)[3]) {
	cout << "carc_j" << endl;
	double xp, xn;
	double yp, yn;
	double yawp, yawn;
	vector<double> dn, dp;
	double d1[3], d2[3], d3[3];
	double* k;
	k = generate_last_state(p[0] + h[0], p[1], p[2], k0);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0] - h[0], p[1], p[2], k0);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
	for (int i = 0; i < 3; i++) {
		d1[i] = (dp[i] - dn[i]) / (2.0 * h[0]);

	}
	k = generate_last_state(p[0], p[1] + h[1], p[2], k0);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1] - h[1], p[2], k0);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
	for (int i = 0; i < 3; i++) {
		d2[i] = (dp[i] - dn[i]) / (2.0 * h[1]);
	}
	k = generate_last_state(p[0], p[1], p[2] + h[2], k0);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1], p[2] - h[2], k0);
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
double selection_learning_param(double* dp, double* p, double k0, State target) {
	double mincost = 100000000.0;
	double mina = 1.0;
	double maxa = 2.0;
	double da = 0.5;
	double a[2] = { 1.0, 1.5 };
	double tp[3];
	double* b;
	for (int i = 0; i < 3; i++) {
		printf("SSS : %lf %lf : ", dp[i], p[i]);
	}
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			tp[j] = p[j] + a[j] * dp[j];
		}
		b = generate_last_state(tp[0], tp[1], tp[2], k0);
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
		printf("inv %lf %lf %lf\n", inv[i][0], inv[i][1], inv[i][2]);
	}
	return 0;
}
void optimize_trajectory(State target, double k0, double* p, double* xp, double* yp, double* yawp, double v) {
	cout << "optimize_trajectory" << endl;
	for (int i = 0; i < max_iter; i++) {
		cout << "o_check : " << i << endl;
		vector<double> x;
		vector<double> y;
		vector<double> yaw;

		generate_trajectory(p[0], p[1], p[2], k0, x, y, yaw, v);


		vector<double> dc;
		dc = calc_diff2(target, x[x.size() - 1], y[y.size() - 1], yaw[yaw.size() - 1]);
		double cost1 = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[2] * dc[2]);

		if (cost1 <= cost_th) {
			cout << "find_good_path!" << endl;
			*xp = x[x.size() - 1];
			*yp = y[y.size() - 1];
			*yawp = yaw[yaw.size() - 1];
			return;
		}
		double j[3][3];
		double invJ[3][3], dp[3];
		double* dpp = dp;
		int chekk;
		try {
			carc_j(target, p, h, k0, j);
			chekk = f_inv(j, invJ);
			if (chekk) { throw chekk; }
			for (int i = 0; i < 3; i++) {
				dp[i] = -invJ[i][0] * dc[0] - invJ[i][1] * dc[1] - invJ[i][2] * dc[2];
			}
		}
		catch (int a) {
			*xp = 0;
			cout << "cannot calc path LinAlgError" << endl;
			return;
		}
		double alpha = selection_learning_param(dpp, p, k0, target);
		for (int i = 0; i < 3; i++) {
			p[i] += alpha * dp[i];
		}
		// it need to check return argument
	}
	*xp = 0;
	cout << "can't find path" << endl;
	return;
}
double optimize_trajectory_costmap(State target, double k0, double* p, double* xp, double* yp, double* yawp, double v, double (*map)[500]) {
	cout << "optimize_trajectory_costmap" << endl;
	for (int i = 0; i < max_iter; i++) {
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		double cost = generate_trajectory_costmap(p[0], p[1], p[2], k0, map, x, y, yaw, v);
		vector<double> dc;
		dc = calc_diff2(target, x[x.size() - 1], y[y.size() - 1], yaw[yaw.size() - 1]);
		printf("dc %lf %lf %lf\n", dc[0], dc[1], dc[2]);
		double cost1 = sqrt(dc[0] * dc[0] + dc[1] * dc[1] + dc[2] * dc[2]);
		if (cost1 <= cost_th) {
			cout << "find_good_path!" << endl;
			if(cost==11111){
				*xp=0;
				cout<<"generated path on obstracle or line, but don't worry I will find anouther path : "<<target.x<<","<< target.y <<endl;
				return 0;
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
			carc_j(target, p, h, k0, j);
			chekk = f_inv(j, invJ);
			if (chekk) { throw chekk; }
			for (int i = 0; i < 3; i++) {
				dp[i] = -invJ[i][0] * dc[0] - invJ[i][1] * dc[1] - invJ[i][2] * dc[2];
			}
		}
		catch (int a) {
			*xp = 0;
			cout << "cannot calc path LinAlgError : " << target.x<<","<<target.y<<endl;
			return 0;
		}
		double alpha = selection_learning_param(dpp, p, k0, target);
		for (int i = 0; i < 3; i++) {
			p[i] += alpha * dp[i];
		}
		// it need to check return argument
	}
	*xp = 0;
	cout << "can't find path in this state : " << target.x<<","<<target.y<<endl;
	return 0;
}
int k_min(double a, double b) {
	cout << "k_min" << endl;
	if (a >= b) return int(b);
	else return int(a);
}
int k_max(double a, double b) {
	cout << "k_max" << endl;
	if (a <= b) return int(b);
	else return int(a);
}
void line_on_map(double(*line)[3], double(*map)[500], int type = 0) {
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
}
void lidar_on_map(float(*lidar)[4], double(*map)[500], int type = 0) {
	cout << "lidar_on_map" << endl;
	int m0 = 1e9, M0 = -1;
	int m1 = 1e9, M1 = -1;
	for (int i = 0; i < 4; i++) {
		m0 = min(m0, lidar[0][i]);
		m1 = min(m1, lidar[1][i]);
		M0 = max(M0, lidar[0][i]);
		M1 = max(M1, lidar[1][i]);
	}

	for (int i = max(m0 - 5,0) ; i <= min(M0 + 5, 499); i++) {
		for (int j = min(m1+5, 250) ; j <= max(M1-5,-249); j++) {// 여기 수정 필요함 미리 배열 좌표로 변환할지 생각해보기!
			int y = 250 - j;
			int x = i;
			map[y][x] = 1;
		}
	}
}
vector< vector<double> > get_lookup_table() {
	FILE* fpi;
	fpi = fopen("lookuptable.csv", "r");
	vector< vector<double> > res;
	char tmp[300];
	for (int i = 0; i < 7; i++) {
		fscanf(fpi, "%s", tmp);
	}
	int cnt = 1;
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
	cout << cnt << endl;
	fclose(fpi);
	return res;
}
vector<double> search_nearest_one_from_lookuptable(double tx, double ty, double tyaw, vector< vector<double> >& lookup_table, double v) {
	double mind = 1e9;
	int minid = -1;
	int first = 0;
	int k = lookup_table.size();
	int last = k;
	for (int i = k - 1; i >= 0; i--) {
		double dv = round(v);
		if (dv == lookuptable[i][0]) {
			last == i + 1;
			for (int j = i; j >= 0; j--) {
				if (lookup_table[j][0] == dv - 1) {
					first = j;
				}
			}
		}
		break;
	}
	printf("first last : %d %d", first, last);
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
	cout << minid << endl;
	return lookup_table[minid];
}
vector< vector<double> > generate_path(vector< vector<double> >& target_states, double k0, double v) {
	vector< vector<double> > lookup_table = get_lookup_table();
	vector< vector<double> > result;
	int k = target_states.size();
	for (int i = 0; i < k; i++) {
		cout << "g_check : " << i << endl;
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
		optimize_trajectory(target, k0, init_p, &x, &y, &yaw, v/3.6);// km/h -> m/s check!!
		if (x) {
			cout << "`find_good_path`" << endl;
			vector<double> t;
			t.push_back(x);
			t.push_back(y);
			t.push_back(yaw);
			t.push_back(init_p[0]);
			t.push_back(init_p[1]);
			t.push_back(init_p[2]);

			result.push_back(t);
		}
	}
	return result;
}
vector< vector<double> > generate_path_costmap(vector< vector<double> >& target_states, double k0, double v, double (*map)[500]) {
	vector< vector<double> > lookup_table = get_lookup_table();
	vector< vector<double> > result;
	int k = target_states.size();
	for (int i = 0; i < k; i++) {
		cout << "g_check : " << i << endl;
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
		double cost = optimize_trajectory(target, k0, init_p, &x, &y, &yaw, v/3.6, map);// km/h -> m/s check!!
		if (x) {
				cout << "`find_good_path`" << endl;
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
vector< vector<double> > calc_lane_states(double l_center, double l_heading, double l_width, double v_width, double d, int nxy, double v) {
	cout << "calc_lane_states" << endl;
	double xc = cos(l_heading) * d + sin(l_heading) * l_center;
	double yc = sin(l_heading) * d + cos(l_heading) * l_center;
	vector< vector<double> > states;
	for (int i = 0; i < nxy; i++) {
		double delta = -0.5 * (l_width - v_width) + (l_width - v_width) * i / (nxy - 1);
		double xf = xc - delta * sin(l_heading);
		double yf = yc + delta * cos(l_heading);
		double yawf = l_heading;
		vector<double> t;
		t.push_back(v);
		t.push_back(xf);
		t.push_back(yf);
		t.push_back(yawf);
		states.push_back(t);
	}
	return states;
}
void lane_state_sampling_test1() {
	cout << "lane_state_sampling_test1" << endl;
	double k0 = 0;
	double l_center = 0.0;
	double l_heading = 0.0;
	double l_width = 10.0;
	double v_width = 1.16;
	double d = 10;
	int nxy = 10;
	double v = 10;
	vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy, v);
	vector< vector<double> > result = generate_path(states, k0);

	double mind = 1e9;
	printf("result size : %ld\n", result.size());
	if(result.size()=0){
		cout<<"Warning ! NO PATH GENERATED TT;"<<cout;
	}
	for (int i = 0; i < result.size(); i++) {
		vector<double> table = result[i];
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		generate_trajectory(table[3], table[4], table[5], k0, x, y, yaw);
	}
		/*
				d = math.sqrt((sample_des[0] - table[0][-1])**2+(sample_des[1] - table[1][-1])**2)
				if d<mind:
					mind= d
					x = table[0]
					y = table[1]
		*/
}
class Planning : public rclcpp::Node{
public:
	Planning()
		: Node("local_planning") {
		auto artiv_state_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Ioniq_info", 1, Planning::state_callback, this);
		/*auto artiv_location_sub = this->create_subscription<geometry_msgs::msg::NavSatFix>("/gps_fix", 1, Planning::location_callback, this);
		auto lidar_obstracle_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("topic", 1, Planning::lidar_obstracle_callback, this);
		auto destination_sub = this->create_subscription<std_msgs::msg:: ? ? ? ? >("???", 1, Planning::destination_callback, this);
*/
		auto path_publisher_x = this->create_publisher<std_msgs::msg::Float32MultiArray>("/local_path/x", 1);
		auto path_publisher_y = this->create_publisher<std_msgs::msg::Float32MultiArray>("/local_path/y", 1);
	}
private:
	double v, steer;
	double xs, ys;
	double xd[3], yd[3];
	vector< vector < float> > > lidar;
	double local_map[500][500];
	void resat_map(double (*map)[500]){
		for(int i=0; i<500; i++){
			for(int j=0; j<500; j++){map[j][i] = 0;}
		}
	}

	void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
		cout<<"state_callback!!!"<<endl;
		v = msg->data[3];
		steer = msg->data[0];
	}
	void lidar_obstracle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		int size = sizeof(msg->data);
		if (size % 8 != 0) {
			cout << "Warning lidar data check data!!!!!!!!!!!!" << endl;
			return;
		}
		float l[2][4];
		int k = int(size / 8);
		for (int i = 0; i < k; i++) {
			for (int j = 0; j < 4; j += 1) {
				l[0][j] = msg->data[i * 8 + j * 2]*10;
				l[1][j] = msg->data[i * 8 + j * 2 + 1]*10;
			}
			lidar_on_map(l, local_map);
		}
	}
	void location_callback(const std_msgs::msg::NavSatFix::SharedPtr msg)	{

	}
	void destination_callback(const std_msgs::msg::String::SharedPtr msg)	{
		//main callback do path planning
		// here will set destination
		double k0 = steer/10; //check this value, this value is for ERP42
		double l_center = 0.0;
		double l_heading = 0.0;
		double l_width = 10.0;
		double v_width = 1.16;
		double d = 10;
		int nxy = 10;
		double cost=0;
		double vc = v;// this need to ask, can private variable use in private function?? can!!!
		vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy, vc);
		vector< vector<double> > result = generate_path_costmap(states, k0,vc, local_map);
		double mind = 1e9;
		vector <double> vm;
		printf("result size : %ld\n", result.size());
		for (int i = 0; i < result.size(); i++) {
			vector<double> table = result[i];
			double d = sqrt(table[0]*table[0]+table[1]*table[1]) + con * table[6];
			if(d<mind){
				mind = d;
				vm = table;
			}
		}
		vector<double> xt;
		vector<double> yt;
		vector<double> yawt;
		generate_trajectory(vm[3], vm[4], vm[5], k0,xy,yt,yawt,vc/3.6);

		for(int i=0; i<xt.size i++){
			x.data.push_back(xt[i]);
			y.data.push_back(yt[i]);
		}

	}
	/*rclcpp::Subscription<std_msgs::msg::String>::SharedPtr artiv_steer_sub;
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr artiv_state_sub;
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr artiv_location_sub;
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lidar_obstracle_sub;
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destination_sub;
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr HD_map_line_sub;
	  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::
	  */
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, arcv);
	rclcpp::spin(std::make_shared<Planning>());
	rclcpp::shutdown();
	return 0;
	/*vector< vector<double> > res = get_lookup_table();
	for(int i=0;i<10;i++,puts("")) for(int j=0;j<8;j++) printf("%lf ",res[i][j]);*/
}
