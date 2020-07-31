#include <bits/stdc++.h>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <tracking_msg/TrackingObjectArray.h>

//#include"matplotlib-cpp-master/matplotlibcpp.h"
//namespace plt = matplotlibcpp;

using namespace std;

double mapk[500][500];
 int max_iter = 50;
 double h[3] = { 0.5, 0.02, 0.02 }; //check
 double cost_th = 0.15;
 double con = 0.1;// map cost constant
//p is ve0
 double L = 1.0;
 double ds = 0.2;
 double PI = acos(-1);
ros::Publisher x_pub;
ros::Publisher y_pub;
struct State {
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double v = 0.0;
};
bool check_map(double(*map)[500], double x, double y) {
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
void generate_trajectory(double s, double km, double kf, double k0, vector<double>& x, vector<double>& y, vector<double>& yaw, double v){
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
double generate_trajectory_costmap(double s, double km, double kf, double k0, double (*map)[500], vector<double>& x, vector<double>& y, vector<double>& yaw, double v){
	double n = s/ds;//야 계산중 초기화가 안된값이 안들어 감
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
	double dt = time/n;
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
		k = check_map(map, state.x, state.y);
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
double* generate_last_state(double s, double km, double kf, double k0, double v) {

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
void carc_j(State target, double* p, double* h, double k0, double(*j)[3],double v) {
	double xp, xn;
	double yp, yn;
	double yawp, yawn;
	vector<double> dn, dp;
	double d1[3], d2[3], d3[3];
	double *k;


	k = generate_last_state(p[0] + h[0], p[1], p[2], k0,v);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
  printf("dp : %lf %lf %lf\n", dp[0], dp[1],dp[2]);
	k = generate_last_state(p[0] - h[0], p[1], p[2], k0,v);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
  printf("dn : %lf %lf %lf\n", dn[0], dn[1],dn[2]);

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
    printf("j %lf %lf %lf\n", d1[i], d2[i],d3[i]);
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
	double* b;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			tp[j] = p[j] + a[j] * dp[j];
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
double optimize_trajectory_costmap(State target, double k0, double* p, double* xp, double* yp, double* yawp, double v, double (*map)[500]) {
	for (int i = 0; i < max_iter; i++) {
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		double cost = generate_trajectory_costmap(p[0], p[1], p[2], k0, map, x, y, yaw, v);
		vector<double> dc;
		dc = calc_diff2(target, x[x.size() - 1], y[y.size() - 1], yaw[yaw.size() - 1]);
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
			carc_j(target, p, h, k0, j,v);
    			chekk = f_inv(j, invJ);
      for(int i=0; i<3; i++){
        printf("inv %lf %lf %lf\n", invJ[i][0],invJ[i][1],invJ[i][2]);
      }
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
		double alpha = selection_learning_param(dpp, p, k0, target, v);
		for (int i = 0; i < 3; i++) {
			p[i] += alpha * dp[i];
		}
		// it need to check return argument
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
void lidar_on_map(float (*lidar)[4], double(*map)[500], int type = 0) {
	cout << "lidar_on_map" << endl;
	int m0 = 1e9, M0 = -1;
	int m1 = 1e9, M1 = -1;
	for (int i = 0; i < 4; i++) {
		m0 = min(m0, int(lidar[0][i]));
		m1 = min(m1, int(lidar[1][i]));
		M0 = max(M0, int(lidar[0][i]));
		M1 = max(M1, int(lidar[1][i]));
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
	fpi = fopen("/home/vision/rs1_ws/src/local_path_planner/src/lookuptable.csv", "r");
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
	double mind = 1e9;
	int minid = -1;
	int first = 0;
	int k = lookup_table.size();
	int last = k;
  double dv = double(round(v));
	for (int i = k - 1; i >= 0; i--) {
    cout<<lookup_table[i][0]<<endl;
		if (lookup_table[i][0] ==dv) {
      cout<<"check"<<endl;
			last = i + 1;
			for (int j = i; j >= 0; j--) {
				if (lookup_table[j][0] == dv - 1) {
          cout<<"check"<<endl;
					first = j;
          break;
				}
			}
		}
    break;
	}
	printf("first last : %d %d %lf\n", first, last, dv);
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
  cout<<"minid : "<<minid<<endl;
	return lookup_table[minid];
}
vector< vector<double> > generate_path_costmap(vector< vector<double> >& target_states, double k0, double v, double (*map)[500],vector< vector<double> > &lookup_table) {
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
		cost = optimize_trajectory_costmap(target, k0, init_p, &x, &y, &yaw, v/3.6, map);// km/h -> m/s check!!
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
vector< vector<double> > calc_lane_states(double l_center, double l_heading, double l_width, double d, double v) {
	double dx = sin(l_heading) * l_width/20;
	double dy = cos(l_heading) * l_width/20;
	double y0 = l_center;
	double x0 = d;
	double yawf = l_heading;
	vector< vector<double> > states;
  vector<double> x;
  vector<double> y;
	for (int i = 0; i < 19; i++) {
		double xf = x0 + (4-i)*dx;
		double yf = y0 - (4-i)*dy;
		vector<double> t;
		t.push_back(v);
		t.push_back(xf);
    x.push_back(xf);
    y.push_back(yf);
		t.push_back(yf);
		t.push_back(yawf);
		states.push_back(t);
	}
  //plt::plot(x,y,"-g");
  //plt::show();
	return states;
}
void lane_state_sampling_test1(){
		cout<<"lane_state_sampling_test1"<<endl;
			clock_t start= clock();
    double k0 = 0;
    double l_center = 0.0;
    double l_heading = 0.0;
    double l_width = 10.0;
    double v_width = 1.16;
    double d0 = 10;
    vector< vector <double> > lookup_table = get_lookup_table();
		double v=10;
    double local_map[500][500];
    for(int i=0; i<500; i++){
      for(int j=0; j<500; j++)  local_map[j][i]=0;
    }
    vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width,  d0, v);
    vector< vector<double> > result = generate_path_costmap(states, k0, v, local_map, lookup_table);
		double mind = 1e9;
		printf("result size : %ld\n", result.size());
    for(int i=0;i<result.size();i++){
    		vector<double> table = result[i];
				vector<double> x;
				vector<double> y;
				vector<double> yaw;
        generate_trajectory(table[3],table[4],table[5],k0,x,y,yaw,v/3.6);
        //plt::plot(x,y,"-r");
				printf("\n");
    }
		clock_t end= clock();
		printf("calc time: %lf\n",double(end-start)/CLOCKS_PER_SEC);
		//plt::show(0.01);
}
class Planning {
public:
	ros::Subscriber state_sub;
	ros::Subscriber destination_sub;
  ros::Subscriber location_sub;
  ros::Subscriber lidal_sub;

	double v{5}, steer{0};
	double x_current = 0;
  double y_current = 0;
	vector< vector<double> > lookup_table;
	double xd[3], yd[3];
	vector< vector < float> >  lidar;
	double local_map[500][500];
	void reset_map(double (*map)[500]){
		for(int i=0; i<500; i++){
			for(int j=0; j<500; j++){map[j][i] = 0;}
		}
	}

	Planning(ros::NodeHandle * nodeHandle){
		lookup_table = get_lookup_table();
		reset_map(local_map);
		ros::NodeHandle nh(*nodeHandle);
		x_pub = nh.advertise<std_msgs::Float32MultiArray>("/local_path/x", 1);
		y_pub = nh.advertise<std_msgs::Float32MultiArray>("/local_path/y", 1);

		state_sub = nh.subscribe("/Ioniq_info", 1, &Planning::state_callback, this);
	  lidal_sub = nh.subscribe("/tracking/tracking_objects", 0, &Planning::lidar_obstracle_callback, this);
		//ros::Subscriber line_sub = nh.subscribe("/", 1, line_callback);
		location_sub = nh.subscribe("/utm_fix", 1, &Planning::location_callback, this);
		destination_sub = nh.subscribe("/destination_info",1,&Planning::destination_callback, this);
	}

	void state_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	//	cout<<"state_callback!!!"<<endl;
		v = msg->data[23];
		steer = msg->data[3];

    cout<<v<<", "<<steer<<endl;
	}
/*	void lidar_obstracle_callback(const tracking_msg::TrackingObjectArray& msg) {
		int size = msg.size;
    cout<<"get lidar, size : "<<size<<endl;
		cout<<"size:"<<sizeof(msg.array[0].bev.data)<<endl;

		float l[2][4];
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < 4; j += 1) {
				l[0][j] = msg.array[i].bev.data[j * 2]*10;
				l[1][j] = msg.array[i].bev.data[j * 2 + 1]*10;
        printf("%f %f ", l[0][j], l[1][j]);
			}
      cout<<endl;
			lidar_on_map(l, local_map);
		}
	}*/
	void location_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)	{
    cout<<"get current location"<<endl;
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    cout<<x_current<<","<<y_current<<endl;
	}
	void destination_callback(const std_msgs::Float32MultiArray& msg)	{
    cout<<"make path"<<endl;
		float d[2][2];

    d[0][0] = 100;
    d[0][1] = 0;
    d[1][0] = 200;
    d[1][1] = 50;
		std_msgs::Float32MultiArray x;
		std_msgs::Float32MultiArray y;
		double det = d[0][0]*d[0][0]*d[1][0]-d[0][0]*d[1][0]*d[1][0];
		double inv[2][2] = {{d[1][0],-d[0][0]},{-d[1][0]*d[1][0], d[0][0]*d[0][0]}};
		double A = (inv[0][0]*d[0][1]+inv[0][1]*d[1][1])/det;
		double B = (inv[1][0]*d[0][1]+inv[1][1]*d[1][1])/det;
		//main callback do path planning
		// here will set destination
		double k0 = steer/100; //check this value, this value is for ERP42
		double l_center = A*100+B*10;
		double l_heading = atan(2*A*10+B);
		double l_width = 10.0;
		double v_width = 1.16;
		double d0 = 20;
		int nxy = 10;
		double cost=0;
		// this need to ask, can private variable use in private function?? can!!!
		vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width,  d0, v);
		vector< vector<double> > result = generate_path_costmap(states, k0,v, local_map, lookup_table);
		double mind = 1e9;
		vector <double> vm;
		printf("result size : %ld\n", result.size());
    if(result.size()==0){
      cout<<"no path generated"<<endl;
      x.data.push_back(1);
      x_pub.publish(x);
      return;

    }
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
		generate_trajectory(vm[3], vm[4], vm[5], k0,xt,yt,yawt,v/3.6);
		for(int i=0; i<xt.size(); i++){
			x.data.push_back(xt[i]);
			y.data.push_back(yt[i]);
		}
		reset_map(local_map);
		x_pub.publish(x);
		y_pub.publish(y);
 }
};
int main(int argc, char* argv[]) {
	cout<<"main"<<endl;
	ros::init(argc, argv, "local_path_planner");
	ros::NodeHandle nh;
	Planning plan(&nh);
	ros::spin();
//  lane_state_sampling_test1();
	return 0;
}
