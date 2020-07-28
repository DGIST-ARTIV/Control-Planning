#include<bits/stdc++.h>
double mapk[500 ][500];
int max_iter = 50;
double h[3]={0.5, 0.02, 0.02}; //check
double cost_th = 0.1;

//p is ve
using namespace std;

double L = 1.0;
double ds = 0.2;
double v = 10.0;// 여기 다시 /3.6으로 바꾸기.
double PI = acos(-1);

struct State{
	double x = 0.0;
	double y = 0.0;
	double yaw = 0.0;
	double v = 0.0;
};

bool check_map(double (*map)[500], double x, double y){
	cout<<"check_map"<<endl;
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
	cout<<"generate_trajectory"<<endl;

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

double* generate_last_state(double s, double km, double kf, double k0){
	double n = s/ds;
	double time = s/v;
	double cost = 0;
	vector<double> x; vector<double> y; vector<double> yaw;
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
	double res[3];
	res[0] = state.x;
	res[1] = state.y;
	res[2] = state.yaw;
	double *m=res;
	return m;
}

double* calc_diff1(State target, vector<double>& x, vector<double>& y, vector<double>& yaw){
	cout<<"calc_diff1"<<endl;
	double d[3] = {target.x - x[x.size()-1], target.y - y[y.size()-1], target.yaw - yaw[yaw.size()-1]};
	double *pd = d;
	return pd;
}


double* calc_diff2(State target,double x,double y, double yaw){
	double d[3] = {target.x - x, target.y - y, target.yaw - yaw};
	double *pd = d;
	return pd;
}


void carc_j(State target, double *p, double *h, double k0, double (*j)[3]){
	cout<<"carc_j"<<endl;
	double xp,xn;
	double yp, yn;
	double yawp, yawn;
	double *dp, *dn;
	double d1[3], d2[3], d3[3];
	double *k;
	k = generate_last_state(p[0]+h[0], p[1], p[2], k0);

	xp = k[0];
	yp = k[1];
	yawp = k[2];

	printf("%lf %lf %lf\n",xp,yp,yawp);


	dp = calc_diff2(target, xp, yp, yawp);

	k = generate_last_state(p[0]-h[0], p[1], p[2], k0);
  xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);

	for(int i=0; i<3; i++)	{
		d1[i] = (dp[i] - dn[i])/(2.0 * h[0]);
	}

  k= generate_last_state(p[0], p[1]+h[1], p[2], k0);
	xp = k[0];
	yp = k[1];
	yawp = k[2];

	printf("%lf %lf %lf\n",xp,yp,yawp);

	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1]-h[1], p[2], k0);
  xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);

	for(int i=0; i<3; i++){
		d2[i] = (dp[i] - dn[i])/(2.0 * h[1]);
	}
	k = generate_last_state(p[0], p[1], p[2]+h[2], k0);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
		printf("%lf %lf %lf\n",xp,yp,yawp);
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0], p[1], p[2]-h[2], k0);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);

	for(int i=0; i<3; i++)	{
		d3[i] = (dp[i] - dn[i])/(2.0 * h[2]);
	}


	for(int i=0; i<3; i++)	{
		j[i][0] = d1[i];
		j[i][1] = d2[i];
		j[i][2] = d3[i];
		printf("%lf %lf %lf\n",j[i][0],j[i][1],j[i][2]);

	}

    return ;

}

double selection_learning_param( double *dp, double *p, double k0, State target){
	cout<<"selection_learning_param"<<endl;
	double mincost = 100000000.0;
	double mina = 1.0;
	double maxa = 2.0;
	double da = 0.5;
	double a[3] = {1.0, 1.5,2.0};
	for (int i=0; i<3; i++)	{
		double tp[3];
		for(int i=0; i<3; i++)		{
			tp[i]=p[i]+a[i]*dp[i];
		}
		double x = generate_last_state(p[0], p[1], p[2], k0)[0];
		double y = generate_last_state(p[0], p[1], p[2], k0)[1];
		double yaw = generate_last_state(p[0], p[1], p[2], k0)[2];
		double *dc;
		dc = calc_diff2(target,x,y,yaw);
		double cost = sqrt(dc[0]*dc[0]+dc[1]*dc[1]+dc[1]*dc[1]);
		if (cost<=mincost && a[i]!=0.0)		{
			mina = a[i];
			mincost = cost;
		}

	}
	return mina;
}

int f_inv(double (*a)[3], double (*inv)[3]){
		for (int i=0; i<3; i++){
			printf("%lf %lf %lf\n", a[i][0], a[i][1],a[i][2]);
		}
		cout<<"f_inv"<<endl;
    double determinant = a[0][0]*a[1][1]*a[2][2] - a[0][0]*a[1][2]*a[2][1]
                + a[0][1]*a[1][2]*a[2][0] - a[0][1]*a[1][0]*a[2][2]
                + a[0][2]*a[1][0]*a[2][1] - a[0][2]*a[1][1]*a[2][0];
    if(determinant==0.0) {
        cout<<"\nNo inverse matrix exists.\n"<<endl;
        return 1;
    }
    for(int i=0;i<3;i+=1) {
        for(int j=0;j<3;j+=1) {
            inv[i][j]=1.0/determinant*
                (a[(i+1)%3][(j+1)%3]*a[(i+2)%3][(j+2)%3] - a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]);
        }
    }
    return 0;
}

void optimize_trajectory(State target,double k0, double *p, double *xp, double *yp, double *yawp){
	cout<<"optimize_trajectory"<<endl;
	for(int i=0; i<50; i++){
		cout<<"check : "<<i<<endl;
		vector<double> x;
		vector<double> y;
		vector<double> yaw;

		generate_trajectory(p[0],p[1],p[2],k0,x,y,yaw);


		double *dc;
		dc = calc_diff2(target, x[x.size()-1], y[y.size()-1],yaw[yaw.size()-1]);
		printf("%lf %lf %lf",dc[0],dc[1],dc[2]);
		double cost1 = sqrt(dc[0]*dc[0]+dc[1]*dc[1]+dc[2]*dc[2]);
		if(cost1<=cost_th){
			cout<<"find_good_path"<<endl;
			*xp=x[x.size()-1];
			*yp=y[y.size()-1];
			*yawp=yaw[yaw.size()-1];
			return;
		}
		double j[3][3], invJ[3][3],dp[3];

		int chekk;
		try{
			carc_j(target,p,h,k0,j);
			for(int i=0;i<3; i++){
				printf("%lf %lf %lf\n", j[i][0],j[i][1],j[i][2]);
			}
			chekk = f_inv(j, invJ);
			if (chekk) {throw chekk;}
			for(int i=0; i<3; i++){
				dp[i] = -invJ[i][0]*dc[0]-invJ[i][1]*dc[1]-invJ[i][2]*dc[2];
			}
		}
		catch(int a){
			*xp=0;
			cout<<"cannot calc path LinAlgError"<<endl;
			return ;
		}
		double alpha = selection_learning_param(dp,p,k0,target);
		for (int i=0; i<3; i++){
			p[i] += alpha*dp[i];
		}
		// it need to check return argument


	}
	*xp=0;
	cout<<"can't find path"<<endl;
	return;
}
double map[500][500];

int k_min(double a, double b){
	cout<<"k_min"<<endl;
	if (a>=b) return int(b);
	else return int(a);
}

int k_max(double a, double b){
	cout<<"k_max"<<endl;
	if (a<=b) return int(b);
	else return int(a);
}


void line_on_map(double (*line)[3], double (*map)[500], int type = 0){
	cout<<"line_on_map"<<endl;
	double A, B, C;

	double a[3][3] = {
		{line[0][0]*line[0][0], line[0][0], 1.0},
		{line[0][1]*line[0][1], line[0][1], 1.0},
		{line[0][2]*line[0][2], line[0][2], 1.0}
	};

	double inv[3][3];

	if(f_inv(a,inv)==1){
		printf("interpolation fail\n");
		return;
	}

	A = inv[0][0] * line[1][0] + inv[0][1] * line[1][1] + inv[0][2] * line[1][2];
	B = inv[1][0] * line[1][0] + inv[1][1] * line[1][1] + inv[1][2] * line[1][2];
	C = inv[2][0] * line[1][0] + inv[2][1] * line[1][1] + inv[2][2] * line[1][2];

	double k1 =line[0][0];
	double k2 =line[0][2];
	for(int x = k_max(k1,0); x< k_min(k2,500); x++){
		double tmp = A*x*x + B*x + C;
		int y = 250 - int(tmp);
		map[y][x] = 1;
	}
}

void lidar_on_map(int (*lidar)[4], double (*map)[500], int type=0){
	cout<<"lidar_on_map"<<endl;
	int m0 = 1e9, M0 = -1;
	int m1 = 1e9, M1 = -1;
	for(int i=0;i<4;i++){
		m0 = min(m0, lidar[0][i]);
		m1 = min(m1, lidar[1][i]);
		M0 = max(M0, lidar[0][i]);
		M1 = max(M1, lidar[1][i]);
	}
	for(int i = m0; i<=M0; i++){
		for(int j=m1; j<=M1; j++){
			int y = 250-j;
			int x = i;
			map[y][x] = 1;
		}
	}
}

vector< vector<double> > get_lookup_table(){
	cout<<"get_lookup_table"<<endl;
	FILE *fpi;
	fpi = fopen("lookuptable.csv", "r");
	vector< vector<double> > res;
	char tmp[300];
	for(int i=0;i<7;i++){
		fscanf(fpi,"%s",tmp);

	}
	int cnt =1;
	while(1){
		cnt++;
		double v0, k0, x, y, yaw, km, kf, sf;
		fscanf(fpi, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",&v0,&x,&y,&yaw,&sf,&km,&kf);
		if(feof(fpi)) break;
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
	cout<<cnt<<endl;
	fclose(fpi);
	return res;
}

vector<double> search_nearest_one_from_lookuptable(double tx, double ty, double tyaw, vector< vector<double> >& lookup_table, double v){
	cout<<"search_nearest_one_from_lookuptable"<<endl;
	double mind = 1e9;
	int minid = -1;
	int first = 0;
	int k = lookup_table.size();
	int last = k;
	for(int i = k-1; i>=0; i--){
		double dv = round(lookup_table[i][0]);
		if (dv == v){
			last == i+1;
			for(int j=i; j>=0; j--){
					if(lookup_table[j][0]==v-1){
						first=j;
					}
			}
		}
		break;
	}
	cout<<"first : "<<first<<endl;
	cout<<"last : "<<last<<endl;
	for(int i=first;i<last;i++){
		double dx = tx - lookup_table[i][1];
		double dy = ty - lookup_table[i][2];
		double dyaw = tyaw - lookup_table[i][3];
		double d = sqrt(dx*dx + dy*dy + dyaw*dyaw);
		if( d <= mind ){
			minid = i;
			mind = d;
		}
	}
	cout<<minid<<endl;
	return lookup_table[minid];
}

vector< vector<double> > generate_path(vector< vector<double> >& target_states, double k0){
	vector< vector<double> > lookup_table = get_lookup_table();
	vector< vector<double> > result;
	cout<<"generate_path"<<endl;

	int k = target_states.size();
	for(int i=0; i<k; i++){
		cout<<"check : "<<i<<endl;
		vector<double> bestp;
		double tx, ty, tyaw,tv;
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
		double init_p[3] = {sqrt(tx*tx + ty*ty), bestp[5], bestp[6]};
		double x,y,yaw,cost;
		double p[3];
		optimize_trajectory(target, k0, init_p, &x, &y, &yaw);
		vector< vector<double> > result;
		if(x){
			cout<<"find_good_path"<<endl;
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

vector< vector<double> > calc_lane_states(double l_center, double l_heading, double l_width, double v_width, double d, int nxy, double v){
		cout<<"calc_lane_states"<<endl;
		double xc = cos(l_heading) * d + sin(l_heading) * l_center;
    double yc = sin(l_heading) * d + cos(l_heading) * l_center;

    vector< vector<double> > states;

    for(int i=0;i<nxy;i++){
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

void lane_state_sampling_test1(){
		cout<<"lane_state_sampling_test1"<<endl;
    double k0 = 0;
    double l_center = 0.0;
    double l_heading = 0.0;
    double l_width = 10.0;
    double v_width = 1.16;
    double d = 10;
    int nxy = 3;
		double v=10;
    vector< vector<double> > states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy,v);
		cout<<states.size()<<endl;
    vector< vector<double> > result = generate_path(states, k0);

		double mind = 1e9;

    for(int i=0;i<result.size();i++){
    		vector<double> table = result[i];

/*
        d = math.sqrt((sample_des[0] - table[0][-1])**2+(sample_des[1] - table[1][-1])**2)
        if d<mind:
            mind= d
            x = table[0]
            y = table[1]

*/
    }
}

int main(){

	lane_state_sampling_test1();

	/*vector< vector<double> > res = get_lookup_table();
	for(int i=0;i<10;i++,puts("")) for(int j=0;j<8;j++) printf("%lf ",res[i][j]);*/
}
