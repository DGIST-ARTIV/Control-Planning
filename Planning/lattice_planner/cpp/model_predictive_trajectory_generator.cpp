#include<bits/stdc++.h>
double mapk[500][500];
int max_iter = 50;
double h[3]={0.5, 0.02, 0.02}; //check
double cost_th = 0.1;

//p is ve
using namespace std;

double L = 1.0;
double ds = 0.2;
double v = 5.0/3.6;
double PI = acos(-1);

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

vector<double> generate_last_state(double s, double km, double kf, double k0){
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

///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

double* calc_diff1(double *target, vector<double>& x, vector<double>& y, vector<double>& yaw){

	double d[3] = {target[0] - x[x.size()-1], target[1] - y[y.size()-1], target[2] - yaw[yaw.size()-1]};
	double *pd = d;
	return pd;
}


double* calc_diff2(double *target,double x,double y, double yaw){

	double d[3] = {target[0] - x, target[1] - y, target[2] - yaw};
	double *pd = d;
	return pd;
}


void carc_j(double *target, double *p, double *h, double k0, double (*J)[3]){
	double xp,xn;
	double yp, yn;
	double yawp, yawn;
	double *dp, *dn;
	double d1[3], d2[3], d3[3];
	
	xp = generate_last_state(p[0]+h[0], p[1], p[2], k0)[0];
	yp = generate_last_state(p[0]+h[0], p[1], p[2], k0)[1];
	yp = generate_last_state(p[0]+h[0], p[1], p[2], k0)[2];
	dp = calc_diff2(target, xp, yp, yawp);

    xn = generate_last_state(p[0]-h[0], p[1], p[2], k0)[0];
	yn = generate_last_state(p[0]-h[0], p[1], p[2], k0)[1];
	yn = generate_last_state(p[0]-h[0], p[1], p[2], k0)[2];
	dn = calc_diff2(target, xn, yn, yawn);
	
	for(int i; i<3; i++)	{
		d1[i] = (dp[i] - dn[i])/(2.0 * h[0]);
	}

	xp = generate_last_state(p[0], p[1]+h[1], p[2], k0)[0];
	yp = generate_last_state(p[0], p[1]+h[1], p[2], k0)[1];
	yp = generate_last_state(p[0], p[1]+h[1], p[2], k0)[2];
	dp = calc_diff2(target, xp, yp, yawp);

    xn = generate_last_state(p[0], p[1]-h[1], p[2], k0)[0];
	yn = generate_last_state(p[0], p[1]-h[1], p[2], k0)[1];
	yn = generate_last_state(p[0], p[1]-h[1], p[2], k0)[2];
	dn = calc_diff2(target, xn, yn, yawn);
	
	for(int i; i<3; i++){
		d2[i] = (dp[i] - dn[i])/(2.0 * h[1]);
	}

	xp = generate_last_state(p[0], p[1], p[2]+h[2], k0)[0];
	yp = generate_last_state(p[0], p[1], p[2]+h[2], k0)[1];
	yp = generate_last_state(p[0], p[1], p[2]+h[2], k0)[2];
	dp = calc_diff2(target, xp, yp, yawp);

    xn = generate_last_state(p[0], p[1], p[2]-h[2], k0)[0];
	yn = generate_last_state(p[0], p[1], p[2]-h[2], k0)[1];
	yn = generate_last_state(p[0], p[1], p[2]-h[2], k0)[2];
	dn = calc_diff2(target, xn, yn, yawn);
	
	for(int i; i<3; i++)	{
		d3[i] = (dp[i] - dn[i])/(2.0 * h[2]);
	}

	for(int i; i<3; i++)	{
		J[i][0] = d1[i];
		J[i][1] = d2[i];
		J[i][2] = d3[i];
	}
	
    return ;

}

double selection_learning_param( double *dp, double *p, double k0, double *target){
	double mincost = 100000000.0;
	double mina = 1.0;
	double maxa = 2.0;
	double da = 0.5;
	double a[3] = {1.0, 1.5,2.0};
	for (int i; i<3; i++)	{
		double tp[3];
		for(int i; i<3; i++)		{
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
    double determinant = a[0][0]*a[1][1]*a[2][2] - a[0][0]*a[1][2]*a[2][1]
                + a[0][1]*a[1][2]*a[2][0] - a[0][1]*a[1][0]*a[2][2]
                + a[0][2]*a[1][0]*a[2][1] - a[0][2]*a[1][1]*a[2][0];
    if(determinant==0.0) {
        std::cout<<"\nNo inverse matrix exists.\n"<<std::endl;
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

void optimize_trajectory(double *target,double k0, double *p, double *map, double *xp, double *yp double *yawp){
	for(int i; i<max_iter; i++){
		vector<double> x;
		vector<double> y;
		vector<double> yaw;
		generate_trajectory(p[0],p[1],p[2],k0,x,y,yaw);
		double *dc;
		dc = calc_diff2(target, x[x.size()-1], y[y.size()-1],yaw[yaw.size()-1]);
		double cost1 = sqrt(dc[0]*dc[0]+dc[1]*dc[1]+dc[2]*dc[2]);
		if(cost1<=cost_th){
			cout<<"find_good_path"<<endl;
			break;
		}
		double J[3][3], invJ[3][3],dp[3];
		int chekk;
		try{
			carc_j(target,p,h,k0, J);
			chekk = f_inv(J, invJ);
			if (chekk);
				throw chekk;
			for(int i; i<3; i++){
				dp[i] = -invJ[i][0]*dc[0]-invJ[i][1]*dc[1]-invJ[i][2]*dc[2];
			}
		}
		catch(int a){
			cout<<"cannot calc path LinAlgError"<<endl;
			return ;
		}
		double alpha = selection_learning_param(dp,p,k0,target);
		for (int i; i<3; i++){
			p[i] += alpha*dp[i];	
		}

		// it need to check return argument
		
	}
	xp = x[x.size()-1];
	yp = y[y.size()-1];
	yawp = yaw[yaw.size()-1];

	return;
}

