#ifndef mptg
#define mptg
#include<bits/stdc++.h>

double mapk[500][500];
int max_iter = 50;
double h[3]={0.5, 0.02, 0.02}; //check
double cost_th = 0.1;


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
	vector<double> k;
	k = generate_last_state(p[0] + h[0], p[1], p[2], k0,v);
	xp = k[0];
	yp = k[1];
	yawp = k[2];
	dp = calc_diff2(target, xp, yp, yawp);
	k = generate_last_state(p[0] - h[0], p[1], p[2], k0,v);
	xn = k[0];
	yn = k[1];
	yawn = k[2];
	dn = calc_diff2(target, xn, yn, yawn);
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
	vector<double> b;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			tp[j] = p[j] + a[i] * dp[j];
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
#indef
