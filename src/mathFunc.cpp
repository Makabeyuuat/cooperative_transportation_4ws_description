#include <stdio.h>
#include <math.h>

#include "mathFunc.h"



//三角関数
double Cos(double x) {
	return(cos(x));
}

double Sin(double x) {
	return(sin(x));
}

double Tan(double x) {
	return(tan(x));
}

double ArcTan(double  x, double y) {
	return atan2(y, x);
}

double Sec(double x) {
	return(1 / cos(x));
}

//階乗
long long int factorial(double n) {
	long long int ans;

	ans = 1;
	for (int i = 2; i <= n; i++) {
		ans *= i;
	}
	return(ans);
}

double Power(double x, double y) {

	return(pow(x, y));
}

double Sqrt(double x) {
	return(sqrt(x));
}


//逆行列計算
double inv(double x[3][3], double y[3][3]) {
	double det;

	det = (x[0][0] * x[1][1] * x[2][2] + x[0][1] * x[1][2] * x[2][0] + x[0][2] * x[1][0] * x[2][1] - x[0][2] * x[1][1] * x[2][0] - x[0][1] * x[1][0] * x[2][2] - x[0][0] * x[1][2] * x[2][1]);

	if (det == 0) {
		return(0);
	}
	else {
		y[0][0] = (x[1][1] * x[2][2] - x[1][2] * x[2][1]) / det;
		y[0][1] = -(x[0][1] * x[2][2] - x[0][2] * x[2][1]) / det;
		y[0][2] = (x[0][1] * x[1][2] - x[0][2] * x[1][1]) / det;
		y[1][0] = -(x[1][0] * x[2][2] - x[1][2] * x[2][0]) / det;
		y[1][1] = (x[0][0] * x[2][2] - x[0][2] * x[2][0]) / det;
		y[1][2] = -(x[0][0] * x[1][2] - x[0][2] * x[1][0]) / det;
		y[2][0] = (x[1][0] * x[2][1] - x[1][1] * x[2][0]) / det;
		y[2][1] = -(x[0][0] * x[2][1] - x[0][1] * x[2][0]) / det;
		y[2][2] = (x[0][0] * x[1][1] - x[0][1] * x[1][0]) / det;

		return(1);
	}
}
