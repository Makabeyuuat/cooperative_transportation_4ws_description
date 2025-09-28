#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cfloat>
#include <time.h>
#include <vector>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mathFunc.h"		// 数学関数のヘッダーファイル
#include "Bezier.h"			// Bezier 曲線の関数
#include "vehicle_ct.hpp"       // Vehicle クラスの宣言
#include "callback_ct.hpp"     // コールバック関数の宣言
#include "initial.hpp"		// 初期値設定のヘッダーファイル
#include "DynamicsCalc.hpp"
#include "getInputValue.hpp"
#include "csvLogger.hpp"	// グローバル変数のヘッダーファイル
#include "wheel_kinematics.hpp"

using namespace std;



// void rungeS();
// void initials(double*, double*, double*);
//double	fs0(double*), fs1(double*);

//static FUNC fs[2] = { fs0, fs1 };


Search searchP(const std::vector<double>&);
Search searchPP(const std::vector<double>&);


int main(int argc, char** argv)
{

	ros::init(argc, argv, "cooperative_transport_node");
    ros::NodeHandle nh;
	ros::Rate loop_rate(100);

    // callback.cppで定義している各コールバックを購読
	ros::Subscriber joint_state_sub    = nh.subscribe("/cooperative_transportation_4ws/joint_states", 10, jointStateCallback);
    ros::Subscriber sub_base = nh.subscribe("/cooperative_transportation_4ws/true_base_link", 10, trueBaseLinkCallback);
    ros::Subscriber sub_carrier = nh.subscribe("/cooperative_transportation_4ws/true_carrier", 10, trueCarrierCallback);
    ros::Subscriber sub_linkage1 = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point1", 10, trueLinkagePoint1Callback);
    ros::Subscriber sub_linkage2 = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point2", 10, trueLinkagePoint2Callback);
    ros::Subscriber sub_linkage3 = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point3", 10, trueLinkagePoint3Callback);
	ros::Subscriber true_v1_front_sub = nh.subscribe("/cooperative_transportation_4ws/true_v1_front", 10, trueV1FrontLeftSteeringCallback);
	ros::Subscriber true_v2_front_sub = nh.subscribe("/cooperative_transportation_4ws/true_v2_front", 10, trueV2FrontLeftSteeringCallback);
	ros::Subscriber true_v3_front_sub = nh.subscribe("/cooperative_transportation_4ws/true_v3_front", 10, trueV3FrontLeftSteeringCallback);

	
    // 車両ごとのインスタンス生成（車両IDはvehicle.cppで用いるIDに対応）
    Vehicle vehicle1(nh, "v1");
    Vehicle vehicle2(nh, "v2");
    Vehicle vehicle3(nh, "v3");
	// 各クラスをインスタンス化
    DynamicsCalculator dynamics_calc;
	getInputValue getInputValue(0.02);

	//データファイル作成
  	std::string pkg = ros::package::getPath("cooperative_transportation_4ws_description");
  	std::string data_dir = pkg + "/data";

  	CSVLogger logger(data_dir, 100000);



	time_t current_time;
	current_time = time(NULL);
	printf("%s", ctime(&current_time));

	double	h;
	long	i, j, n, time;

	//gazebo上の初期値をx_oldに代入
	while (ros::ok() && ! got_body_pos) {
		ros::spinOnce();
		loop_rate.sleep();
		//デバッグ用ログ出力
		ROS_INFO("Waiting for getting initial pose...");
	}
	//デバッグ用ログ出力
	// ROS_INFO("Locking initial pose and calling initial(): x=%.3f, y=%.3f, theta_1=%.3f",
    //        x_old[1], x_old[2], x_old[4]);
	

	//初期値を設定
	initial(t_max, h, x_old, x_new, x_input);

	//経路のQ分割
	s[0] = 0;
	for (int i = 1; i < Q_sample; i++) {
		qs[i] = qs[i - 1] + 1.0 / 100000.0;
		s[i] = sqrt(pow(d1Rxdq1(Bx, qs, i), 2) + pow(d1Rydq1(By, qs, i), 2)) - s[i - 1];

	}

	//曲率の値出力
	for (int i = 0; i < Q_sample; i++) {
		R[i][0] = Rx(Bx, qs, i);
		R[i][1] = Ry(By, qs, i);
		dRdq[i][0] = d1Rxdq1(Bx, qs, i);
		dRdq[i][1] = d1Rydq1(By, qs, i);
		d2Rdq2[i][0] = d2Rxdq2(Bx, qs, i);
		d2Rdq2[i][1] = d2Rydq2(By, qs, i);
		d3Rdq3[i][0] = d3Rxdq3(Bx, qs, i);
		d3Rdq3[i][1] = d3Rydq3(By, qs, i);
		d4Rdq4[i][0] = d4Rxdq4(Bx, qs, i);
		d4Rdq4[i][1] = d4Rydq4(By, qs, i);

		cs[i][0] = (-(d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i)) + d1Rxdq1(Bx, qs, i) * d2Rydq2(By, qs, i)) /
			Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 1.5);


		cs[i][1] = (Power(d1Rydq1(By, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) - d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			- Power(d1Rxdq1(Bx, qs, i), 2) * (3 * d2Rxdq2(Bx, qs, i) * d2Rydq2(By, qs, i) + d1Rydq1(By, qs, i) * d3Rxdq3(Bx, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * d3Rydq3(By, qs, i) + d1Rxdq1(Bx, qs, i) * d1Rydq1(By, qs, i) * (3 * Power(d2Rxdq2(Bx, qs, i), 2)
				- 3 * Power(d2Rydq2(By, qs, i), 2) + d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 3);

		cs[i][2] = (-(Power(d1Rxdq1(Bx, qs, i), 4) * (4 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
			+ 6 * d2Rxdq2(Bx, qs, i) * d3Rydq3(By, qs, i) + d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rxdq1(Bx, qs, i), 2) * d1Rydq1(By, qs, i) * (-15 * Power(d2Rxdq2(Bx, qs, i), 3)
				+ d2Rxdq2(Bx, qs, i) * (39 * Power(d2Rydq2(By, qs, i), 2) - 2 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i))
				+ 2 * d1Rydq1(By, qs, i) * (d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i) - d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i)))
			+ Power(d1Rydq1(By, qs, i), 3) * (3 * Power(d2Rxdq2(Bx, qs, i), 3) + d2Rxdq2(Bx, qs, i) * (-15 * Power(d2Rydq2(By, qs, i), 2)
				+ 4 * d1Rydq1(By, qs, i) * d3Rydq3(By, qs, i)) + d1Rydq1(By, qs, i) * (6 * d2Rydq2(By, qs, i) * d3Rxdq3(Bx, qs, i)
					- d1Rydq1(By, qs, i) * d4Rxdq4(Bx, qs, i))) + Power(d1Rxdq1(Bx, qs, i), 5) * d4Rydq4(By, qs, i)
			+ d1Rxdq1(Bx, qs, i) * Power(d1Rydq1(By, qs, i), 2) * (-39 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i)
				+ 15 * Power(d2Rydq2(By, qs, i), 3) + 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i)
				- 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i) + Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))
			+ Power(d1Rxdq1(Bx, qs, i), 3) * (15 * Power(d2Rxdq2(Bx, qs, i), 2) * d2Rydq2(By, qs, i) - 3 * Power(d2Rydq2(By, qs, i), 3)
				+ 10 * d1Rydq1(By, qs, i) * d2Rxdq2(Bx, qs, i) * d3Rxdq3(Bx, qs, i) - 10 * d1Rydq1(By, qs, i) * d2Rydq2(By, qs, i) * d3Rydq3(By, qs, i)
				+ 2 * Power(d1Rydq1(By, qs, i), 2) * d4Rydq4(By, qs, i))) / Power(Power(d1Rxdq1(Bx, qs, i), 2) + Power(d1Rydq1(By, qs, i), 2), 4.00000000005);
	}


	//rungeS();

	
	n = (long)(t_max / h) + 1;

	
	
	dynamics_calc.calcXold(x_old);

	//係数aの計算(ここでx_oldも計算)
	dynamics_calc.computeCoefficients(x_old);
	
	//全探索
	searchP(x_old);

    dynamics_calc.calculate(x_old, t_max, l1, l2, l3, sr.j);
	//制御入力
	getInputValue.getU(x_old, sr.j);
	getInputValue.rungeKutta(x_old, sr.j);
	//再度、車両の速度を計算
	dynamics_calc.computeCoefficients(x_old);

	// 例：v1f,v1r, Phi[1], x_old[10] が既知
    auto c1 = wheelkin::compute4ws_from_along(v1f, v1r, Phi[1], x_old[10], lv, lt, wheelRadius);
    auto c2 = wheelkin::compute4ws_from_along(v2f, v2r, Phi[2], x_old[16], lv, lt, wheelRadius);
    auto c3 = wheelkin::compute4ws_from_along(v3f, v3r, Phi[3], x_old[22], lv, lt, wheelRadius);
	// 各車両へ steering コマンドと車輪の回転速度コマンドを送信
    // vehicle1.publishSteeringCommand(Phi[1], Phi[1], x_old[10], x_old[10]);
    // vehicle2.publishSteeringCommand(Phi[2], Phi[2], x_old[16], x_old[16]);
    // vehicle3.publishSteeringCommand(Phi[3], Phi[3], x_old[22], x_old[22]);
    // vehicle1.publishWheelCommand(v1f, v1f, v1r, v1r);
    // vehicle2.publishWheelCommand(v2f, v2f, v2r, v2r);
    // vehicle3.publishWheelCommand(v3f, v3f, v3r, v3r);
	vehicle1.publishSteeringCommand(c1.delta_fl, c1.delta_fr, c1.delta_rl, c1.delta_rr);
    vehicle2.publishSteeringCommand(c2.delta_fl, c2.delta_fr, c2.delta_rl, c2.delta_rr);
    vehicle3.publishSteeringCommand(c3.delta_fl, c3.delta_fr, c3.delta_rl, c3.delta_rr);
    vehicle1.publishWheelCommand(c1.omega_fl, c1.omega_fr, c1.omega_rl, c1.omega_rr);
    vehicle2.publishWheelCommand(c2.omega_fl, c2.omega_fr, c2.omega_rl, c2.omega_rr);
    vehicle3.publishWheelCommand(c3.omega_fl, c3.omega_fr, c3.omega_rl, c3.omega_rr);

	logger.logData();
	
	//Gazeboのフィードバックをもとに計算
	while(ros::ok()) {
		double current_time = ros::Time::now().toSec();
        //double desired_steering = 0.5 * sin(current_time);
		//ROSのコールバックを処理
		ros::spinOnce();
	
		dynamics_calc.calcXold(x_old);

		//部分探索
		searchPP(x_old);
		//係数aの計算(ここでx_oldも計算)
		dynamics_calc.computeCoefficients(x_old);

		dynamics_calc.calculate(x_old, t_max, l1, l2, l3, sr.j);

	
		//制御入力を計算
		getInputValue.getU(x_old, sr.j);
		getInputValue.rungeKutta(x_old, sr.j);
		//再度、車両の速度を計算
		dynamics_calc.computeCoefficients(x_old);
	
		// 例：v1f,v1r, Phi[1], x_old[10] が既知
        auto c1 = wheelkin::compute4ws_from_along(v1f, v1r, Phi[1], x_old[10], lv, lt, wheelRadius);
        auto c2 = wheelkin::compute4ws_from_along(v2f, v2r, Phi[2], x_old[16], lv, lt, wheelRadius);
        auto c3 = wheelkin::compute4ws_from_along(v3f, v3r, Phi[3], x_old[22], lv, lt, wheelRadius);

		// 各車両へ steering コマンドと車輪の回転速度コマンドを送信
        // vehicle1.publishSteeringCommand(Phi[1], Phi[1], x_old[10], x_old[10]);
        // vehicle2.publishSteeringCommand(Phi[2], Phi[2], x_old[16], x_old[16]);
        // vehicle3.publishSteeringCommand(Phi[3], Phi[3], x_old[22], x_old[22]);

        // vehicle1.publishWheelCommand(v1f, v1f, v1r, v1r);
        // vehicle2.publishWheelCommand(v2f, v2f, v2r, v2r);
        // vehicle3.publishWheelCommand(v3f, v3f, v3r, v3r);


		vehicle1.publishSteeringCommand(c1.delta_fl, c1.delta_fr, c1.delta_rl, c1.delta_rr);
        vehicle2.publishSteeringCommand(c2.delta_fl, c2.delta_fr, c2.delta_rl, c2.delta_rr);
        vehicle3.publishSteeringCommand(c3.delta_fl, c3.delta_fr, c3.delta_rl, c3.delta_rr);

        vehicle1.publishWheelCommand(c1.omega_fl, c1.omega_fr, c1.omega_rl, c1.omega_rr);
        vehicle2.publishWheelCommand(c2.omega_fl, c2.omega_fr, c2.omega_rl, c2.omega_rr);
        vehicle3.publishWheelCommand(c3.omega_fl, c3.omega_fr, c3.omega_rl, c3.omega_rr);

		//デバッグ用ログ出力
		ROS_INFO_THROTTLE(0.1,"t+carrier:t=%.3f, x=%.3f, y=%.3f, theta0=%.3f, phi1=%.3f, theta1=%.3f",
           x_old[0], x_old[1], x_old[2], x_old[3], x_old[4], x_old[5]);

		ROS_INFO_THROTTLE(0.1,"vehicle1: phi2=%.3f, theta2=%.3f, phi3=%.3f, theta3=%.3f,phi4=%.3f, theta4=%.3f",
           x_old[6], x_old[7], x_old[8], x_old[9], x_old[10], x_old[11]);

		ROS_INFO_THROTTLE(0.1,"vehicle2: phi5=%.3f, theta5=%.3f, phi6=%.3f, theta6=%.3f, phi7=%.3f, theta7=%.3f",
           x_old[12], x_old[13], x_old[14], x_old[15], x_old[16], x_old[17]);

		ROS_INFO_THROTTLE(0.1,"vehicle3: phi8=%.3f, theta8=%.3f, phi9=%.3f, theta9=%.3f, phi10=%.3f, theta10=%.3f",
           x_old[18], x_old[19], x_old[20], x_old[21], x_old[22], x_old[23]);
		
		
		// ROS_INFO_THROTTLE(0.1, "sr: j=%d, Psx=%.3f, Psy=%.3f, d=%.3f, Cs=%.6f, dCs1=%.6f, dCs2=%.6f",
    	//     sr.j, sr.Psx, sr.Psy, sr.d, sr.Cs, sr.Cs1, sr.Cs2);

		ROS_INFO_THROTTLE(0.1, "v1f=%.6f, v1r=%.6f, v2f=%.6f, v2r=%.6f, v3f=%.6f, v3r=%.6f",
    	    v1f, v1r, v2f, v2r, v3f, v3r);
		
		// ROS_INFO_THROTTLE(0.1, "v1fl=%.6f, v1fr=%.6f, v1rl=%.6f, v1rr=%.6f",
    	//     c1.omega_fl, c1.omega_fr, c1.omega_rl, c1.omega_rr);
		// ROS_INFO_THROTTLE(0.1, "v2fl=%.6f, v2fr=%.6f, v2rl=%.6f, v2rr=%.6f",
		//     c2.omega_fl, c2.omega_fr, c2.omega_rl, c2.omega_rr);
		// ROS_INFO_THROTTLE(0.1, "v3fl=%.6f, v3fr=%.6f, v3rl=%.6f, v3rr=%.6f",
		//     c3.omega_fl, c3.omega_fr, c3.omega_rl, c3.omega_rr);

		// ROS_INFO_THROTTLE(0.1, "del1fl=%.6f, del1fr=%.6f, del1rl=%.6f, del1rr=%.6f",
    	//     c1.delta_fl, c1.delta_fr, c1.delta_rl, c1.delta_rr);
		// ROS_INFO_THROTTLE(0.1, "del2fl=%.6f, del2fr=%.6f, del2rl=%.6f, del2rr=%.6f",
		//     c2.delta_fl, c2.delta_fr, c2.delta_rl, c2.delta_rr);
		// ROS_INFO_THROTTLE(0.1, "del3fl=%.6f, del3fr=%.6f, del3rl=%.6f, del3rr=%.6f",
		//     c3.delta_fl, c3.delta_fr, c3.delta_rl, c3.delta_rr);
		

		ROS_INFO_THROTTLE(0.1, "Phi1=%.6f, Phi2=%.6f, Phi3=%.6f\n",
    	    Phi[1], Phi[2], Phi[3]);




		logger.logData();
		loop_rate.sleep();

	}
	return 0;
}





//経路長の計算
// void rungeS()
// {
// 	static double	ks[2][4], qs[2][4], rs[2][4];
// 	static double	xs[3][2];
// 	static double	x_olds[2], x_news[2];


// 	FILE* fout;

// 	double	h, q_max;
// 	long	i, j, n, time;

// 	fout = fopen("runge1.csv", "w");

// 	initials(&q_max, &h, x_olds);


// 	n = (long)(q_max / h) + 1;

// 	for (i = 0; i < 2; i++) {
// 		fprintf(fout, "%10.7lf,", x_olds[i]);
// 	}
// 	fprintf(fout, "\n");

// 	for (i = 0; i < 2; i++)
// 		qs[i][3] = 0.0;
// 	for (j = 1; j < n; j++) {

// 		for (i = 0; i < 2; i++) {
// 			ks[i][0] = h * (*fs[i])(x_olds);
// 			rs[i][0] = (ks[i][0] - 2.0 * qs[i][3]) / 2.0;
// 			xs[0][i] = x_olds[i] + rs[i][0];
// 			qs[i][0] = qs[i][3] + 3.0 * rs[i][0] - ks[i][0] / 2.0;
// 		}
// 		for (i = 0; i < 2; i++) {
// 			ks[i][1] = h * (*f[i])(xs[0]);
// 			rs[i][1] = (1.0 - sqrt(0.5)) * (ks[i][1] - qs[i][0]);
// 			xs[1][i] = xs[0][i] + rs[i][1];
// 			qs[i][1] = qs[i][0] + 3.0 * rs[i][1] - (1.0 - sqrt(0.5)) * ks[i][1];
// 		}
// 		for (i = 0; i < 2; i++) {
// 			ks[i][2] = h * (*fs[i])(xs[0]);
// 			rs[i][2] = (1.0 + sqrt(0.5)) * (ks[i][2] - qs[i][1]);
// 			xs[2][i] = xs[1][i] + rs[i][2];
// 			qs[i][2] = qs[i][1] + 3.0 * rs[i][2] - (1.0 + sqrt(0.5)) * ks[i][2];
// 		}
// 		for (i = 0; i < 2; i++) {
// 			ks[i][3] = h * (*f[i])(xs[0]);
// 			rs[i][3] = (ks[i][3] - 2.0 * qs[i][2]) / 6.0;
// 			x_news[i] = xs[2][i] + rs[i][3];
// 			qs[i][3] = qs[i][2] + 3.0 * rs[i][3] - ks[i][3] / 2.0;
// 		}

// 		s[j] = x_news[1];

// 		for (i = 0; i < 2; i++) {
// 			fprintf(fout, "%10.7lf,", x_news[i]);
// 		}
// 		fprintf(fout, "\n");


// 		for (i = 0; i < 2; i++)
// 			x_olds[i] = x_news[i];

// 	}



// }

// void initials(double* q_max, double* dq, double* x0) {


// 	int i;
// 	fflush(stdin);

// 	*q_max = 1.0;

// 	*dq = 0.00001;


// 	for (i = 0; i < 2; i++) {
// 		x0[i] = 0.0;
// 	}

// }


// double fs0(double* x) {

// 	return(1.0);
// }

// double fs1(double* x) {

// 	double	t = x[0];
// 	double	ret;
// 	ret = sqrt(pow(d1Rxdq1(Bx, qs, (int)(x[0] * 100)), 2) + pow(d1Rydq1(By, qs, (int)(x[0] * 100)), 2));
// 	return(ret);
// }





//全探索
Search searchP(const std::vector<double>& x) {


	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0 = DBL_MAX;


	for (i = 0; i < Q_sample; i++) {


		//内積計算
		dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

		dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

		if (-0.001 < dot && dot < 0.001) {

			if (dist < dist0) {
				dist0 = dist;
				sr.Psx = R[i][0];
				sr.Psy = R[i][1];
				sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
				sr.Cs = cs[i][0];
				sr.Cs1 = cs[i][1];
				sr.Cs2 = cs[i][2];
				sr.j = i;

			}

		}
	}

	return(sr);

}

//部分探索
Search searchPP(const std::vector<double>& x) {
	int i;
	double dot = 0.0;
	double dist = 0.0;
	double dist0 = DBL_MAX;

	if (sr.j < PSdist) {
		for (i = 0; i < sr.j + PSdist; i++) {
			//内積計算
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.001 < dot && dot < 0.001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

			}
		}
	}
	else if (sr.j > Q_sample - PSdist) {
		for (i = sr.j - PSdist; i < Q_sample; i++) {
			//内積計算
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.001 < dot && dot < 0.001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

			}

		}
	}
	else {
		for (i = sr.j - PSdist; i < sr.j + PSdist; i++) {
			//内積計算
			dot = (x[1] - R[i][0]) * (dRdq[i][0] / norm(Bx, By, qs, i)) + (x[2] - R[i][1]) * (dRdq[i][1] / norm(Bx, By, qs, i));

			dist = sqrt(pow((x[1] - R[i][0]), 2) + pow((x[2] - R[i][1]), 2));

			if (-0.001 < dot && dot < 0.001) {

				if (dist < dist0) {
					dist0 = dist;
					sr.Psx = R[i][0];
					sr.Psy = R[i][1];
					sr.d = (x[1] - R[i][0]) * (-(dRdq[i][1] / norm(Bx, By, qs, i))) + (x[2] - R[i][1]) * (dRdq[i][0] / norm(Bx, By, qs, i));
					sr.Cs = cs[i][0];
					sr.Cs1 = cs[i][1];
					sr.Cs2 = cs[i][2];
					sr.j = i;
				}

			}

		}
	}


	return(sr);

}






