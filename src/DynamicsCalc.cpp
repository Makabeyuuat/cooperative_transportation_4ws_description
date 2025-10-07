#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cfloat>
#include <time.h>
#include <vector>
#include "mathFunc.h"		// 数学関数のヘッダーファイル
#include "Bezier.h"			// Bezier 曲線の関数
#include "vehicle.hpp"       // Vehicle クラスの宣言
#include "callback_ct.hpp"     // コールバック関数の宣言
#include "initial.hpp"		// 初期値設定のヘッダーファイル
#include "DynamicsCalc.hpp"

DynamicsCalculator::DynamicsCalculator() { }


void DynamicsCalculator::calcXold(std::vector<double>& x_old){

    //車両の姿勢角
    double theta4 = true_vehicle_yaw[0] + PAI;        // 車体yaw（v1）
    double theta7 = true_vehicle_yaw[1];              // 車体yaw（v2）
    double theta10 = true_vehicle_yaw[2];             // 車体yaw（v3）

     //状態変数を計算
     //gazebo
     //荷台の姿勢角
     x_old[1] =  true_carrier_pos[0];
     x_old[2] =  true_carrier_pos[1];
     x_old[5] =  true_carrier_yaw; 
     //後輪：V1,V2,V3
     x_old[10] = true_vehicle1_steering_yaw[1];
     x_old[16] = true_vehicle2_steering_yaw[1];
     x_old[22] = true_vehicle3_steering_yaw[1];

    // 摺動量（delta）
    double delta1 = delta_pos[0];
    double delta2 = delta_pos[1];
    double delta3 = delta_pos[2];

    //デバッグ用ログ出力
	// ROS_INFO_THROTTLE(0.1,"DynamicsCalc:delta1=%.3f, delta2=%.3f, delta3=%.3f, SIGN = %d",
     //  delta1, delta2, delta3, VEHICLE1_SLIDING_MECHANISM_SIGN);

    // --- 仮想リンクの“車体基準角” thetav* をまず作る（参考コードの作法に揃える） ---
    // thetav3,5,7 は「スライダj側」、thetav2,4,6 は「アームi側」という対応（参考）
    double thetav3 = calc_thetavj_from_deltai(delta1, VEHICLE1_SLIDING_MECHANISM_SIGN);
    double thetav6 = calc_thetavj_from_deltai(delta2, VEHICLE2_SLIDING_MECHANISM_SIGN);
    double thetav9 = calc_thetavj_from_deltai(delta3, VEHICLE3_SLIDING_MECHANISM_SIGN);

    double thetav2 = calc_thetavi_from_thetavj(thetav3, VEHICLE1_SLIDING_MECHANISM_SIGN);
    double thetav5 = calc_thetavi_from_thetavj(thetav6, VEHICLE2_SLIDING_MECHANISM_SIGN);
    double thetav8 = calc_thetavi_from_thetavj(thetav9, VEHICLE3_SLIDING_MECHANISM_SIGN);


     //デバッグ用ログ出力
	// ROS_INFO_THROTTLE(0.25,"DynamicsCalc:thetav2=%.3f, thetav3=%.3f, thetav5=%.3f, thetav6=%.3f, thetav8=%.3f, thetav9=%.3f",
     //  thetav2, thetav3, thetav5, thetav6, thetav8, thetav9);

    //仮想リンクを計算
    // v1
    double th2 = thetav2 - (3.0/2.0)*PAI  + theta4;
    double th3 = -thetav3 - 0.5*PAI + theta4;
    // v2
    double th5 = thetav5 + PAI/2.0 + theta7;
    double th6 = -thetav6 + 3.0*PAI/2.0 + theta7;
    // v3
    double th8 = thetav8 + PAI/2.0 + theta10;
    double th9 = -thetav9 + 3.0*PAI/2.0 + theta10;

    //x_oldに格納
    x_old[7]  = th2;
    x_old[9]  = th3;
    x_old[13] = th5;
    x_old[15] = th6;
    x_old[19] = th8;
    x_old[21] = th9;

     //gazebo+culculate
     //v1前輪
     x_old[8] = true_vehicle1_steering_yaw[0] - th3;
     x_old[11] = theta4;       
     //v2前輪
     x_old[14] = true_vehicle2_steering_yaw[0] - th6;
     x_old[17] = theta7;        
     //v3前輪
     x_old[20] = true_vehicle3_steering_yaw[0] - th9;
     x_old[23] = theta10;

     // //デバッグ用ログ出力
	// ROS_INFO_THROTTLE(0.1,"calcX:t=%.3f, x=%.3f, y=%.3f, theta0=%.3f, phi1=%.3f, theta1=%.3f",
     //  x_old[0], x_old[1], x_old[2], x_old[3], x_old[4], x_old[5]);
	// ROS_INFO_THROTTLE(0.1,"vehicle1: phi2=%.3f, theta2=%.3f, phi3=%.3f, theta3=%.3f,phi4=%.3f, theta4=%.3f",
     //  x_old[6], x_old[7], x_old[8], x_old[9], x_old[10], x_old[11]);
	// ROS_INFO_THROTTLE(0.1,"vehicle2: phi5=%.3f, theta5=%.3f, phi6=%.3f, theta6=%.3f, phi7=%.3f, theta7=%.3f",
     //  x_old[12], x_old[13], x_old[14], x_old[15], x_old[16], x_old[17]);
	// ROS_INFO_THROTTLE(0.1,"vehicle3: phi8=%.3f, theta8=%.3f, phi9=%.3f, theta9=%.3f, phi10=%.3f, theta10=%.3f",
     //  x_old[18], x_old[19], x_old[20], x_old[21], x_old[22], x_old[23]);

//     //車両の姿勢角
//     x_old[11] = theta4 + PAI;
//     x_old[17] = theta7;
//     x_old[23] = theta10;

    // 仕上げ：wrap（±π）しておくと後段の tan() が安定
//     auto wrap_pi = [](double a){ while(a<=-PAI) a+=2*PAI; while(a> PAI) a-=2*PAI; return a; };
//     x_old[7]  = wrap_pi(x_old[7]);
//     x_old[9]  = wrap_pi(x_old[9]);
//     x_old[13] = wrap_pi(x_old[13]);
//     x_old[15] = wrap_pi(x_old[15]);
//     x_old[19] = wrap_pi(x_old[19]);
//     x_old[21] = wrap_pi(x_old[21]);
//     x_old[8]  = wrap_pi(x_old[8]);
//     x_old[14] = wrap_pi(x_old[14]);
//     x_old[20] = wrap_pi(x_old[20]);
//     x_old[10] = wrap_pi(x_old[10]);
//     x_old[16] = wrap_pi(x_old[16]);
//     x_old[22] = wrap_pi(x_old[22]);
//     x_old[11] = wrap_pi(x_old[11]);
//     x_old[17] = wrap_pi(x_old[17]);
//     x_old[23] = wrap_pi(x_old[23]);
}

// delta1, delta2, delta3からthetav3, thetav5, thetav7を計算する関数
double DynamicsCalculator::calc_thetavj_from_deltai(double deltai, int vehicle_sliding_mechanism_sign) {

	double k1 = lv*l3;
	double k2 = 2*deltai*l3;
	double k3 = deltai*deltai + (lv/2.0)*(lv/2.0) + l3*l3 - 4.0*l2*l2;
	double ret = atan2(k1,k2) - atan2(sqrt(k1*k1+k2*k2-k3*k3),k3);

	return(ret);
}

// thetav3, thetav5, thetav7からthetav2, thetav4, thetav6を計算する関数
double DynamicsCalculator::calc_thetavi_from_thetavj(double thetavj, int vehicle_sliding_mechanism_sign) {

	double sinthetavi = (lv - 2.0*l3*sin(thetavj)) / (4.0*l2);
	double costhetavi = vehicle_sliding_mechanism_sign * (sqrt(1-pow(sinthetavi,2)));

	double ret = atan2(sinthetavi,costhetavi);
	return(ret); 
}


//モデルの速度成分の計算
void DynamicsCalculator::computeCoefficients(const std::vector<double>& x) {
     // //thetaの計算
     // double Th1  = x_old[3]  - x_old[4]  - x_old[5];
     // double Th2  = x_old[4]  + x_old[5]  - x_old[6]  - x_old[7];
     // double Th3  = x_old[6]  + x_old[7]  - x_old[8]  - x_old[9];
     // double Th4  = x_old[8]  + x_old[9]  - x_old[11];
     // double Th5  = x_old[3]  - x_old[12] - x_old[13];
     // double Th6  = x_old[12] + x_old[13] - x_old[14] - x_old[15];
     // double Th7  = x_old[14] + x_old[15] - x_old[17];
     // double Th8  = x_old[3]  - x_old[18] - x_old[19];
     // double Th9  = x_old[18] + x_old[19] - x_old[20] - x_old[21];
     // double Th10 = x_old[20] + x_old[21] - x_old[23];


     // // 使う直前
     // double phi1 = wrapPi(x_old[4]);
     // double phi2 = wrapPi(x_old[6]);
     // double phi3 = wrapPi(x_old[8]);
     // double phi4 = wrapPi(x_old[10]);
     // double phi5 = wrapPi(x_old[12]);
     // double phi6 = wrapPi(x_old[14]);
     // double phi7 = wrapPi(x_old[16]);
     // double phi8 = wrapPi(x_old[18]);
     // double phi9 = wrapPi(x_old[20]);
     // double phi10= wrapPi(x_old[22]);

     // double tan_phi1 = tan(std::clamp(phi1, -LIM, LIM));
     // double tan_phi2 = tan(std::clamp(phi2, -LIM, LIM));
     // double tan_phi3 = tan(std::clamp(phi3, -LIM, LIM));
     // double tan_phi5 = tan(std::clamp(phi5, -LIM, LIM));
     // double tan_phi6 = tan(std::clamp(phi6, -LIM, LIM));
     // double tan_phi8 = tan(std::clamp(phi8, -LIM, LIM));
     // double tan_phi9 = tan(std::clamp(phi9, -LIM, LIM));
 
     // // ai を globals::ai に書き込む
     // ai.a0t  = 1.0;
 
     // ai.a1n1 = ai.a0t * std::sin(Th1);
     // ai.a1t1 = ai.a0t * std::cos(Th1)- 2*ai.a1n1 * tan_phi1;
 
     // ai.a1n2 = (2*ai.a1n1/std::cos(phi1))* std::sin(x_old[3]-x_old[5]-PAI/6);
     // ai.a1t2 = ai.a0t + (2*ai.a1n1/std::cos(phi1))  * std::cos(x_old[3]-x_old[5]-PAI/6);
 
     // ai.a1n3 = -(2*ai.a1n1/std::cos(phi1)) * std::sin(x_old[3]-x_old[5]+PAI/6);
     // ai.a1t3 = ai.a0t- (2*ai.a1n1/std::cos(phi1)) * std::cos(x_old[3]-x_old[5]+PAI/6);
 
     // // 第1車両
     // ai.a2n = ai.a1t1*std::sin(Th2) - ai.a1n1*std::cos(Th2);
     // ai.a2t = ai.a1t1*std::cos(Th2) + ai.a1n1*std::sin(Th2) - 2*ai.a2n * tan_phi2;
 
     // ai.a3n = ai.a2t*std::sin(Th3) - ai.a2n*std::cos(Th3);
     // ai.a3t = ai.a2t*std::cos(Th3) + ai.a2n*std::sin(Th3) - 2*ai.a3n * tan_phi3;
 
     // ai.V3t = ai.a2t*std::cos(Th3) + ai.a2n*std::sin(Th3) - ai.a3n * tan_phi3;
     // ai.a4t = ai.V3t * std::cos(Th4);
 
     // // 第2車両
     // ai.a5n = ai.a1t2*std::sin(Th5) - ai.a1n2*std::cos(Th5);
     // ai.a5t = ai.a1t2*std::cos(Th5) + ai.a1n2*std::sin(Th5) - 2*ai.a5n * tan_phi5;
 
     // ai.a6n = ai.a5t*std::sin(Th6) - ai.a5n*std::cos(Th6);
     // ai.a6t = ai.a5t*std::cos(Th6) + ai.a5n*std::sin(Th6) - 2*ai.a6n * tan_phi6;
 
     // ai.V6t = ai.a5t*std::cos(Th6) + ai.a5n*std::sin(Th6) - ai.a6n * tan_phi6;
     // ai.a7t = ai.V6t * std::cos(Th7);
 
     // // 第3車両
     // ai.a8n = ai.a1t3*std::sin(Th8) - ai.a1n3*std::cos(Th8);
     // ai.a8t = ai.a1t3*std::cos(Th8) + ai.a1n3*std::sin(Th8) - 2*ai.a8n * tan_phi8;
 
     // ai.a9n = ai.a8t*std::sin(Th9) - ai.a8n*std::cos(Th9);
     // ai.a9t = ai.a8t*std::cos(Th9) + ai.a8n*std::sin(Th9) - 2*ai.a9n * tan_phi9;
 
     // ai.V9t = ai.a8t*std::cos(Th9) + ai.a8n*std::sin(Th9) - ai.a9n * tan_phi9;
     // ai.a10t = ai.V9t * std::cos(Th10);
 
     // // 車両の速度成分
     // v1 = (ai.a2t*std::cos(Th3)
     //       + ai.a2n*std::sin(Th3)
     //       - ai.a3n*tan_phi3) * u1;
     // v2 = (ai.a5t*std::cos(Th6)
     //       + ai.a5n*std::sin(Th6)
     //       - ai.a6n*tan_phi6) * u1;
     // v3 = (ai.a8t*std::cos(Th9)
     //       + ai.a8n*std::sin(Th9)
     //       - ai.a9n*tan_phi9) * u1;

     double Theta1 = x[3] - x[4] - x[5];
	double Theta2 = x[4] + x[5] - x[6] - x[7];
	double Theta3 = x[6] + x[7] - x[8] - x[9];
	double Theta4 = x[8] + x[9] - x[11];
	double Theta5 = x[3] - x[12] - x[13];
	double Theta6 = x[12] + x[13] - x[14] - x[15];
	double Theta7 = x[14] + x[15] - x[17];
	double Theta8 = x[3] - x[18] - x[19];
	double Theta9 = x[18] + x[19] - x[20] - x[21];
	double Theta10 = x[20] + x[21] - x[23];

	ai.a0t = 1.0;

	ai.a1n1 = ai.a0t * sin(Theta1);
	ai.a1t1 = ai.a0t * cos(Theta1) - 2 * ai.a1n1 * tan(x[4]);

	ai.a1n2 = (2 * ai.a1n1 / (cos(x[4]))) * sin(x[3] - x[5] - PAI / 6);
	ai.a1t2 = ai.a0t + (2 * ai.a1n1 / (cos(x[4]))) * cos(x[3] - x[5] - PAI / 6);

	ai.a1n3 = -(2 * ai.a1n1 / (cos(x[4]))) * sin(x[3] - x[5] + PAI / 6);
	ai.a1t3 = ai.a0t - (2 * ai.a1n1 / (cos(x[4]))) * cos(x[3] - x[5] + PAI / 6);

	//第１車両
	ai.a2n = ai.a1t1 * sin(Theta2) - ai.a1n1 * cos(Theta2);
	ai.a2t = ai.a1t1 * cos(Theta2) + ai.a1n1 * sin(Theta2) - 2 * ai.a2n * tan(x[6]);

	ai.a3n = ai.a2t * sin(Theta3) - ai.a2n * cos(Theta3);
	ai.a3t = ai.a2t * cos(Theta3) + ai.a2n * sin(Theta3) - 2 * ai.a3n * tan(x[8]);


	ai.V3t = ai.a2t * cos(Theta3) + ai.a2n * sin(Theta3) - ai.a3n * tan(x[8]);


	ai.a4t = ai.V3t * cos(Theta4);

	//第２車両
	ai.a5n = ai.a1t2 * sin(Theta5) - ai.a1n2 * cos(Theta5);
	ai.a5t = ai.a1t2 * cos(Theta5) + ai.a1n2 * sin(Theta5) - 2 * ai.a5n * tan(x[12]);

	ai.a6n = ai.a5t * sin(Theta6) - ai.a5n * cos(Theta6);
	ai.a6t = ai.a5t * cos(Theta6) + ai.a5n * sin(Theta6) - 2 * ai.a6n * tan(x[14]);


	ai.V6t = ai.a5t * cos(Theta6) + ai.a5n * sin(Theta6) - ai.a6n * tan(x[14]);


	ai.a7t = ai.V6t * cos(Theta7);

	//第３車両
	ai.a8n = ai.a1t3 * sin(Theta8) - ai.a1n3 * cos(Theta8);
	ai.a8t = ai.a1t3 * cos(Theta8) + ai.a1n3 * sin(Theta8) - 2 * ai.a8n * tan(x[18]);

	ai.a9n = ai.a8t * sin(Theta9) - ai.a8n * cos(Theta9);
	ai.a9t = ai.a8t * cos(Theta9) + ai.a8n * sin(Theta9) - 2 * ai.a9n * tan(x[20]);


	ai.V9t = ai.a8t * cos(Theta9) + ai.a8n * sin(Theta9) - ai.a9n * tan(x[20]);


	ai.a10t = ai.V9t * cos(Theta10);

	//各車両の操舵角方向の速度
	v1 = (ai.a2t * cos(Theta3) + ai.a2n * sin(Theta3) - ai.a3n * tan(x[8])) * u1;
	v2 = (ai.a5t * cos(Theta6) + ai.a5n * sin(Theta6) - ai.a6n * tan(x[14])) * u1;
	v3 = (ai.a8t * cos(Theta9) + ai.a8n * sin(Theta9) - ai.a9n * tan(x[20])) * u1;


     Phi[1] = x_old[9] + x_old[8] - x_old[11] + PAI;
	Phi[2] = x_old[15] + x_old[14] - x_old[17];
	Phi[3] = x_old[21] + x_old[20] - x_old[23];


     // 車両の前後輪の速度成分
     v1f = v1 * cos(Phi[1]);
     v1r = v1 * cos(x[10]);      
     v2f = v2 * cos(Phi[2]);
     v2r = v2 * cos(x[16]);
     v3f = v3 * cos(Phi[3]);
     v3r = v3 * cos(x[22]);

     // //デバッグ用ログ出力
	// ROS_INFO_THROTTLE(0.1,"DynamicsCalc:v1=%.3f, v2=%.3f, v3=%.3f",
     //  v1, v2, v3);   
      
     //  ROS_INFO_THROTTLE(0.1,
     //  "v3 angles: theta8=%.3f, theta9=%.3f, theta10=%.3f | phi9(x20)=%.3f | tan(phi9)=%.3f | Thetap9=%.3f, Thetap10=%.3f",
     //  x_old[19], x_old[21], x_old[23], x_old[20], tan(x_old[20]), Thetap[9], Thetap[10]);

     //  ROS_INFO_THROTTLE(0.1,
     //  "v2 angles: theta5=%.3f, theta6=%.3f, theta7=%.3f | phi6(x14)=%.3f | tan(phi6)=%.3f | Thetap6=%.3f, Thetap7=%.3f",
     //  x_old[13], x_old[15], x_old[17], x_old[14], tan(x_old[14]), Thetap[6], Thetap[7]);

 }


void DynamicsCalculator::calculate(const std::vector<double>& x_old, double t_max, double l1, double l2, double l3, int sr_j)
{

     
    // 摺動機構の変位の計算
//     delta1d = -(3.0 / 20.0) * sin(4 * PAI * x_old[0] / t_max);
//     Ddelta1d = -(3.0 / 20.0) * (4 * PAI / t_max) * cos(2 * PAI * x_old[0] / t_max);
//     DDdelta1d = (3.0 / 20.0) * (4 * PAI / t_max) * (2 * PAI / t_max) * sin(2 * PAI * x_old[0] / t_max);

//     delta2d = (3.0 / 20.0) * sin(2 * PAI * x_old[0] / t_max);
//     Ddelta2d = (3.0 / 20.0) * (2 * PAI / t_max) * cos(2 * PAI * x_old[0] / t_max);
//     DDdelta2d = -(3.0 / 20.0) * (2 * PAI / t_max) * (2 * PAI / t_max) * sin(2 * PAI * x_old[0] / t_max);

//     delta3d = (3.0 / 20.0) * sin(2 * PAI * x_old[0] / t_max);
//     Ddelta3d = (3.0 / 20.0) * (2 * PAI / t_max) * cos(2 * PAI * x_old[0] / t_max);
//     DDdelta3d = -(3.0 / 20.0) * (2 * PAI / t_max) * (2 * PAI / t_max) * sin(2 * PAI * x_old[0] / t_max);
    delta1d =  0.0;
    Ddelta1d =  0.0;
    DDdelta1d = 0.0;

    delta2d =  0.0;
    Ddelta2d = 0.0;
    DDdelta2d = 0.0;

    delta3d = 0.0;
    Ddelta3d = 0.0;
    DDdelta3d = 0.0;

    // thetaT の計算（グローバル変数 thetaT に代入）
    thetaT = atan2(dRdq[sr_j][1], dRdq[sr_j][0]);

    // 各 Thetap の計算（グローバル配列 Thetap に代入）
    Thetap0 = x_old[3]  - thetaT;
    Thetap1 = x_old[5]  - thetaT;
    Thetap2 = x_old[7]  - thetaT;
    Thetap3 = x_old[9]  - thetaT;
    Thetap4 = x_old[11] - thetaT;
    Thetap5 = x_old[13] - thetaT;
    Thetap6 = x_old[15] - thetaT;
    Thetap7 = x_old[17] - thetaT;
    Thetap8 = x_old[19] - thetaT;
    Thetap9 = x_old[21] - thetaT;
    Thetap10= x_old[23] - thetaT;
    

    // 車両の後輪間中点の計算（グローバル変数 x1, Y1, x2, y2, x3, y3 に代入）
    x1 = x_old[1] + 2 * l1 * cos(x_old[5]) + 2 * l2 * cos(x_old[7])
         + l3 * cos(x_old[9]) + cos(x_old[11]);
    Y1 = x_old[2] + 2 * l1 * sin(x_old[5]) + 2 * l2 * sin(x_old[7])
         + l3 * sin(x_old[9]) + sin(x_old[11]);

    x2 = x_old[1] + 2 * l1 * cos(x_old[5] + 2 * PAI / 3)
         - 2 * l2 * cos(x_old[13]) - l3 * cos(x_old[15]) - cos(x_old[17]);
    y2 = x_old[2] + 2 * l1 * sin(x_old[5] + 2 * PAI / 3)
         - 2 * l2 * sin(x_old[13]) - l3 * sin(x_old[15]) - sin(x_old[17]);

    x3 = x_old[1] + 2 * l1 * cos(x_old[5] + 4 * PAI / 3)
         - 2 * l2 * cos(x_old[19]) - l3 * cos(x_old[21]) - cos(x_old[23]);
    y3 = x_old[2] + 2 * l1 * sin(x_old[5] + 4 * PAI / 3)
         - 2 * l2 * sin(x_old[19]) - l3 * sin(x_old[21]) - sin(x_old[23]);

    // 荷台の頂点の計算（グローバル配列 Trix, Triy に代入）
    Trix[1] = x_old[1] + 2 * l1 * cos(x_old[5]);
    Triy[1] = x_old[2] + 2 * l1 * sin(x_old[5]);
    Trix[2] = x_old[1] + 2 * l1 * cos(x_old[5] + 2 * PAI / 3);
    Triy[2] = x_old[2] + 2 * l1 * sin(x_old[5] + 2 * PAI / 3);
    Trix[3] = x_old[1] + 2 * l1 * cos(x_old[5] + 4 * PAI / 3);
    Triy[3] = x_old[2] + 2 * l1 * sin(x_old[5] + 4 * PAI / 3);

    //非ホロ計算
    dx1 = (f1(x_old) + l1 * f5(x_old) * (-sin(x_old[5])));
    dy1 = (f2(x_old) + l1 * f5(x_old) * cos(x_old[5]));
    nh1 = dx1 * sin(x_old[5] + x_old[4]) - dy1 * cos(x_old[5] + x_old[4]);    
    //vehicle-1
    dx2 = dx1 + (l1 * f5(x_old) * (-sin(x_old[5]))) + (l2 * f7(x_old) * (-sin(x_old[7])));
    dy2 = dy1 + (l1 * f5(x_old) * (cos(x_old[5]))) + (l2 * f7(x_old) * (cos(x_old[7])));
    nh2 = dx2 * sin(x_old[7] + x_old[6]) - dy2 * cos(x_old[7] + x_old[6]);    
    dx3 = dx2 + (l2 * f7(x_old) * (-sin(x_old[7]))) + (l3 * f9(x_old) * (-sin(x_old[9])));
    dy3 = dy2 + (l2 * f7(x_old) * (cos(x_old[7]))) + (l3 * f9(x_old) * (cos(x_old[9])));
    nh3 = dx3 * sin(x_old[9] + x_old[8]) - dy3 * cos(x_old[9] + x_old[8]);    
    dx4 = dx3 + (lv * f11(x_old) * (-sin(x_old[11])));
    dy4 = dy3 + (lv * f11(x_old) * (cos(x_old[11])));
    nh4 = dx4 * sin(x_old[11] + x_old[10]) - dy4 * cos(x_old[11] + x_old[10]);    
    //vehicle-2
    dx5 = f1(x_old) + (2 * l1 * f5(x_old) * (-sin(x_old[5] + 2 * PAI / 3))) - (l2 * f13(x_old) * (-sin(x_old[13])));
    dy5 = f2(x_old) + (2 * l1 * f5(x_old) * (cos(x_old[5] + 2 * PAI / 3))) - (l2 * f13(x_old) * (cos(x_old[13])));
    nh5 = dx5 * sin(x_old[13] + x_old[12]) - dy5 * cos(x_old[13] + x_old[12]);    
    dx6 = dx5 - (l2 * f13(x_old) * (-sin(x_old[13]))) - (l3 * f15(x_old) * (-sin(x_old[15])));
    dy6 = dy5 - (l2 * f13(x_old) * (cos(x_old[13]))) - (l3 * f15(x_old) * (cos(x_old[15])));
    nh6 = dx6 * sin(x_old[17] + Phi[2]) - dy6 * cos(x_old[17] + Phi[2]);    
    dx7 = dx6 - (lv * f17(x_old) * (-sin(x_old[17])));
    dy7 = dy6 - (lv * f17(x_old) * (cos(x_old[17])));
    nh7 = dx7 * sin(x_old[17] + x_old[16]) - dy7 * cos(x_old[17] + x_old[16]);    
    //vehicle-3
    dx8 = f1(x_old) + (2 * l1 * f5(x_old) * (-sin(x_old[5] + 4 * PAI / 3))) - (l2 * f19(x_old) * (-sin(x_old[19])));
    dy8 = f2(x_old) + (2 * l1 * f5(x_old) * (cos(x_old[5] + 4 * PAI / 3))) - (l2 * f19(x_old) * (cos(x_old[19])));
    nh8 = dx8 * sin(x_old[19] + x_old[18]) - dy8 * cos(x_old[19] + x_old[18]);    
    dx9 = dx8 - (l2 * f19(x_old) * (-sin(x_old[19]))) - (l3 * f21(x_old) * (-sin(x_old[21])));
    dy9 = dy8 - (l2 * f19(x_old) * (cos(x_old[19]))) - (l3 * f21(x_old) * (cos(x_old[21])));
    nh9 = dx9 * sin(x_old[23] + Phi[3]) - dy9 * cos(x_old[23] + Phi[3]);    
    dx10 = dx9 - (lv * f23(x_old) * (-sin(x_old[23])));
    dy10 = dy9 - (lv * f23(x_old) * (cos(x_old[23])));
    nh10 = dx10 * sin(x_old[23] + x_old[22]) - dy10 * cos(x_old[23] + x_old[22]);
    
}
