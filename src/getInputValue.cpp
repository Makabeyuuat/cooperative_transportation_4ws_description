#include "cooperative_transportation_4ws_backstepping/dynamics_integrator.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"        // 数学関数のヘッダーファイル
#include "cooperative_transportation_4ws_backstepping/Bezier.h"         // Bezier 曲線の関数
#include "cooperative_transportation_4ws_backstepping/vehicle.hpp"       // Vehicle クラスの宣言
#include "cooperative_transportation_4ws_backstepping/callback.hpp"     // コールバック関数の宣言
#include "cooperative_transportation_4ws_backstepping/getInputValue.hpp"
#include "cooperative_transportation_4ws_backstepping/differential_equations.hpp"
#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include <ros/ros.h> 
#include <Eigen/Dense>


// コンストラクタ
getInputValue::getInputValue(double h)
    : h(h),
    k(DIM+1, std::vector<double>(4,0.0)),
    r(DIM+1, std::vector<double>(4,0.0)),
    q(DIM+1, std::vector<double>(4,0.0)),
    x(3,   std::vector<double>(DIM+1,0.0)),
    fAllVec(fAll.begin(), fAll.end()),
    kinematics_solver_()
   
{

}

void getInputValue::rungeKutta(std::vector<double>& x_old, int sr_j) {
    int n = static_cast<int>(x_old.size());  // n = DIM+1
    // ---ルンゲ‐クッタ法による状態更新---
    // 第1段階
    for (int i = 0; i < n; i++) {
        double dx = fAllVec[i](x_old);
        k[i][0] = h * dx;
        r[i][0] = (k[i][0] - 2.00 * q[i][3]) / 2.00;
        x[0][i] = x_old[i] + r[i][0];
        q[i][0] = q[i][3] + 3.0 * r[i][0] - k[i][0] / 2.00;
    }   
    // 第2段階
    for (int i = 0; i < n; i++) {
        double dx = fAllVec[i](x[0]);
        k[i][1] = h * dx;
        r[i][1] = (1.0 - std::sqrt(0.5)) * (k[i][1] - q[i][0]);
        x[1][i] = x[0][i] + r[i][1];
        q[i][1] = q[i][0] + 3.0 * r[i][1] - (1.0 - std::sqrt(0.5)) * k[i][1];
    }
    // 第3段階
    for (int i = 0; i < n; i++) {
        double dx = fAllVec[i](x[0]);
        k[i][2] = h * dx;
        r[i][2] = (1.0 + std::sqrt(0.5)) * (k[i][2] - q[i][1]);
        x[2][i] = x[1][i] + r[i][2];
        q[i][2] = q[i][1] + 3.0 * r[i][2] - (1.0 + std::sqrt(0.5)) * k[i][2];
    }
    // 第4段階
    for (int i = 0; i < n; i++) {
        double dx = fAllVec[i](x[0]);
        k[i][3] = h * dx;
        r[i][3] = (k[i][3] - 2.00 * q[i][2]) / 6.0;
        x_new[i] = x[2][i] + r[i][3];
        q[i][3] = q[i][2] + 3.0 * r[i][3] - k[i][3] / 2.00;
    }


    // --- 状態ベクトルの更新-kinematics ---
	//荷台
    //t
	//  x_old[0]=x_new[0];
    // //theta0
    // x_old[3]=x_new[3];
    // //phi1
    // x_old[4]=x_new[4];
    // //第一車両
    // //phi2
	//  x_old[6]=x_new[6];
    // //phi3
	//  x_old[8]=x_new[8];
    // //theta3
	//  x_old[9]=x_new[9];
    // //phi4
	//  x_old[10]=x_new[10];
    // //theta4
	//  x_old[11]=x_new[11];
    //  //第2車両
	//  x_old[12]=x_new[12];
	//  x_old[14]=x_new[14];
	//  x_old[15]=x_new[15];
	//  x_old[16]=x_new[16];
	//  x_old[17]=x_new[17];
    //  //第3車両
	//  x_old[18]=x_new[18];
	//  x_old[20]=x_new[20];
	//  x_old[21]=x_new[21];
	//  x_old[22]=x_new[22];
	//  x_old[23]=x_new[23];
	// Phi[1] = x_old[9] + x_old[8] - x_old[11];
	// Phi[2] = x_old[15] + x_old[14] - x_old[17];
	// Phi[3] = x_old[21] + x_old[20] - x_old[23];




	 // --- 状態ベクトルの更新-dynamics ---
    //仮想ステアリングのみ、rungeKuttaで更新
	//荷台
    //t
	 x_old[0]=x_new[0];
    //theta0
    x_old[3]=x_new[3];
    //phi1
    x_old[4]=x_new[4];
    //第1車両
    //phi2
	 x_old[6]=x_new[6];
     //第2車両
	 x_old[12]=x_new[12];
     //第3車両
	 x_old[18]=x_new[18];



	


   
    
}

void getInputValue::getU(std::vector<double>& x_old, int sr_j) {
    // --- 制御入力の計算 ---
    // 各内部関数を呼び出して制御入力を計算
    U1(x_old, sr_j);
    U2(x_old, sr_j);
    U3(x_old, sr_j);
    U4_U5_U6(x_old, sr_j);
    U7_U8_U9(x_old, sr_j);
    U10_U11_U12(x_old, sr_j);

	// ROS_INFO_THROTTLE(0.1,"getU: u1=%.3f, u2=%.3f, u3=%.3f, ...u12=%.3f",
    //        u1, u2, u3, u12);


}

void getInputValue::getXInput(std::vector<double>& x_old, std::vector<double>& x_input){
	
}
// getter 関数
// double getInputValue::getU4()  const { return u4; }
// double getInputValue::getU5()  const { return u5; }
// double getInputValue::getU6()  const { return u6; }
// double getInputValue::getU7()  const { return u7; }
// double getInputValue::getU8()  const { return u8; }
// double getInputValue::getU9()  const { return u9; }
// double getInputValue::getU10() const { return u10; }
// double getInputValue::getU11() const { return u11; }
// double getInputValue::getU12() const { return u12; }

// --- 制御入力計算用内部関数 ---

void getInputValue::U1(const std::vector<double>& x_old, int sr_j) {
   w1 = a0;
	u1 = ((1 - sr.d * sr.Cs) / cos(Thetap0)) * w1;

   u_kinematics[0] = u1;
}

void getInputValue::U2(const std::vector<double>& x_old, int sr_j) {

	
	//経路追従
	d0d = 0.0;
	dd0d = 0.0;
	ddd0d = 0.0;

	//重心を経路に対して周期的に変化させる
	/*d0d = -0.7 *  sin(2* PAI * x[0] / t_max);
	printf("%lf\n", d0d);
	dd0d =  -0.7 * (2* PAI / t_max) * cos(2 * PAI * x[0] / t_max);
	ddd0d = 0.7 * (2 * PAI / t_max) * (2 * PAI / t_max) * sin(2 * PAI * x[0] / t_max);*/



	z21 = (1 - sr.Cs * sr.d) * tan(Thetap0);
	z22 = sr.d;

	alpha21 = -(sr.Cs * (1 - sr.Cs * sr.d) * Power(Sec(Thetap0), 2)) - sr.Cs * (1 - sr.Cs * sr.d) * Power(tan(Thetap0), 2) - sr.d * tan(Thetap0) * sr.Cs1;
	alpha22 = (1 - sr.Cs * sr.d) * Power(Sec(Thetap0), 2);

	w2 = ddd0d / a0 + (k1 + k2) * ((dd0d / a0) - z21) + k1 * k2 * ((d0d / a0) - z22 / a0);

	u2 = (1 / alpha22) * (w2 - alpha21 * u1);

   u_kinematics[1] = u2;
}

void getInputValue::U3(const std::vector<double>& x, int sr_j) {



	z31 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1);
	z32 = Thetap1;

	alpha31 = (Cos(x[4] - Thetap0 + Thetap1) * Power(1 - sr.Cs * sr.d, 2) * Sec(x[4]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1)) / l1 +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4])) / l1) + (sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d)) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha32 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4])) / l1) + (sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d)) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * Tan(Thetap0);

	alpha33 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4])) / l1 + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) / l1);

	w3 = ddthetap1d / a0 + (k3 + k4) * ((dthetap1d / a0) - z31) + k3 * k4 * ((thetap1d / a0) - z32 / a0);

	u3 = (1 / alpha33) * (w3 - (alpha31 * u1 + alpha32 * u2));

   u_kinematics[2] = u3;
}

void getInputValue::U4_U5_U6(const std::vector<double>& x, int sr_j) {

	double u4and = 0.0;
	double u5and = 0.0;
	double u6and = 0.0;
	Eigen::Matrix<double,6,1> ALPHA4 =  kinematics_solver_.ALPHA4_vec();
	Eigen::Matrix<double,6,1> ALPHA5 =  kinematics_solver_.ALPHA5_vec();
	Eigen::Matrix<double,6,1> ALPHA6 =  kinematics_solver_.ALPHA6_vec();
	Eigen::Matrix<double,4,1> THETAP2D =  kinematics_solver_.thetap2d_vec();
	Eigen::Matrix<double,4,1> THETAP3D =  kinematics_solver_.thetap3d_vec();
	Eigen::Matrix<double,4,1> K2_the =  kinematics_solver_.K2_vec();
	Eigen::Matrix<double,4,1> K3_the =  kinematics_solver_.K3_vec();
   
  

	z41 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[6]) * (-(Cos(x[4] - x[6] + Thetap1 - Thetap2) * Sin(x[4] - Thetap0 + Thetap1)) - Sin(x[4] - x[6] + Thetap1 - Thetap2) * (Cos(x[4] - Thetap0 + Thetap1) + 2 * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) / l2);

	z42 = Thetap2;

	alpha41 = ALPHA4(0);
	alpha42 = ALPHA4(1);
	alpha43 = ALPHA4(2);
	alpha44 = ALPHA4(3);


	z51 = (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[8])*(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) - Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6]))))/l3);
	z52 = Thetap3;

	alpha51 = ALPHA5(0);
	alpha52 = ALPHA5(1);
	alpha53 = ALPHA5(2);
	alpha54 = ALPHA5(3);
	alpha55 = ALPHA5(4);

	z61 = (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sin(x_old[8] + Thetap3 - Thetap4)*(Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) + Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6])) - (-(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))) + Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6])))*Tan(x_old[8])) - Cos(x_old[8] + Thetap3 - Thetap4)*(Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) + Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6])) - (-(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))) + Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6])))*Tan(x_old[8]))*Tan(x_old[10]))/lv);
	z62 = Thetap4;

	alpha61 = ALPHA6(0);
	alpha62 = ALPHA6(1);
	alpha63 = ALPHA6(2);
	alpha64 = ALPHA6(3);
	alpha65 = ALPHA6(4);
	alpha66 = ALPHA6(5);

	//摺動機構の目標相対姿勢角設計
	thetap2d  = THETAP2D(0); 
	dthetap2d = THETAP2D(1);
	K21 = K2_the(0);
	K22 = K2_the(1);
	K23 = K2_the(2);
	K24 = K2_the(3);

	thetap3d = THETAP3D(0);
	dthetap3d = THETAP3D(1);
	K31 = K3_the(0);
	K32 = K3_the(1);
	K33 = K3_the(2);
	K34 = K3_the(3);


	u4_a[0][0] = alpha44 - (K21 / a0);
	u4_a[0][1] = -K22 / a0;
	u4_a[0][2] = -K23 / a0;
	u4_a[1][0] = alpha54 - (K31 / a0);
	u4_a[1][1] = alpha55 - (K32 / a0);
	u4_a[1][2] = -(K33 / a0);
	u4_a[2][0] = alpha64;
	u4_a[2][1] = alpha65;
	u4_a[2][2] = alpha66;

	b4 = K24 / a0 + (k5 + k6) * (dthetap2d / a0 - z41) + k5 * k6 * (thetap2d / a0 - z42 / a0) - (alpha41 * a0 + alpha42 * u2 + alpha43 * u3);
	b5 = K34 / a0 + (k7 + k8) * (dthetap3d / a0 - z51) + k7 * k8 * (thetap3d / a0 - z52 / a0) - (alpha51 * a0 + alpha52 * u2 + alpha53 * u3);
	b6 = ddthetap4d / a0 + (k9 + k10) * ((dthetap4d / a0) - z61) + k9 * k10 * ((thetap4d / a0) - z62 / a0) - (alpha61 * a0 + alpha62 * u2 + alpha63 * u3);

	inv(u4_a, u4_inv_a);

	u4and = u4_inv_a[0][0] * b4 + u4_inv_a[0][1] * b5 + u4_inv_a[0][2] * b6;
	u5and = u4_inv_a[1][0] * b4 + u4_inv_a[1][1] * b5 + u4_inv_a[1][2] * b6;
	u6and = u4_inv_a[2][0] * b4 + u4_inv_a[2][1] * b5 + u4_inv_a[2][2] * b6;


	u4 = u4and;
	u5 = u5and;
	u6 = u6and;

    u_kinematics(3) = u4;
	u_kinematics(4) = u5;
	u_kinematics(5) = u6;

    w4 = (1/ a0)*(K21*u_kinematics(3) + K22*u_kinematics(4) + K23*u_kinematics(5) + K24) + (k5 + k6) * (dthetap2d / a0 - z41) + k5 * k6 * (thetap2d / a0 - z42 / a0);
	w5 = (1/ a0)*(K31*u_kinematics(3) + K32*u_kinematics(4) + K33*u_kinematics(5) + K34) + (k7 + k8) * (dthetap3d / a0 - z51) + k7 * k8 * (thetap3d / a0 - z52 / a0);
	w6 = ddthetap4d / a0 + (k9 + k10) * ((dthetap4d / a0) - z61) + k9 * k10 * ((thetap4d / a0) - z62 / a0);
}



void getInputValue::U7_U8_U9(const std::vector<double>& x, int sr_j) {

	/*thetap7d = -0.7 *  sin(2* PAI * x[0] / t_max);
	dthetap7d =  -0.7 * (2* PAI / t_max) * cos(2 * PAI * x[0] / t_max);
	ddthetap7d =  0.7 * (4 * PAI * PAI / t_max * t_max) * sin(2 * PAI * x[0] / t_max);*/

	double u7and = 0.0;
	double u8and = 0.0;
	double u9and = 0.0;
	Eigen::Matrix<double,4,1> THETAP5D =  kinematics_solver_.thetap5d_vec();
	Eigen::Matrix<double,4,1> THETAP6D =  kinematics_solver_.thetap6d_vec();
	Eigen::Matrix<double,4,1> K5_the =  kinematics_solver_.K5_vec();
	Eigen::Matrix<double,4,1> K6_the =  kinematics_solver_.K6_vec();

	z71 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2);
	z72 = Thetap5;

	alpha71 = (Power(1 - sr.Cs * sr.d, 2) * Sec(x[12]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
		(-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[12]) * Power(Sec(Thetap0), 2) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2)) / l2 +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[12]) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			(2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha72 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[12]) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
		(2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) * Tan(Thetap0);

	alpha73 = ((1 - sr.Cs * sr.d) * Sec(x[12]) * Sec(Thetap0) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) / l2;

	alpha77 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((Sec(x[12]) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) / l2 +
		(Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) / l2);

	z81 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3);
	z82 = Thetap6;

	alpha81 = (Power(1 - sr.Cs * sr.d, 2) * Sec(x[14]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) *
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
			(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[14]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) *
			(-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[14]) * Power(Sec(Thetap0), 2) * (-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3)) / l3 +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) *
			(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			(2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha82 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
		(2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
			2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
				2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) * Tan(Thetap0);

	alpha83 = ((1 - sr.Cs * sr.d) * Sec(x[14]) * Sec(Thetap0) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) * Tan(x[4]) +
			Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
			2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[12])))) / l3;

	alpha87 = ((1 - sr.Cs * sr.d) * Sec(x[14]) * Sec(Thetap0) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
		(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * Power(Sec(x[12]), 2) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) -
			2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3;

	alpha88 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((Sec(x[14]) * (-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3 +
		(Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0 - Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0 - Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) / l3);

	z91 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
		Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv);

	z92 = Thetap7;

	alpha91 = (Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[12]) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) / l2) *
		(Sin(x[14] + Thetap6 - Thetap7) * (Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) +
			(-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
				(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * (Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) +
				(-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
					(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[14]) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])))) / l3) *
			(Sin(x[14] + Thetap6 - Thetap7) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) +
				Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
					(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
						Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
							2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
				Cos(x[14] + Thetap6 - Thetap7) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
					(-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
						Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
							2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]) +
				Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
					(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
						Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
							2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (Sin(x[14] + Thetap6 - Thetap7) *
			((-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
						(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
							(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
						(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
							(-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-(Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14]))) -
			Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
				Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
					(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
						Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
							2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv)) / lv +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) *
			Sin(x[12] - x[14] + Thetap5 - Thetap6) + Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sin(x[14] + Thetap6 - Thetap7) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
					Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
						2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
							2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) *
				Sin(x[12] - x[14] + Thetap5 - Thetap6) + Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
				Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
					Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
					(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
						Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
							2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv) *sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d *sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) *sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha92 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sin(x[14] + Thetap6 - Thetap7) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
		(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
			2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
				2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
		Cos(x[14] + Thetap6 - Thetap7) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
			2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				(2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) + (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) + 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - (2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
			Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
				(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
					Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
						2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16])) / lv) * Tan(Thetap0);


	alpha93 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[14] + Thetap6 - Thetap7) * (Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) * Tan(x[4]) +
			Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
			2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[12])) -
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
			Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) * Tan(x[4]) +
				Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
				2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
					Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[12]))) * Tan(x[14])) -
		Cos(x[14] + Thetap6 - Thetap7) * (Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
			Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) * Tan(x[4]) +
				Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
				2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
					Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) * Tan(x[4]) +
					Cos(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
					2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) - 2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
						Sin(x[12] - Thetap0 + Thetap5) * (-2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv;

	alpha97 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[14] + Thetap6 - Thetap7) * (Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) +
		(-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * Power(Sec(x[12]), 2) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) -
			2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
			(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * Power(Sec(x[12]), 2) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) -
				2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
		Cos(x[14] + Thetap6 - Thetap7) * (Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) +
			(-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * Power(Sec(x[12]), 2) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) -
				2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5))) +
				(-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * Power(Sec(x[12]), 2) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) -
					2 * (-(Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) +
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv;


	alpha98 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[14] + Thetap6 - Thetap7) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
		Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		Power(Sec(x[14]), 2) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) -
		(-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) +
		Cos(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) -
		Cos(x[14] + Thetap6 - Thetap7) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			Power(Sec(x[14]), 2) * (-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) -
			(-((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6)) -
				Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]) +
		Sin(x[14] + Thetap6 - Thetap7) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
			Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
			(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
				Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
					2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14])) * Tan(x[16]))) / lv;

	alpha99 = -((Cos(x[14] + Thetap6 - Thetap7) * (1 - sr.Cs * sr.d) * Power(Sec(x[16]), 2) * Sec(Thetap0) * ((-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Sin(x[12] - x[14] + Thetap5 - Thetap6) +
		Cos(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
			2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12])) -
		(-(Cos(x[12] - x[14] + Thetap5 - Thetap6) * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5))) +
			Sin(x[12] - x[14] + Thetap5 - Thetap6) * (Cos(x[12] - Thetap0 + Thetap5) * (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[12] - Thetap0 + Thetap5) -
				2 * (-2 * Cos(x[12] - Thetap0 + Thetap5) * Sec(x[4]) * Sin(PAI / 6.0- Thetap0 + Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 - 2 * Cos(PAI / 6.0- Thetap0 + Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[12] - Thetap0 + Thetap5)) * Tan(x[12]))) * Tan(x[14]))) / lv);

	//摺動機構の目標相対姿勢角設計
	thetap5d = THETAP5D(0);
	dthetap5d = THETAP5D(1);
	K51 = K5_the(0);
	K52 = K5_the(1);
	K53 = K5_the(2);
	K54 = K5_the(3);

	thetap6d = THETAP6D(0);
	dthetap6d = THETAP6D(1);
	K61 = K6_the(0);
	K62 = K6_the(1);
	K63 = K6_the(2);
	K64 = K6_the(3);


	u7_a[0][0] = alpha77 - (K51 / a0);
	u7_a[0][1] = -K52 / a0;
	u7_a[0][2] = -K53 / a0;
	u7_a[1][0] = alpha87 - (K61 / a0);
	u7_a[1][1] = alpha88 - (K62 / a0);
	u7_a[1][2] = -K63 / a0;
	u7_a[2][0] = alpha97;
	u7_a[2][1] = alpha98;
	u7_a[2][2] = alpha99;

	b7 = K54 / a0 + (k11 + k12) * (dthetap5d / a0 - z71) + k11 * k12 * (thetap5d / a0 - z72 / a0) - (alpha71 * a0 + alpha72 * u2 + alpha73 * u3);
	b8 = K64 / a0 + (k13 + k14) * (dthetap6d / a0 - z81) + k13 * k14 * (thetap6d / a0 - z82 / a0) - (alpha81 * a0 + alpha82 * u2 + alpha83 * u3);
	b9 = ddthetap7d / a0 + (k15 + k16) * ((dthetap7d / a0) - z91) + k15 * k16 * ((thetap7d / a0) - z92 / a0) - (alpha91 * a0 + alpha92 * u2 + alpha93 * u3);

	inv(u7_a, u7_inv_a);

	u7and = u7_inv_a[0][0] * b7 + u7_inv_a[0][1] * b8 + u7_inv_a[0][2] * b9;
	u8and = u7_inv_a[1][0] * b7 + u7_inv_a[1][1] * b8 + u7_inv_a[1][2] * b9;
	u9and = u7_inv_a[2][0] * b7 + u7_inv_a[2][1] * b8 + u7_inv_a[2][2] * b9;

	u7 = u7and;
	u8 = u8and;
	u9 = u9and;

   u_kinematics(6) = u7;
	u_kinematics(7) = u8;
	u_kinematics(8) = u9;

   w7 = (1/ a0)*(K51*u_kinematics(6) + K52*u_kinematics(7) + K53*u_kinematics(8) + K54) + (k11 + k12) * (dthetap5d / a0 - z71) + k11 * k12 * (thetap5d / a0 - z72 / a0);
	w8 = (1/ a0)*(K61*u_kinematics(6) + K62*u_kinematics(7) + K63*u_kinematics(8) + K64) + (k13 + k14) * (dthetap6d / a0 - z81) + k13 * k14 * (thetap6d / a0 - z82 / a0);
	w9 = ddthetap7d / a0 + (k15 + k16) * ((dthetap7d / a0) - z91) + k15 * k16 * ((thetap7d / a0) - z92 / a0);
}

void getInputValue::U10_U11_U12(const std::vector<double>& x, int sr_j) {

	double u10and = 0.0;
	double u11and = 0.0;
	double u12and = 0.0;

   Eigen::Matrix<double,4,1> THETAP8D =  kinematics_solver_.thetap8d_vec();
   Eigen::Matrix<double,4,1> THETAP9D =  kinematics_solver_.thetap9d_vec();
   Eigen::Matrix<double,4,1> K8_the =  kinematics_solver_.K8_vec();
   Eigen::Matrix<double,4,1> K9_the =  kinematics_solver_.K9_vec();



	z101 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2);

	z102 = Thetap8;

	alpha101 = (Power(1 - sr.Cs * sr.d, 2) * Sec(x[18]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
		(2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[18]) * Power(Sec(Thetap0), 2) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2)) / l2 +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[18]) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			(-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));


	alpha102 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[18]) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
		(-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) * Tan(Thetap0);

	alpha103 = ((1 - sr.Cs * sr.d) * Sec(x[18]) * Sec(Thetap0) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) / l2;

	alpha1010 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((Sec(x[18]) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) / l2 +
		(Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) / l2);


	z111 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3);

	z112 = Thetap9;

	alpha111 = (Power(1 - sr.Cs * sr.d, 2) * Sec(x[20]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) *
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
			(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[20]) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) *
			(-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) +
				2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3 +
		(Power(1 - sr.Cs * sr.d, 2) * Sec(x[20]) * Power(Sec(Thetap0), 2) * (-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3)) / l3 +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) *
			(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			(-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha112 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
		(-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
			2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
				2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) * Tan(Thetap0);

	alpha113 = ((1 - sr.Cs * sr.d) * Sec(x[20]) * Sec(Thetap0) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) * Tan(x[4]) +
			Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
			2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[18])))) / l3;

	alpha1110 = ((1 - sr.Cs * sr.d) * Sec(x[20]) * Sec(Thetap0) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
		(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * Power(Sec(x[18]), 2) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) -
			2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3;

	alpha1111 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((Sec(x[20]) * (-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3 +
		(Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0 + Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0 + Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) / l3);

	z121 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
		Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv);
	z122 = Thetap10;

	alpha121 = (Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[18]) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) / l2) *
		(Sin(x[20] - Thetap10 + Thetap9) * (Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) +
			(-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
				(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * (Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) +
				(-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
					(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[20]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])))) / l3) *
			(Sin(x[20] - Thetap10 + Thetap9) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
				Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
					(-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
						Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
							2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) +
				Cos(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
					(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
						Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
							2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) +
				Sin(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
					(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
						Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
							2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) / l1) * (Sin(x[20] - Thetap10 + Thetap9) *
			((-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) +
					2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
						(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) +
						2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
							(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
				(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) +
					2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
						(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
					(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) +
						2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) -
							(2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])))) / lv +
		(Power(1 - sr.Cs * sr.d, 2) * Power(Sec(Thetap0), 2) * (-(Cos(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) -
			Sin(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) *
			(-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
				Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
					(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
						Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
							2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv)) / lv +
		(1 - sr.Cs * sr.d) * (-(Power(sr.Cs, 2) / (1 - sr.Cs * sr.d)) - sr.Cs * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) *
			Sin(x[18] - x[20] + Thetap8 - Thetap9) + Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv)) * Tan(Thetap0) -
		sr.Cs * ((1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sin(x[20] - Thetap10 + Thetap9) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
					Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
						2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
							2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv) +
			(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) *
				Sin(x[18] - x[20] + Thetap8 - Thetap9) + Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
				Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
					Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
					(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
						Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
							2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv) * Tan(Thetap0)) -
		sr.d * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv) * sr.Cs1 +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0) * sr.d * sr.Cs1) / Power(1 - sr.Cs * sr.d, 2)) - (Cos(Thetap0) * sr.Cs1) / (1 - sr.Cs * sr.d));

	alpha122 = (1 - sr.Cs * sr.d) * Sec(Thetap0) * ((sr.Cs * Sin(Thetap0)) / (1 - sr.Cs * sr.d) + (Sin(x[20] - Thetap10 + Thetap9) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
		Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
		(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
			2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
				2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
		Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
			Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
				2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
					2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) +
				Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				(-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) +
					2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) + (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) + Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) -
						2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - (-2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv) +
		(1 - sr.Cs * sr.d) * Sec(Thetap0) * (-((sr.Cs * Cos(Thetap0)) / (1 - sr.Cs * sr.d)) + (Sin(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) *
			Sin(x[18] - x[20] + Thetap8 - Thetap9) + Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
			Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
				(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
					Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
						2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv) * Tan(Thetap0);

	alpha123 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[20] - Thetap10 + Thetap9) * (Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
		Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) * Tan(x[4]) +
			Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
			2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[18])) -
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
			Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) * Tan(x[4]) +
				Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
				2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
					Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[18]))) * Tan(x[20])) -
		Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * (Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
			Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) * Tan(x[4]) +
				Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
				2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
					Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
				Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[18] - Thetap0 + Thetap8) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) * Tan(x[4]) +
					Cos(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4])) -
					2 * (-2 * Cos(x[4] - Thetap0 + Thetap1) * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) - 2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]) -
						Sin(x[18] - Thetap0 + Thetap8) * (2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Cos(x[4] - Thetap0 + Thetap1) * Sec(x[4]) + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1) * Tan(x[4]))) * Tan(x[18]))) * Tan(x[20])))) / lv;

	alpha1210 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[20] - Thetap10 + Thetap9) * (Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) +
		(-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * Power(Sec(x[18]), 2) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) -
			2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
			(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * Power(Sec(x[18]), 2) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) -
				2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
		Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * (Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) +
			(-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * Power(Sec(x[18]), 2) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) -
				2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8))) +
				(-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * Power(Sec(x[18]), 2) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) -
					2 * (-(Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1))) + 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) +
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])))) / lv;

	alpha1211 = ((1 - sr.Cs * sr.d) * Sec(Thetap0) * (Sin(x[20] - Thetap10 + Thetap9) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
		Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		Power(Sec(x[20]), 2) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) -
		(-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) -
		Cos(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			Power(Sec(x[20]), 2) * (-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) -
			(-((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9)) -
				Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) +
		Cos(x[20] - Thetap10 + Thetap9) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])) +
		Sin(x[20] - Thetap10 + Thetap9) * Tan(x[22]) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
			Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
			(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
				Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
					2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20])))) / lv;

	alpha1212 = -((Cos(x[20] - Thetap10 + Thetap9) * (1 - sr.Cs * sr.d) * Power(Sec(x[22]), 2) * Sec(Thetap0) * ((-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Sin(x[18] - x[20] + Thetap8 - Thetap9) +
		Cos(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
			2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18])) -
		(-(Cos(x[18] - x[20] + Thetap8 - Thetap9) * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8))) +
			Sin(x[18] - x[20] + Thetap8 - Thetap9) * (Cos(x[18] - Thetap0 + Thetap8) * (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) - 2 * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) * Sin(x[18] - Thetap0 + Thetap8) -
				2 * (-2 * Cos(x[18] - Thetap0 + Thetap8) * Sec(x[4]) * Sin(PAI / 6.0+ Thetap0 - Thetap1) * Sin(x[4] - Thetap0 + Thetap1) - (1 + 2 * Cos(PAI / 6.0+ Thetap0 - Thetap1) * Sec(x[4]) * Sin(x[4] - Thetap0 + Thetap1)) * Sin(x[18] - Thetap0 + Thetap8)) * Tan(x[18]))) * Tan(x[20]))) / lv);



	//摺動機構の目標相対姿勢角設計
	thetap8d = THETAP8D(0);
	dthetap8d = THETAP8D(1);
	K81 = K8_the(0);
	K82 = K8_the(1);
	K83 = K8_the(2);
	K84 = K8_the(3);

	thetap9d = THETAP9D(0);
	dthetap9d = THETAP9D(1);
	K91 = K9_the(0);
	K92 = K9_the(1);
	K93 = K9_the(2);
	K94 = K9_the(3);

        
	u10_a[0][0] = alpha1010 - (K81 / a0);
	u10_a[0][1] = -K82 / a0;
	u10_a[0][2] = -K83 / a0;
	u10_a[1][0] = alpha1110 - (K91 / a0);
	u10_a[1][1] = alpha1111 - (K92 / a0);
	u10_a[1][2] = -K93 / a0;
	u10_a[2][0] = alpha1210;
	u10_a[2][1] = alpha1211;
	u10_a[2][2] = alpha1212;

	b10 = K84 / a0 + (k17 + k18) * (dthetap8d / a0 - z101) + k17 * k18 * (thetap8d / a0 - z102 / a0) - (alpha101 * a0 + alpha102 * u2 + alpha103 * u3);
	b11 = K94 / a0 + (k19 + k20) * (dthetap9d / a0 - z111) + k19 * k20 * (thetap9d / a0 - z112 / a0) - (alpha111 * a0 + alpha112 * u2 + alpha113 * u3);
	b12 = ddthetap10d / a0 + (k21 + k22) * ((dthetap10d / a0) - z121) + k21 * k22 * ((thetap10d / a0) - z122 / a0) - (alpha121 * a0 + alpha122 * u2 + alpha123 * u3);

	inv(u10_a, u10_inv_a);

	u10and = u10_inv_a[0][0] * b10 + u10_inv_a[0][1] * b11 + u10_inv_a[0][2] * b12;
	u11and = u10_inv_a[1][0] * b10 + u10_inv_a[1][1] * b11 + u10_inv_a[1][2] * b12;
	u12and = u10_inv_a[2][0] * b10 + u10_inv_a[2][1] * b11 + u10_inv_a[2][2] * b12;

	u10 = u10and;
	u11 = u11and;
	u12 = u12and;

    u_kinematics(9) = u10;
	u_kinematics(10) = u11;
	u_kinematics(11) = u12;

    w10 = (1/ a0)*(K81*u_kinematics(9) + K82*u_kinematics(10) + K83*u_kinematics(11) + K84) + (k17 + k18) * (dthetap8d / a0 - z101) + k17 * k18 * (thetap8d / a0 - z102 / a0);
	w11 = (1/ a0)*(K91*u_kinematics(9) + K92*u_kinematics(10) + K93*u_kinematics(11) + K94) + (k19 + k20) * (dthetap9d / a0 - z111) + k19 * k20 * (thetap9d / a0 - z112 / a0);
	w12 = ddthetap10d / a0 + (k21 + k22) * ((dthetap10d / a0) - z121) + k21 * k22 * ((thetap10d / a0) - z122 / a0);
   

   // // ... 係数の計算（alpha****, K**, a0, k**, thetap**, dthetap**, ddthetap**, z**, u2, u3）までは今まで通り ...

   // // --- A と b の構築（u10,u11,u12 ブロック）---
   // // --- 係数行列・右辺の組み立て ---
   // Eigen::Matrix3d A10;
   // A10 <<  alpha1010 - (K81 / a0),   -K82 / a0,              -K83 / a0,
   //         alpha1110 - (K91 / a0),   alpha1111 - (K92 / a0), -K93 / a0,
   //         alpha1210,                alpha1211,              alpha1212;

   // Eigen::Vector3d b10v;
   // b10v << ( K84 / a0
   //           + (k17 + k18) * (dthetap8d / a0 - z101)
   //           + (k17 * k18) * (thetap8d / a0 - z102 / a0)
   //           - (alpha101 * a0 + alpha102 * u2 + alpha103 * u3) ),
   //         ( K94 / a0
   //           + (k19 + k20) * (dthetap9d / a0 - z111)
   //           + (k19 * k20) * (thetap9d / a0 - z112 / a0)
   //           - (alpha111 * a0 + alpha112 * u2 + alpha113 * u3) ),
   //         ( ddthetap10d / a0
   //           + (k21 + k22) * (dthetap10d / a0 - z121)
   //           + (k21 * k22) * (thetap10d / a0 - z122 / a0)
   //           - (alpha121 * a0 + alpha122 * u2 + alpha123 * u3) );

   // // --- 解法：CompleteOrthogonalDecomposition（最小二乗/ランク落ちに強い）---
   // Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> cod;
   // cod.setThreshold(1e-12);               // 数値スケールに応じて調整可
   // cod.compute(A10);

   // // ランク判定（3x3なので rank==3 が正則）
   // Eigen::Vector3d x10;
   // if (cod.rank() < 3) {
   //     // ランク落ち時：ティホノフ正則化付き最小二乗でフォールバック
   //     const double lambda = 1e-8;        // 必要に応じて 1e-10〜1e-6 で調整
   //     const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
   //     x10 = (A10.transpose() * A10 + lambda * I).ldlt().solve(A10.transpose() * b10v);
   // } else {
   //     // フルランク：CODの解を採用
   //     x10 = cod.solve(b10v);
   // }

   // // 残差チェック（任意：ログ用）
   // const double rel_err = (A10 * x10 - b10v).norm() / (b10v.norm() + 1e-12);
   // // 例: ROS_DEBUG_STREAM("U10-12 solve rel_err=" << rel_err);

   // // --- 代入 ---
   // u10and = x10(0);
   // u11and = x10(1);
   // u12and = x10(2);

   // u10 = u10and;
   // u11 = u11and;
   // u12 = u12and;


}