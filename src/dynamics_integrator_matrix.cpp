#include "cooperative_transportation_4ws_backstepping/dynamics_integrator.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"        // 数学関数のヘッダーファイル
#include "cooperative_transportation_4ws_backstepping/Bezier.h"         // Bezier 曲線の関数
#include "cooperative_transportation_4ws_backstepping/vehicle.hpp"       // Vehicle クラスの宣言
#include "cooperative_transportation_4ws_backstepping/callback.hpp"     // コールバック関数の宣言
#include "cooperative_transportation_4ws_backstepping/getInputValue.hpp"
#include "cooperative_transportation_4ws_backstepping/differential_equations.hpp"
#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/SVD> 
#include <Eigen/Core>   
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>

// ここで IOFormat を定義しておく
static const Eigen::IOFormat CleanFmt(
    /*precision=*/6,
    /*flags=*/Eigen::DontAlignCols,
    /*coeffSeparator=*/", ",
    /*rowSeparator=*/"\n",
    /*rowPrefix=*/"[", 
    /*rowSuffix=*/"]"
);

using namespace std;

DynamicsIntegrator::DynamicsIntegrator(double dt)
                                       :kinematics_solver_()

 {}


//状態変数の目標加速度を計算
Eigen::Matrix<double,23,1> DynamicsIntegrator::computeXAlpha(
    const std::vector<double>& x,
    const std::vector<double>& x_d,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {
      double x0dot   = x_d[1];
      double y0dot   = x_d[2];
      double thetap0dot = u_kinematics(1);
      double phi1dot = u_kinematics(2);
      double phi2dot = u_kinematics(3);
      double phi3dot = x_d[8];
      double phi4dot = x_d[10];
      double phi5dot = u_kinematics(6);
      double phi6dot = x_d[14];
      double phi7dot = x_d[16];
      double phi8dot = u_kinematics(9);
      double phi9dot = x_d[20];
      double phi10dot = x_d[22];

      Eigen::Matrix<double,23,12> SX =  kinematics_solver_.SX_mat();
      Eigen::Matrix<double,23,12> dSXdt =  kinematics_solver_.dSXdt_mat();
      //Eigen::Matrix<double,12,1> pdud =  kinematics_solver_.pd_ud_vec();
      Eigen::Matrix<double,12,23> udpdX =  kinematics_solver_.udpdX_mat();
      

      
      //ゲイン
      Eigen::Matrix<double,12,1> gains;
      gains <<  5.0,10.0,10.0,0.0,20.0,20.0,0.0,20.0,20.0,0.0,20.0,20.0;
      Eigen::Matrix<double,12,12> C = gains.asDiagonal();
      

      // 実際の前進速度と前輪操舵角速度
      u1_act = x0dot * cos(x_old[3]) + y0dot * sin(x_old[3]);
      u2_act = thetap0dot;
      u3_act = phi1dot;
      u4_act = phi2dot;
      u5_act = phi3dot;
      u6_act = phi4dot;
      u7_act = phi5dot;
      u8_act = phi6dot;
      u9_act = phi7dot;
      u10_act = phi8dot;
      u11_act = phi9dot;
      u12_act = phi10dot;

      Eigen::Matrix<double,23,1> Xalpha;
      
      // 結果格納用ベクトル
      u_act<<
          u1_act, u2_act, u3_act, u4_act, u5_act, u6_act, 
          u7_act, u8_act, u9_act, u10_act, u11_act, u12_act;
      
      
    
      //偏差ベクトル
      Eigen::Matrix<double,12,1> r_b = u_act - u_kinematics;   
      Eigen::Matrix<double,12,1> dot_C_rb = C*r_b;

      //thetapの2,3階微分を計算
      //getDtheta(nu, u_kinematics);

      Eigen::Matrix<double,23,1> Xdot = SX * u_act;
      //目標加速度νを計算
      nu = -dot_C_rb + udpdX * Xdot;
      
      
      //状態変数ベクトルの目標加速度
      Xalpha = dSXdt * u_act + SX * nu;

      //デバッグ用ログ出力
	  ROS_INFO_THROTTLE(0.04,"Ps: t = %.3f x=%.3f, y=%.3f, Q=%d, thetap0=%.3f", x_old[0], sr.Psx, sr.Psy, sr.j, Thetap0);
      ROS_INFO_THROTTLE(0.04,"u_kinematics: 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f, 7=%.3f, 8=%.3f,9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f", u_kinematics(0), u_kinematics(1), u_kinematics(2), u_kinematics(3), u_kinematics(4), u_kinematics(5), u_kinematics(6), u_kinematics(7), u_kinematics(8), u_kinematics(9), u_kinematics(10), u_kinematics(11));
	  ROS_INFO_THROTTLE(0.04,"u_act: 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f, 7=%.3f, 8=%.3f,9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f", u_act(0), u_act(1), u_act(2), u_act(3), u_act(4), u_act(5), u_act(6), u_act(7), u_act(8), u_act(9), u_act(10), u_act(11));
	  ROS_INFO_THROTTLE(0.04,"nu: 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f, 7=%.3f, 8=%.3f,9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f", nu(0), nu(1), nu(2), nu(3), nu(4), nu(5), nu(6), nu(7), nu(8), nu(9), nu(10), nu(11));
      
      return Xalpha;
    }


//一般化座標の目標加速度を計算
Eigen::Matrix<double,27,1> DynamicsIntegrator::computeAlpha(
    const Eigen::Matrix<double,27,1>& q,
    const Eigen::Matrix<double,27,1>& qdot,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {
        Eigen::Matrix<double,27,1> alpha;
        //状態変数ベクトルの目標加速度 
        Eigen::Matrix<double,23,1> Xalpha = computeXAlpha(x_old, x_d, u_kinematics);
        
        ROS_INFO_THROTTLE(0.04,"q:ca: x1=%.3f, y0=%.3f, theta0=%.3f", q(0), q(1), q(2));
        ROS_INFO_THROTTLE(0.04,"q:v1: x1=%.3f, y1=%.3f, thetav1=%.3f, s1=%.3f, v1f=%.3f, v1r=%.3f", q(3), q(4), q(5), q(6), v1f, v1r);
	    ROS_INFO_THROTTLE(0.04,"q:v2: x2=%.3f, y2=%.3f, thetav2=%.3f, s2=%.3f, v2f=%.3f, v2r=%.3f", q(7), q(8), q(9), q(10), v2f, v2r);
	    ROS_INFO_THROTTLE(0.04,"q:v3: x3=%.3f, y3=%.3f, thetav3=%.3f, s3=%.3f, v3f=%.3f, v3r=%.3f", q(11), q(12), q(13), q(14), v3f, v3r);
        ROS_INFO_THROTTLE(0.04,"q_dot:velca: x1=%.3f, y0=%.3f, theta0=%.3f", q(0), q(1), q(2));
        ROS_INFO_THROTTLE(0.04,"q_dot:velv1: phiR1=%.3f, varphiR1=%.3f, phiF1=%.3f, varphiF1=%.3f", qdot(15), qdot(16), qdot(17), qdot(18));
	    ROS_INFO_THROTTLE(0.04,"q_dot:velv2: phiR2=%.3f, varphiR2=%.3f, phiF2=%.3f, varphiF2=%.3f", qdot(19), qdot(20), qdot(21), qdot(22));
	    ROS_INFO_THROTTLE(0.04,"q_dot:velv3: phiR3=%.3f, varphiR3=%.3f, phiF3=%.3f, varphiF3=%.3f", qdot(23), qdot(24), qdot(25), qdot(26));

        asd =         Xalpha(0);
        athetap4d =   Xalpha(10);
        athetap7d =   Xalpha(16);
        athetap10d =  Xalpha(22);

        alpha = kinematics_solver_.aqd_vec();

        return alpha;
      }


void DynamicsIntegrator::step(
    const Eigen::Matrix<double,27,1>& q,
    const Eigen::Matrix<double,27,1>& qdot,
    const Eigen::Matrix<double,12,1>& u_kinematics)
    {

      Eigen::Matrix<double,27,27> C =  kinematics_solver_.Cxi_mat();
      Eigen::Matrix<double,27,27> M =  kinematics_solver_.Mxi_mat();
      Eigen::Matrix<double,18,27> A =  kinematics_solver_.Axi_mat();
      Eigen::Matrix<double,27,1>  K =  kinematics_solver_.Kxi_vec();

      //目標加速度の取得
      Eigen::Matrix<double,27,1> alpha = computeAlpha(q, qdot, u_kinematics);


      //qの目標加速度を分解
      constexpr int NXI = 15;
      constexpr int NZETA = 12;
          
      Eigen::Matrix<double,NXI,1>  alpha_xi  = alpha.head<NXI>();
      Eigen::Matrix<double,NZETA,1> alpha_zeta = alpha.tail<NZETA>();
          
      Eigen::Matrix<double,NXI,1>  qdot_xi   = qdot.head<NXI>();
      Eigen::Matrix<double,NZETA,1> qdot_zeta  = qdot.tail<NZETA>();



      // ラムダの導出
      //慣性行列 M (27x27)
      Eigen::Matrix<double,NXI,   NXI>   M_xixi   = M.topLeftCorner   (NXI,   NXI);  
      Eigen::Matrix<double,NXI,   NZETA> M_xizeta = M.topRightCorner  (NXI,   NZETA);
      Eigen::Matrix<double,NZETA, NXI>   M_zetaxi = M.bottomLeftCorner(NZETA, NXI);  
      Eigen::Matrix<double,NZETA, NZETA> M_zetazeta = M.bottomRightCorner(NZETA, NZETA);

      //コリオリ行列C
      Eigen::Matrix<double,NXI,   NXI>   C_xixi   = C.topLeftCorner   (NXI,   NXI);
      Eigen::Matrix<double,NXI,   NZETA> C_xizeta = C.topRightCorner  (NXI,   NZETA);
      Eigen::Matrix<double,NZETA, NXI>   C_zetaxi = C.bottomLeftCorner(NZETA, NXI);
      Eigen::Matrix<double,NZETA, NZETA> C_zetazeta = C.bottomRightCorner(NZETA, NZETA);

      //拘束行列 A 
      Eigen::Matrix<double,18, NXI>   A_xi   = A.leftCols (NXI);   
      Eigen::Matrix<double,18, NZETA> A_zeta = A.rightCols(NZETA); 

      //Aを転置
      Eigen::Matrix<double,27,18> AT = A.transpose();      
      Eigen::Matrix<double,NXI,   18> AT_xi   = AT.topRows(NXI); 
      Eigen::Matrix<double,NZETA, 18> AT_zeta = AT.bottomRows(NZETA);

      //ddotq,dotqを含まない項K
      Eigen::Matrix<double, NXI,   1> K_xi   = K.topRows   (NXI);
      Eigen::Matrix<double, NZETA, 1> K_zeta = K.bottomRows(NZETA);

      
      Eigen::Matrix<double,15,1> rhs =  (M_xixi * alpha_xi + C_xixi * qdot_xi + K_xi);
      // COD による最小ノルム解
      Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double,15,18>> cod(AT_xi);
      Eigen::Matrix<double,18,1> lambda_vec = cod.solve(rhs);
      lambda_data = lambda_vec;

      ROS_INFO_THROTTLE(0.04,"lambda_vec: la1=%.3f, la2=%.3f, la3=%.3f, la4=%.3f, la5=%.3f, la6=%.3f, la7=%.3f, la8=%.3f,la9=%.3f, la10=%.3f, la11=%.3f, la12=%.3f", 
        lambda_vec(0), lambda_vec(1), lambda_vec(2), lambda_vec(3), lambda_vec(4), lambda_vec(5), lambda_vec(6), lambda_vec(7), lambda_vec(8), lambda_vec(9), lambda_vec(10), lambda_vec(11));
	  ROS_INFO_THROTTLE(0.04,"lambda_vec: la13=%.3f, la14=%.3f, la15=%.3f, la16=%.3f, la17=%.3f, la18=%.3f", lambda_vec(12), lambda_vec(13), lambda_vec(14), lambda_vec(15), lambda_vec(16), lambda_vec(17));



      //駆動力の導出
      Eigen::Matrix<double,12,1> rhs_zeta_only = M_zetazeta * alpha_zeta + C_zetazeta * qdot_zeta  + K_zeta;                   
      //拘束力を引いて最終的な駆動トルク
      Eigen::Matrix<double,12,1> Q_zeta = rhs_zeta_only - AT_zeta * lambda_vec;


      //駆動力計算
      Q_phiR1   = Q_zeta(0);
      Q_varphiR1 = Q_zeta(1);
      Q_phiF1 = Q_zeta(2);
      Q_varphiF1 = Q_zeta(3);
      Q_phiR2   = Q_zeta(4);
      Q_varphiR2 = Q_zeta(5);
      Q_phiF2 = Q_zeta(6);
      Q_varphiF2 = Q_zeta(7);
      Q_phiR3   = Q_zeta(8);
      Q_varphiR3 = Q_zeta(9);
      Q_phiF3 = Q_zeta(10);
      Q_varphiF3 = Q_zeta(11);

      //各車輪の回転トルクを計算
      // 車両1
      std::array<double,2> v1_rearTorque;  
      std::array<double,2> v1_frontTorque;
      v1_rearTorque  = computeRearWheelTorque(Q_varphiR1, q(17),  q(15));
      v1_frontTorque = computeFrontWheelTorque(Q_varphiF1, q(17), q(15));

      // 車両2
      std::array<double,2> v2_rearTorque;  
      std::array<double,2> v2_frontTorque;
      v2_rearTorque  = computeRearWheelTorque(Q_varphiR2, q(21), q(19));
      v2_frontTorque = computeFrontWheelTorque(Q_varphiF2, q(21), q(19));

      // 車両3
      std::array<double,2> v3_rearTorque;  
      std::array<double,2> v3_frontTorque;
      v3_rearTorque  = computeRearWheelTorque(Q_varphiR3, q(25), q(23));
      v3_frontTorque = computeFrontWheelTorque(Q_varphiF3, q(25), q(23));

      v1_torque_rear[0] = v1_rearTorque[0];  // 左後輪
      v1_torque_rear[1] = v1_rearTorque[1];  // 右後輪
      v1_torque_front[0] = v1_frontTorque[0];  // 左後輪
      v1_torque_front[1] = v1_frontTorque[1];  // 右後輪

      v2_torque_rear[0] = v2_rearTorque[0];  // 左後輪
      v2_torque_rear[1] = v2_rearTorque[1];  // 右後輪
      v2_torque_front[0] = v2_frontTorque[0];  // 左後輪
      v2_torque_front[1] = v2_frontTorque[1];  // 右後輪

      v3_torque_rear[0] = v3_rearTorque[0];  // 左後輪
      v3_torque_rear[1] = v3_rearTorque[1];  // 右後輪
      v3_torque_front[0] = v3_frontTorque[0];  // 左後輪
      v3_torque_front[1] = v3_frontTorque[1];  // 右後輪

      //各車輪のステアリングトルクを計算
      Q_phiFL1 = Q_phiF1/2.0;
      Q_phiFR1 = Q_phiF1/2.0; 
      Q_phiRL1 = Q_phiR1/2.0; 
      Q_phiRR1 = Q_phiR1/2.0;

      Q_phiFL2 = Q_phiF2/2.0;
      Q_phiFR2 = Q_phiF2/2.0; 
      Q_phiRL2 = Q_phiR2/2.0; 
      Q_phiRR2 = Q_phiR2/2.0;

      Q_phiFL3 = Q_phiF3/2.0;
      Q_phiFR3 = Q_phiF3/2.0; 
      Q_phiRL3 = Q_phiR3/2.0; 
      Q_phiRR3 = Q_phiR3/2.0;

      ROS_INFO_THROTTLE(0.04,"a_qd ca: a_x0d=%.3f, a_y0d=%.3f, a_theta0d=%.3f", alpha(0),alpha(1), alpha(2));
      ROS_INFO_THROTTLE(0.04,"a_qd v1: a_x1d=%.3f, a_y1d=%.3f, a_theta1d=%.3f", alpha(4),alpha(5), alpha(6));
	  ROS_INFO_THROTTLE(0.04,"a_qd v2: a_x2d=%.3f, a_y2d=%.3f, a_theta2d=%.3f", alpha(8),alpha(9), alpha(10));
	  ROS_INFO_THROTTLE(0.04,"a_qd v3: a_x3d=%.3f, a_y3d=%.3f, a_theta3d=%.3f", alpha(12),alpha(13), alpha(14));

    //   ROS_INFO_THROTTLE(0.04,"v1: Q_phiR1=%.3f, Q_phiF1=%.3f, Q_varphiR1=%.3f, Q_varphiF1=%.3f", Q_phiR1, Q_phiF1, Q_varphiR1, Q_varphiF1);
	//   ROS_INFO_THROTTLE(0.04,"v2: Q_phiR2=%.3f, Q_phiF2=%.3f, Q_varphiR2=%.3f, Q_varphiF2=%.3f", Q_phiR2, Q_phiF2, Q_varphiR2, Q_varphiF2);
	//   ROS_INFO_THROTTLE(0.04,"v3: Q_phiR3=%.3f, Q_phiF3=%.3f, Q_varphiR3=%.3f, Q_varphiF3=%.3f", Q_phiR3, Q_phiF3, Q_varphiR3, Q_varphiF3);

      Eigen::Matrix<double,12,1> Q_rhs     = rhs_zeta_only;
      Eigen::Matrix<double,12,1> Q_const   = - AT_zeta * lambda_vec;
      Eigen::Matrix<double,12,1> Q_total   = Q_rhs + Q_const; // = Q_zeta

      int idx_varphiR1 =1;
      int idx_varphiR2 =5;
      int idx_varphiR3 =9;

      ROS_INFO_THROTTLE(0.04,"v1 varphiR1: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f | varphif1: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f",Q_rhs(1),Q_const(1),Q_total(1), Q_rhs(3),Q_const(3),Q_total(3));
      ROS_INFO_THROTTLE(0.04,"v2 varphiR2: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f | varphif2: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f",Q_rhs(5),Q_const(5),Q_total(5), Q_rhs(7),Q_const(7),Q_total(7));
      ROS_INFO_THROTTLE(0.04,"v3 varphiR3: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f | varphif3: rhs=%.3f, A*lambda_vec=%.3f, total=%.3f\n\n",Q_rhs(9),Q_const(9),Q_total(9) ,Q_rhs(11),Q_const(11),Q_total(11));
     
     
     

      //積分用の配列に代入
      x_dd[0] = 0.0;
      x_dd[1] = 0.0;
      x_dd[2] = 0.0;
      x_dd[3] = 0.0;
      x_dd[4] = 0.0;
      x_dd[5] = 0.0;
      x_dd[6] = 0.0;
      x_dd[7] = 0.0;


}

//内輪差考慮
array<double,2>  DynamicsIntegrator::computeRearWheelOmegas(double speed, double steeringAngle) {
    const double W = 0.05;            // トレッド幅[m]
    array<double,2> omegas;

    if (fabs(steeringAngle) < 1e-6) {
        double omega = speed / wheelRadius;
        omegas[0] = omega;
        omegas[1] = omega;
        return omegas;
    }
    double absPhi = fabs(steeringAngle);
    double R = lv / tan(absPhi);
    double R_in  = R - W/2.0;
    double R_out = R + W/2.0;
    double v_in  = speed * (R_in  / R);
    double v_out = speed * (R_out / R);
    double omega_in  = v_in  / wheelRadius;
    double omega_out = v_out / wheelRadius;

    if (steeringAngle > 0) {
        // 左折: 左が内輪
        omegas[0] = omega_in;
        omegas[1] = omega_out;
    } else {
        // 右折: 右が内輪
        omegas[0] = omega_out;
        omegas[1] = omega_in;
    }
    return omegas;
}



std::array<double, 2> DynamicsIntegrator::computeFrontWheelTorque(
    double Qf,
    double steeringAngleFront,
    double steeringAngleRear)
{
    const double Wf = 0.8; 
    std::array<double, 2> torques;

    double tan_diff = std::tan(steeringAngleFront) - std::tan(steeringAngleRear);
    
    // 直進時は均等配分
    if (std::fabs(tan_diff) < 1e-9) {
        torques[0] = Qf * 0.5;
        torques[1] = Qf * 0.5;
        return torques;
    }

    // 1. 旋回半径の近似 (車両中心または後軸基準のR)
    // 4WSの幾何学中心までの距離 R
    double R_eff = std::abs(lv / tan_diff);

    // 2. 内外輪の半径差 (トレッド幅による)
    double R_inner = R_eff - Wf / 2.0;
    double R_outer = R_eff + Wf / 2.0;
    double sum_R = R_inner + R_outer;

    // 3. トルク配分
    // 【修正】旋回を助ける場合: 外輪トルク > 内輪トルク
    // トルクは半径に比例させるのが一般的 (駆動力 F を均等にする考え方なら T = F*r なので r に比例)
    
    double T_inner = Qf * (R_inner / sum_R); // 半径が小さい方に小さいトルク
    double T_outer = Qf * (R_outer / sum_R); // 半径が大きい方に大きいトルク

    // 4. 左右への割り当て (tan_diff > 0 は左旋回と仮定)
    if (tan_diff > 0) {
        // 左旋回: 左が内輪
        torques[0] = T_inner; // Left
        torques[1] = T_outer; // Right
    } else {
        // 右旋回: 右が内輪
        torques[0] = T_outer; // Left
        torques[1] = T_inner; // Right
    }
    return torques;
}

std::array<double, 2> DynamicsIntegrator::computeRearWheelTorque(
    double Qr,                  // 【変更】入力は後輪合計トルク
    double steeringAngleFront,
    double steeringAngleRear)
{
    const double Wr = 0.8;      // 【変更】後輪のトレッド幅 (前輪と同じならそのままでOK)
    std::array<double, 2> torques;

    // 旋回の厳しさ（曲率）を決めるのは「前後輪の操舵角の差」
    // これは前輪計算時と全く同じロジックでOK
    double tan_diff = std::tan(steeringAngleFront) - std::tan(steeringAngleRear);
    
    // 直進時（またはカニ歩き時）は均等配分
    if (std::fabs(tan_diff) < 1e-9) {
        torques[0] = Qr * 0.5;
        torques[1] = Qr * 0.5;
        return torques;
    }

    // 1. 旋回半径の近似
    // 4WSにおける旋回半径の定義は前後共通の指標を使って問題ありません
    double R_eff = std::abs(lv / tan_diff);

    // 2. 内外輪の半径差 (後輪トレッド幅 Wr を使用)
    double R_inner = R_eff - Wr / 2.0;
    double R_outer = R_eff + Wr / 2.0;
    double sum_R = R_inner + R_outer;

    // 3. トルク配分 (外輪トルク > 内輪トルク)
    double T_inner = Qr * (R_inner / sum_R);
    double T_outer = Qr * (R_outer / sum_R);

    // 4. 左右への割り当て (tan_diff > 0 は左旋回)
    // 旋回方向の判定も前後で共通です
    if (tan_diff > 0) {
        // 左旋回: 左(0)が内輪、右(1)が外輪
        torques[0] = T_inner; // Left (Rear)
        torques[1] = T_outer; // Right (Rear)
    } else {
        // 右旋回: 右(1)が内輪、左(0)が外輪
        torques[0] = T_outer; // Left (Rear)
        torques[1] = T_inner; // Right (Rear)
    }
    return torques;
}

// void DynamicsIntegrator::getDtheta(const Eigen::Matrix<double,12,1>& nu, const Eigen::Matrix<double,12,1>& u_kinematics){
//     Eigen::Matrix<double,4,1> Dthetap2 =  kinematics_solver_.Dthetap2_vec();
//     Eigen::Matrix<double,4,1> Dthetap5 =  kinematics_solver_.Dthetap5_vec();
//     Eigen::Matrix<double,4,1> Dthetap8 =  kinematics_solver_.Dthetap8_vec();

//    //thetap2dの２階微分と３階微分
//     ddthetap2d =  Dthetap2(0);
//     dddthetap2d = Dthetap2(1); 
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //thetap3dの２階微分と３階微分
//     ddthetap3d = Dthetap2(2) ;
//     dddthetap3d =Dthetap2(3) ; 
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
//     //thetap5dの２階微分と３階微分
//     ddthetap5d =  Dthetap5(0);
//     dddthetap5d = Dthetap5(1); 
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //thetap6dの２階微分と３階微分
//     ddthetap6d = Dthetap5(2) ;
//     dddthetap6d =Dthetap5(3) ; 
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   
//     //thetap8dの２階微分と３階微分
//     ddthetap8d =  Dthetap8(0);
//     dddthetap8d = Dthetap8(1); 
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //thetap9dの２階微分と３階微分
//     ddthetap9d = Dthetap8(2) ;
//     dddthetap9d =Dthetap8(3) ; 

//     //  ROS_INFO_THROTTLE(0.04,"v1: thetap2d=%.3f, dthetap2d=%.3f, ddthetap2d=%.3f, dddthetap2d=%.3f, thetap3d=%.3f, dthetap3d=%.3f, ddthetap3d=%.3f, dddthetap3d=%.3f", thetap2d, dthetap2d, Dthetap2(0), Dthetap2(1), thetap3d, dthetap3d, Dthetap2(2), Dthetap2(3));
//     //  ROS_INFO_THROTTLE(0.04,"v2: thetap5d=%.3f, dthetap5d=%.3f, ddthetap5d=%.3f, dddthetap5d=%.3f, thetap6d=%.3f, dthetap6d=%.3f, ddthetap6d=%.3f, dddthetap6d=%.3f", thetap5d, dthetap5d, Dthetap5(0), Dthetap5(1), thetap6d, dthetap6d, Dthetap5(2), Dthetap5(3));
//     //  ROS_INFO_THROTTLE(0.04,"v3: thetap8d=%.3f, dthetap8d=%.3f, ddthetap8d=%.3f, dddthetap8d=%.3f, thetap9d=%.3f, dthetap9d=%.3f, ddthetap9d=%.3f, dddthetap9d=%.3f\n\n", thetap8d, dthetap8d, Dthetap8(0), Dthetap8(1), thetap9d, dthetap9d, Dthetap8(2), Dthetap8(3));


// }




// void DynamicsIntegrator::step(const Eigen::Vector3d& q,
//                               const Eigen::Vector3d& qdot,
//                               double& phi,
//                               double& phidot,
//                               double u1,
//                               double u2)
// {
//     // 状態展開
//     double x        = q(0);
//     double y        = q(1);
//     double theta    = q(2);
//     double xdot     = qdot(0);
//     double ydot     = qdot(1);
//     double thetadot = qdot(2);

//     //PID制御でtauを計算
//     double u1_act = xdot * cos(theta) + ydot * sin(theta);
//     double tau1   = drive_pid_.compute(u1, u1_act);
//     Tau1 = tau1;
//     double tau2   = steer_pid_.compute(u2, phidot);
//     Tau2 = tau2;
//     //駆動力
//     Eigen::Vector3d Qc;
//     Qc << tau1 * std::cos(theta),
//           tau1 * std::sin(theta),
//           0.0;
//     double Qphi = tau2;

//     //各行列を定義
//     // 質量行列(3x3)
//     Eigen::Matrix3d Mxi;
//     Mxi <<  m_b, 0.0, -(lv *m_b * sin(theta))/2.0,
//             0.0, m_b, (lv *m_b * cos(theta))/2.0,
//             -(lv *m_b * sin(theta))/2.0, (lv *m_b * cos(theta))/2.0, (2*I_theta_ + m_b*((pow(lv,2)*pow(cos(theta),2))/2.0 + (pow(lv,2)*pow(sin(theta),2))/2.0))/2.0;

//     // コリオリ行列(3x3)
//     Eigen::Matrix3d Cxi;
//     Cxi <<  0.0, 0.0, -(lv*m_b*cos(theta)*thetadot),
//             0.0, 0.0, -(lv*m_b*sin(theta)*thetadot),
//             0.0, 0.0, 0.0;

//     // 重力ベクトル(3x1)
//     Eigen::Vector3d Kxi;
//     Kxi << GRAV*m_b*sin(rho_), 0.0, -(GRAV*lv*m_b*sin(rho_)*sin(theta))/2.0;

//     // 拘束行列(3x2)
//     Eigen::Matrix<double,3,2> Axi;
//     Axi <<  sin(theta + phi), sin(theta),
//             -cos(theta + phi), -cos(theta),
//             -lv * cos(phi), 0.0;

//     // ヤコビ行列(2x3)
//     Eigen::Matrix<double,2,3> J;
//     J <<  sin(theta + phi), -cos(theta + phi), -lv * cos(phi),
//             sin(theta), -cos(theta), 0.0;

//     // 拘束付き運動方程式行列 H (5x5)
//     Eigen::Matrix<double,5,5> H;
//     H.setZero();
//     H.block<3,3>(0,0) = Mxi;
//     H.block<3,2>(0,3) = Axi;
//     H.block<2,3>(3,0) = J;
//     // H.block<2,2>(3,3) = Zero

//     // 右辺ベクトル b (5x1)
//     Eigen::Matrix<double,5,1> b;
//     b.setZero();
//     b.block<3,1>(0,0) = Qc - Cxi * qdot - Kxi;
//     // 非ホロを微分した値dのddq以外の項を代入
//     b(3) = -xdot*(thetadot+phidot)*cos(theta+phi) - ydot*(thetadot+phidot)*sin(theta+phi)-lv*thetadot*phidot*sin(phi);
//     b(4) = -xdot*thetadot*cos(theta) + ydot*thetadot*sin(theta);

//     // Hの疑似逆行列を計算
//     Eigen::MatrixXd Hxi_pinv = (H.transpose() * H).inverse()* H.transpose();
//     Eigen::VectorXd sol= Hxi_pinv * b;
    
//     Eigen::Vector3d qdd = sol.block<3,1>(0,0);
//     Eigen::Vector2d lambda_vec = sol.block<2,1>(3,0); 
//     double phidd = Qphi;

//     //積分用の配列に代入
//     x_dd[0] = 0.0;
//     x_dd[1] = qdd(0);
//     x_dd[2] = qdd(1);
//     x_dd[3] = qdd(2);
//     x_dd[4] = phidd;
// }