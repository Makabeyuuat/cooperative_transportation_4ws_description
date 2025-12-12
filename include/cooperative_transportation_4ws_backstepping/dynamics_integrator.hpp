#pragma once

#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/getInputValue.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>

// ========= ここから追加：4輪Ni補償 用の型 =========

// 各輪の角速度（ステア/回転）を渡すための簡単な入れ物
struct WheelState4 {
  // ステア角速度 [rad/s]
  double phiRRdot{0}, phiRLdot{0}, phiFRdot{0}, phiFLdot{0};
  // 車輪回転角速度 [rad/s]
  double varphiRRdot{0}, varphiRLdot{0}, varphiFRdot{0}, varphiFLdot{0};
};

// 物理・幾何パラメータ一式
struct NiCompParams {
  // 物理・幾何
  double m{0}, g{9.80665}, L{0}, t{0}, h{0}, r{0};
  // 重心からの半距離（一般式用）: lF+lR=L, wL+wR=t
  double lF{0}, lR{0}, wL{0}, wR{0};

  // 接触/抵抗パラメータ
  double mu_t{1.0};     // 接地（縦）摩擦：飽和上限に使用
  double mu_r{0.015};   // 転がり抵抗係数
  // クーロン補償係数: 名目friction/N0で同定（N0=mg/4）
  double kc_drive{0};   // 車輪回転側（varphi）に使う
  double kc_steer{0};   // ステア側（phi）に使う
  // 粘性
  double b_drive{0};    // 車輪回転用の粘性 [N·m·s/rad]
  double b_steer{0};    // ステア用の粘性 [N·m·s/rad]

  // 数値安定用
  double eps{1e-3};

  // 任意：重力除去後の実前後/左右加速度（あるなら）
  double ax_dyn{0.0};   // 車体x前後[m/s^2]（+で後輪側に荷重転移）
  double ay_dyn{0.0};   // 車体y左右[m/s^2]（+で右側に荷重転移）
};

// 関数の出力（補正トルクと、デバッグ/飽和に必要なNi等）
struct NiCompTorques4 {
  // これを Q_* に「加算」してください
  double d_phiRR{0},    d_phiRL{0},    d_phiFR{0},    d_phiFL{0};       // ステア補正
  double d_varphiRR{0}, d_varphiRL{0}, d_varphiFR{0}, d_varphiFL{0};    // 回転補正

  // 各輪の鉛直荷重と、回転トルクの飽和上限 τ_max = r*mu_t*Ni
  double N_RR{0}, N_RL{0}, N_FR{0}, N_FL{0};
  double tau_max_RR{0}, tau_max_RL{0}, tau_max_FR{0}, tau_max_FL{0};
};



class DynamicsIntegrator {
public:

    //新規メソッド: バックステッピングから得られる目標加速度を計算する 
    Eigen::Matrix<double,27,1> computeAlpha(
        const Eigen::Matrix<double,27,1>& q, // 状態ベクトル (x,y,θ,φ,ψ_f,ψ_r)
        const Eigen::Matrix<double,27,1>& qdot,     // 速度ベクトル         
        const Eigen::Matrix<double,12,1>& u_kinematics);

    Eigen::Matrix<double,23,1> computeXAlpha(
        const std::vector<double>& x_d,
        const std::vector<double>& x_dd,
        const Eigen::Matrix<double,12,1>& u_kinematics);

     Eigen::Matrix<double,27,1> computeAlpha(const Eigen::Matrix<double,27,1>& q,
                                            const Eigen::Matrix<double,27,1>& qdot,
                                            const Eigen::Matrix<double,12,1>& u_kinematics,
                                            const Eigen::Matrix<double,27,1>& qddot);

    DynamicsIntegrator(double dt);


    void step(const Eigen::Matrix<double,27,1>& q,
              const Eigen::Matrix<double,27,1>& qdot,
              const Eigen::Matrix<double,12,1>& u_kinematics);

    std::array<double,2> computeRearWheelOmegas(double speed, double steeringAngle);
    std::array<double,2> computeRearWheelTorque(double Qr, double steeringAngleFront, double steeringAngleRear);
    std::array<double,2> computeFrontWheelTorque(double Qf, double steeringAngleFront, double steeringAngleRear);
    void getDtheta(const Eigen::Matrix<double,12,1>& nu, const Eigen::Matrix<double,12,1>& u_kinematics);

private:
        KinematicsSolver kinematics_solver_;
        // ===== 追加：4輪Ni連動補償の計算メソッド =====
        NiCompTorques4 computeCompensationTorques4W(
        const WheelState4& ws,   // 各輪の角速度
        double alpha,            // 斜面角（世界x軸に沿って上り）
        double psi,              // 登り方向に対する車体ヨーずれ
        const NiCompParams& P);  // パラメータ一式
};