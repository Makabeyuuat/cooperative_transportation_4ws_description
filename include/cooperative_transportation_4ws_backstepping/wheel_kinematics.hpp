// #pragma once
// #include <cmath>

// namespace wheelkin {

// struct AxleOmegas {
//     double left;   // 左ホイール角速度 [rad/s]
//     double right;  // 右ホイール角速度 [rad/s]
// };

// struct WheelCmd {
//     // ステア角（左右同値で保持）
//     double delta_fl, delta_fr, delta_rl, delta_rr;
//     // ホイール角速度 [rad/s]
//     double omega_fl, omega_fr, omega_rl, omega_rr;
// };

// /**
//  * @brief 既知の「車軸中点の車輪向き速度」から左右ホイール角速度を計算
//  * @param v_axle_along  車軸中点の速度（タイヤの向きに沿った成分）[m/s]
//  * @param steer         その車軸の操舵角 [rad]（左折+）
//  * @param lt            トレッド幅 [m]
//  * @param wheelR        タイヤ半径 [m]
//  * @param lv            前後輪間距離（ホイールベース）[m]
//  */
// static inline AxleOmegas computeAxleOmegasFromAxleSpeed(double v_axle_along,
//                                                         double steer,
//                                                         double lt,
//                                                         double wheelR,
//                                                         double lv)
// {
//     AxleOmegas out{};
//     // ほぼ直進：左右同一
//     if (std::fabs(steer) < 1e-8) {
//         double w = v_axle_along / wheelR;
//         out.left = w; out.right = w;
//         return out;
//     }

//     // 軸中点〜ICC距離
//     double R = lv / std::tan(steer);        // steer>0 で左折
//     // 内外輪の距離（R±lt/2）→ 速度は距離に比例
//     double vin  = v_axle_along * (R - 0.5*lt) / R;
//     double vout = v_axle_along * (R + 0.5*lt) / R;

//     double win  = vin  / wheelR;
//     double wout = vout / wheelR;

//     if (steer > 0.0) {
//         // 左折: 左が内輪
//         out.left  = win;
//         out.right = wout;
//     } else {
//         // 右折: 右が内輪
//         out.left  = wout;
//         out.right = win;
//     }
//     return out;
// }

// /**
//  * @brief 4WS 1台ぶん（前/後軸それぞれの“車輪向き速度”が既知）の左右角速度をまとめて計算
//  * @param v_front_along 前軸中点の車輪向き速度 [m/s]
//  * @param v_rear_along  後軸中点の車輪向き速度 [m/s]
//  * @param phi_f         前軸操舵角 [rad]
//  * @param phi_r         後軸操舵角 [rad]
//  * @param lv,lt,wheelR  幾何パラメータ
//  */
// static inline WheelCmd compute4ws_from_along(double v_front_along,
//                                              double v_rear_along,
//                                              double phi_f,
//                                              double phi_r,
//                                              double lv,
//                                              double lt,
//                                              double wheelR)
// {
//     WheelCmd cmd{};
//     cmd.delta_fl = phi_f;  cmd.delta_fr = phi_f;
//     cmd.delta_rl = phi_r;  cmd.delta_rr = phi_r;

//     AxleOmegas f = computeAxleOmegasFromAxleSpeed(v_front_along, phi_f, lt, wheelR, lv);
//     AxleOmegas r = computeAxleOmegasFromAxleSpeed(v_rear_along,  phi_r, lt, wheelR, lv);

//     cmd.omega_fl = f.left;   cmd.omega_fr = f.right;
//     cmd.omega_rl = r.left;   cmd.omega_rr = r.right;
//     return cmd;
// }

// } // namespace wheelkin
#pragma once
#include <cmath>
#include <limits>

namespace wheelkin {

struct AxleOmegas {
    double left;   // 左ホイール角速度 [rad/s]
    double right;  // 右ホイール角速度 [rad/s]
};

struct WheelCmd {
    // ステア角（左右同値で保持）
    double delta_fl, delta_fr, delta_rl, delta_rr;
    // ホイール角速度 [rad/s]
    double omega_fl, omega_fr, omega_rl, omega_rr;
};

/**
 * @note 互換維持のため残置。各軸を独立ICCで扱う旧式の近似です。
 *       4WSで単一ICRを厳密に満たすには下の compute4ws_from_along を使用してください。
 */
static inline AxleOmegas computeAxleOmegasFromAxleSpeed(double v_axle_along,
                                                        double steer,
                                                        double lt,
                                                        double wheelR,
                                                        double lv)
{
    AxleOmegas out{};
    if (std::fabs(steer) < 1e-8) {
        double w = v_axle_along / wheelR;
        out.left = w; out.right = w;
        return out;
    }
    // 旧：各軸ごとにICCを仮定する近似
    double R = lv / std::tan(steer);        // steer>0 で左折
    double vin  = v_axle_along * (R - 0.5*lt) / R;
    double vout = v_axle_along * (R + 0.5*lt) / R;
    double win  = vin  / wheelR;
    double wout = vout / wheelR;
    if (steer > 0.0) { out.left = win;  out.right = wout; }
    else             { out.left = wout; out.right = win;  }
    return out;
}

/**
 * @brief 4WS（単一ICCを共有）の幾何に基づき、前/後軸の“車輪向き速度”から各ホイール角速度を算出
 * @param v_front_along 前軸中点の車輪向き速度 [m/s]
 * @param v_rear_along  後軸中点の車輪向き速度 [m/s]
 * @param phi_f         前軸操舵角 [rad]（左折+）
 * @param phi_r         後軸操舵角 [rad]（左折+）
 * @param lv            ホイールベース L [m]
 * @param lt            トレッド幅 t [m]
 * @param wheelR        タイヤ半径 r_w [m]
 */
static inline WheelCmd compute4ws_from_along(double v_front_along,
                                             double v_rear_along,
                                             double phi_f,
                                             double phi_r,
                                             double lv,
                                             double lt,
                                             double wheelR)
{
    WheelCmd cmd{};
    cmd.delta_fl = phi_f;  cmd.delta_fr = phi_f;
    cmd.delta_rl = phi_r;  cmd.delta_rr = phi_r;

    const double eps = 1e-9;
    const double tf = std::tan(phi_f);
    const double tr = std::tan(phi_r);

    // 単一曲率（ICR共有条件）
    const double kappa = (tf - tr) / lv;

    // クラブ走行（平行移動/直進）近傍：内外差は立たない
    if (std::fabs(kappa) < eps) {
        const double v_mean = 0.5 * (v_front_along + v_rear_along);
        const double w = v_mean / wheelR;
        cmd.omega_fl = cmd.omega_fr = cmd.omega_rl = cmd.omega_rr = w;
        return cmd;
    }

    // ICC 半径（左折を R>0 とする）
    const double R = 1.0 / kappa;

    // 軸中点の回転半径 r_f = sqrt(R^2 + (L/2)^2)
    const double r_mid = std::sqrt(R*R + (lv*0.5)*(lv*0.5));

    // 2つの along 速度から整合する車体角速度を推定（平均。最小二乗と同値）
    const double omega_body = 0.5 * (v_front_along + v_rear_along) / r_mid;

    // ICC=(0, R) と置いた車体座標で、各ホイールのICC距離 rho_i を計算
    auto wheelOmega = [&](double x, double y) {
        const double dx = x;
        const double dy = y - R;                      // ICC までの相対
        const double rho = std::sqrt(dx*dx + dy*dy);  // ICC距離
        return (omega_body * rho) / wheelR;          // ホイール角速度
    };

    // ホイール位置（車体座標系）：前(+L/2), 後(-L/2), 左(+t/2), 右(-t/2)
    cmd.omega_fl = wheelOmega(+lv*0.5, +lt*0.5);
    cmd.omega_fr = wheelOmega(+lv*0.5, -lt*0.5);
    cmd.omega_rl = wheelOmega(-lv*0.5, +lt*0.5);
    cmd.omega_rr = wheelOmega(-lv*0.5, -lt*0.5);

    return cmd;
}

} // namespace wheelkin
