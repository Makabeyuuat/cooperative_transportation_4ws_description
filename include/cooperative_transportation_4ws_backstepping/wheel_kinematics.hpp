#pragma once
#include <cmath>
#include <limits>

namespace wheelkin {

struct WheelCmd {
    double delta_fl, delta_fr, delta_rl, delta_rr;   // steer angles [rad]
    double omega_fl, omega_fr, omega_rl, omega_rr;   // wheel angular rates [rad/s]
};

// 安全な hypot
static inline double hypot2(double a, double b) { return std::sqrt(a*a + b*b); }

/**
 * @brief 4WS：前/後軸中点の「操舵方向速度」(v1f,v1r) から4輪角速度を計算
 *
 * 前提（入力定義に対応）：
 *  - 前軸中点速度ベクトル v_f は、前輪操舵方向 e(phi_f) に一致し、大きさが v_front_along
 *  - 後軸中点速度ベクトル v_r は、後輪操舵方向 e(phi_r) に一致し、大きさが v_rear_along
 *  - 車体座標：原点=車体中心、x前方、y左
 *  - 前軸中点 = (+L/2, 0)、後軸中点 = (-L/2, 0)
 *  - 左右輪の操舵角は各軸で同一（Ackermannの左右差は別途）
 */
static inline WheelCmd compute4ws_from_along(double v_front_along,
                                             double v_rear_along,
                                             double phi_f,
                                             double phi_r,
                                             double lv,       // L [m]
                                             double lt,       // t [m]
                                             double wheelR)   // r_w [m]
{
    WheelCmd cmd{};

    // steer angles are returned as-is (左右同値)
    cmd.delta_fl = phi_f;  cmd.delta_fr = phi_f;
    cmd.delta_rl = phi_r;  cmd.delta_rr = phi_r;

    // パラメータチェック
    if (!(std::isfinite(v_front_along) && std::isfinite(v_rear_along) &&
          std::isfinite(phi_f) && std::isfinite(phi_r) &&
          std::isfinite(lv) && std::isfinite(lt) && std::isfinite(wheelR)) ||
        wheelR <= 0.0 || lv <= 0.0)
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        cmd.omega_fl = cmd.omega_fr = cmd.omega_rl = cmd.omega_rr = nan;
        return cmd;
    }

    // 方向ベクトル
    const double cf = std::cos(phi_f), sf = std::sin(phi_f);
    const double cr = std::cos(phi_r), sr = std::sin(phi_r);

    // ここが重要：
    // 前/後軸中点の速度ベクトルを「方向も含めて」復元する（あなたの定義と一致）
    // v_f = v_front_along * [cos(phi_f), sin(phi_f)]
    // v_r = v_rear_along  * [cos(phi_r), sin(phi_r)]
    const double vfx = v_front_along * cf;
    const double vfy = v_front_along * sf;
    const double vrx = v_rear_along  * cr;
    const double vry = v_rear_along  * sr;

    // 車体ツイスト (v_x, v_y, omega) を復元
    // v_f = [v_x, v_y + omega*L/2]
    // v_r = [v_x, v_y - omega*L/2]
    //
    // v_x は理想的には vfx==vrx だが、数値誤差/入力不整合に備えて平均を採用
    const double vx = 0.5 * (vfx + vrx);

    // omega と v_y は y成分から一意
    const double omega_body = (vfy - vry) / lv;
    const double vy         = 0.5 * (vfy + vry);

    // 各輪中心位置（車体座標）
    const double xf = +0.5 * lv;
    const double xr = -0.5 * lv;
    const double yl = +0.5 * lt;
    const double yr = -0.5 * lt;

    // 剛体速度場：v(x,y) = [vx - omega*y, vy + omega*x]
    auto velAt = [&](double x, double y) {
        const double vix = vx - omega_body * y;
        const double viy = vy + omega_body * x;
        return std::pair<double,double>(vix, viy);
    };

    // 操舵方向へ射影してホイール角速度へ
    auto wheelOmegaFromVel = [&](double vix, double viy, double phi_axle) {
        const double c = std::cos(phi_axle);
        const double s = std::sin(phi_axle);
        const double v_along = c * vix + s * viy;  // wheel heading direction
        return v_along / wheelR;
    };

    // 前輪（操舵角 phi_f）
    {
        auto [vix, viy] = velAt(xf, yl);  // FL
        cmd.omega_fl = wheelOmegaFromVel(vix, viy, phi_f);
    }
    {
        auto [vix, viy] = velAt(xf, yr);  // FR
        cmd.omega_fr = wheelOmegaFromVel(vix, viy, phi_f);
    }

    // 後輪（操舵角 phi_r）
    {
        auto [vix, viy] = velAt(xr, yl);  // RL
        cmd.omega_rl = wheelOmegaFromVel(vix, viy, phi_r);
    }
    {
        auto [vix, viy] = velAt(xr, yr);  // RR
        cmd.omega_rr = wheelOmegaFromVel(vix, viy, phi_r);
    }

    return cmd;
}

} // namespace wheelkin
