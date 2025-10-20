#ifndef INITIAL_HPP
#define INITIAL_HPP
#include <cmath>  // sqrt関数を使うために必要
#include <vector>


#define DIM 23
#define Q_sample 100001  //曲線の分割数
#define PSdist 400
#define PAI 3.14159265358979323846
#define LIM 1.48352986419518  //操舵角の上限（85度）
inline constexpr int BEZIER_ORDER = 3; 
inline constexpr double RAD2DEG = 180.0 / PAI;
inline constexpr double DEG2RAD = PAI / 180.0;

 constexpr int sign(double A){ 
        if (A >= 0) return 1;
        else return -1;
    }


inline double wrapPi(double a){
  while(a >  PAI) a -= 2*PAI;
  while(a <= -PAI) a += 2*PAI;
  return a;
}
inline double clampAbs(double x, double lim){ return std::max(-lim, std::min(lim, x)); }

//係数
typedef struct {
	double a0t;
	double a1n1;
	double a1t1;
	double a1n2;
	double a1t2;
	double a1n3;
	double a1t3;
	double a2n;
	double a2t;
	double a3n;
	double a3t;
	double V3n;
	double V3t;
	double a4n;
	double a4t;
	double a5n;
	double a5t;
	double a6n;
	double a6t;
	double V6n;
	double V6t;
	double a7n;
	double a7t;
	double a8n;
	double a8t;
	double a9n;
	double a9t;
	double V9n;
	double V9t;
	double a10n;
	double a10t;
} Coefficient;

//探索用の構造体
typedef struct {
	double d;
	double Cs;
	double Cs1;
	double Cs2;
	int j;
	double Psx;
	double Psy;
	double thp;

}Search;




inline Coefficient ai;
inline Search sr;


//経路情報
//３次のベジェ曲線
//細長い経路
inline double Bx[BEZIER_ORDER + 1] = { -6.5, 2.0, -2.0, 6.5 };
inline double By[BEZIER_ORDER + 1] = { 2.0, 2.0, -2.0, -2.0 };
//幅広い経路
// inline double Bx[BEZIER_ORDER + 1] = { -7.0, 7.0, -7.0, 7.0 };
// inline double By[BEZIER_ORDER + 1] = { 7.0, 7.0, -7.0, -7.0 };

//曲率の配列を保存
inline double cs[Q_sample][4] = {};
inline double R[Q_sample][2] = {};
inline double dRdq[Q_sample][2] = {};
inline double d2Rdq2[Q_sample][2] = {};
inline double d3Rdq3[Q_sample][2] = {};
inline double d4Rdq4[Q_sample][2] = {};
inline double qs[Q_sample] = {};
inline double s[Q_sample];
inline double t_max;

//callback時に入れる変数
inline double true_carrier_pos[2] = {0, 0};
inline double true_carrier_yaw = 0.0;
inline double true_vehicle_yaw[3] = {0, 0, 0};
inline double true_vehicle1_steering_yaw[2] = {0};
inline double true_vehicle2_steering_yaw[2] = {0};
inline double true_vehicle3_steering_yaw[2] = {0};
//各車両の後輪の相対操舵角
inline double relative_steering_rear[3] = {0, 0, 0}; 
inline double delta_pos[3] = {0.0, 0.0, 0.0};
//コールバックのフラグ変数
inline bool got_body_pos = false;
inline std::vector<double> x_old = std::vector<double>(DIM+ 1, 0.0);
inline std::vector<double> x_new = std::vector<double>(DIM + 1, 0.0);
inline std::vector<double> x_input = std::vector<double>(DIM + 1, 0.0);

//input
//制御入力
inline double  w1;
inline double a0 = 0.2;
inline double u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12;
//制御入力用行列
//行列計算
inline double u4_a[3][3];
inline double u4_inv_a[3][3];
inline double u7_a[3][3];
inline double u7_inv_a[3][3];
inline double u10_a[3][3];
inline double u10_inv_a[3][3];


//フィードバックゲイン(後輪ゲインは低めに設定)
inline double k1 = 10.0;
inline double k2 = 10.0;
inline double k3 = 10.0;
inline double k4 = 10.0;
inline double k5 = 20.0;
inline double k6 = 20.0;
inline double k7 = 20.0;
inline double k8 = 20.0;
inline double k9 = 20.0;
inline double k10 = 20.0;
inline double k11 = 20.0;
inline double k12 = 20.0;
inline double k13 = 20.0;
inline double k14 = 20.0;
inline double k15 = 8.0;
inline double k16 = 8.0;
inline double k17 = 20.0;
inline double k18 = 20.0;
inline double k19 = 20.0;
inline double k20 = 20.0;
inline double k21 = 8.0;
inline double k22 = 8.0;




inline double b4 = 0.0;
inline double b5 = 0.0;
inline double b6 = 0.0;
inline double b7 = 0.0;
inline double b8 = 0.0;
inline double b9 = 0.0;
inline double b10 = 0.0;
inline double b11 = 0.0;
inline double b12 = 0.0;


//制御入力の係数
inline double z21, z22, z31, z32, z41, z42, z51, z52 ,z61, z62, z71, z72, z81, z82, z91, z92, z101, z102, z111, z112, z121, z122;
inline double alpha21, alpha22;
inline double alpha31, alpha32, alpha33;
inline double alpha41,alpha42, alpha43, alpha44;
inline double alpha51, alpha52, alpha53, alpha54, alpha55;
inline double alpha61, alpha62, alpha63, alpha64, alpha65, alpha66;
inline double alpha71, alpha72, alpha73, alpha76, alpha77;
inline double alpha81, alpha82, alpha83, alpha87, alpha88;
inline double alpha91, alpha92, alpha93, alpha97, alpha98, alpha99;
inline double alpha101, alpha102, alpha103, alpha109, alpha1010;
inline double alpha111, alpha112, alpha113, alpha1110, alpha1111;
inline double alpha121, alpha122, alpha123, alpha1210, alpha1211, alpha1212;



//重心の目標相対位置関数
inline double d0d, dd0d, ddd0d;
inline double thetap1d, dthetap1d, ddthetap1d;

inline double thetap2d;
inline double dthetap2d;
inline double K21, K22, K23, K24;

inline double thetap3d, dthetap3d;
inline double K31, K32, K33, K34;

inline double thetap4d = PAI;
inline double dthetap4d = 0;
inline double ddthetap4d = 0;

inline double thetap5d,dthetap5d;
inline double K51, K52, K53, K54;

inline double thetap6d , dthetap6d;
inline double K61, K62, K63, K64;

inline double thetap7d = -PAI / 8.0;
inline double dthetap7d = 0.0;
inline double ddthetap7d = 0.0;

inline double thetap8d, dthetap8d;
inline double K81, K82, K83, K84;

inline double thetap9d, dthetap9d;
inline double K91, K92, K93, K94;

inline double thetap10d = PAI / 6.0;
inline double dthetap10d = 0;
inline double ddthetap10d = 0;

//摺動機構の目標関数とその微分
inline double delta1d = 0.0, Ddelta1d = 0.0, DDdelta1d = 0.0;
inline double delta2d = 0.0, Ddelta2d = 0.0, DDdelta2d = 0.0;
inline double delta3d = 0.0, Ddelta3d = 0.0, DDdelta3d = 0.0;


inline double thetaT = 0.0;
inline double Thetap0, Thetap1, Thetap2, Thetap3, Thetap4, Thetap5, Thetap6, Thetap7, Thetap8, Thetap9, Thetap10;
inline double Phi[4] = {0.0};    // Phi[1]～Phi[3]（Phi[0] は使わない想定）

inline double x1 = 0.0, Y1 = 0.0;
inline double x2 = 0.0, y2 = 0.0;
inline double x3 = 0.0, y3 = 0.0;

inline double Trix[4];     // Trix[1]～Trix[3]
inline double Triy[4];     // Triy[1]～Triy[3]

//各ステアリングの座標
inline double dx1, dy1;
inline double dx2, dy2;
inline double dx3, dy3;
inline double dx4, dy4;
inline double dx5, dy5;
inline double dx6, dy6;
inline double dx7, dy7;
inline double dx8, dy8;
inline double dx9, dy9;
inline double dx10, dy10;

//非ホロ
inline double nh1 = 0.0;
inline double nh2 = 0.0;
inline double nh3 = 0.0;
inline double nh4 = 0.0;
inline double nh5 = 0.0;
inline double nh6 = 0.0;
inline double nh7 = 0.0;
inline double nh8 = 0.0;
inline double nh9 = 0.0;
inline double nh10 = 0.0;


//各リンクの長さ
inline double lv = 0.9;
inline double lt = 0.8;
inline double l1 = 0.8;
// inline double l2 = (sqrt(2) / 8) * lv;
// inline double l3 = (sqrt(2) / 4) * lv;

inline double l2 = (sqrt(3) / 12) * lv;
inline double l3 = (sqrt(3) / 6) * lv;
//摺動機構の仮想リンク
inline int VEHICLE1_SLIDING_MECHANISM_SIGN = sign(1.0);
inline int VEHICLE2_SLIDING_MECHANISM_SIGN = sign(1.0);
inline int VEHICLE3_SLIDING_MECHANISM_SIGN = sign(1.0);

//各車両の速度
inline double v1, v2, v3;
inline double v1f, v2f, v3f, v1r, v2r, v3r;
inline double wheelRadius = 0.153; //車輪の半径

// 初期値設定関数
// 引数: t, dt, x0, x_new
void initial(double &t, double &dt, std::vector<double> &x0, std::vector<double> &x_new, std::vector<double> &x_input);



#endif // INITIAL_HPP
