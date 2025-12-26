#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"		// 数学関数のヘッダーファイル
#include "cooperative_transportation_4ws_backstepping/Bezier.h"			// Bezier 曲線の関数



std::vector<double> x_old;
std::vector<double> x_new;
std::vector<double> x_input;


Eigen::Map<Eigen::Matrix<double,27,1>> q_map(q_twist);
Eigen::Map<Eigen::Matrix<double,27,1>> qdot_map(qdot_twist);


void initial(double &t, double &dt, std::vector<double> &x0, std::vector<double> &x_new, std::vector<double> &x_input) {
    t = 66.8;
    dt = 0.02;

    // 必要なサイズにリサイズ（DIM + 1 個）
    x_old.resize(DIM + 1, 0.0);
    x_new.resize(DIM + 1, 0.0);
    x_input.resize(DIM + 1, 0.0);

    //第1車両の摺動機構
    x0[6] = PAI / 6.0;
    x0[7] = -PAI / 6.0;
    x0[8] = -PAI / 6.0;
    x0[9] = PAI / 6.0;
    x0[11] = 0.0;

    // 第2車両の摺動機構
    x0[12] = -5.0 * PAI / 6.0;
    x0[13] = 5.0 * PAI / 6.0 ;
    x0[14] = -7.0 * PAI / 6.0;
    x0[15] = 7.0 * PAI / 6.0 ;
    x0[16] =  PAI / 8.0;
    x0[17] =  -PAI / 8.0;

    // 第3車両の摺動機構
    x0[18] = -7.0 * PAI / 6.0;
    x0[19] = 7.0 * PAI / 6.0;
    x0[20] = -5.0 * PAI / 6.0;
    x0[21] = 5.0 * PAI / 6.0;
    x0[22] = -PAI / 6.0;
    x0[23] = PAI / 6.0;


    // // 第1車両の摺動機構
    // x0[6] = PAI / 6.0;
    // x0[7] = -PAI / 6.0;
    // x0[8] = -PAI / 6.0;
    // x0[9] = PAI / 6.0;
    // x0[11] = PAI;

    // // 第2車両の摺動機構
    // x0[12] = -5.0 * PAI / 6.0 + PAI / 8.0;
    // x0[13] = 5.0 * PAI / 6.0 - PAI / 8.0;
    // x0[14] = -7.0 * PAI / 6.0 + PAI / 8.0;
    // x0[15] = 7.0 * PAI / 6.0 - PAI / 8.0;
    // x0[16] = PAI / 8.0;
    // x0[17] = -PAI / 8.0;

    // // 第3車両の摺動機構
    // x0[18] = -7.0 * PAI / 6.0 - PAI / 6.0;
    // x0[19] = 7.0 * PAI / 6.0 + PAI / 6.0;
    // x0[20] = -5.0 * PAI / 6.0 - PAI / 6.0;
    // x0[21] = 5.0 * PAI / 6.0 + PAI / 6.0;
    // x0[22] = -PAI / 6.0;
    // x0[23] = PAI / 6.0;




    //経路のQ分割
	qs[0] = 0.0;
	double dt_bezier = 1.0 / (Q_sample - 1);
	for (int i = 1; i < Q_sample; ++i) {
    	qs[i] = i * dt_bezier;
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
}
