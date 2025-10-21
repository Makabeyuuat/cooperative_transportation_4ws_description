#include "initial.hpp"

void initial(double &t, double &dt, std::vector<double> &x0, std::vector<double> &x_new, std::vector<double> &x_input) {
    t = 66.8;
    dt = 0.02;

    // 必要なサイズにリサイズ（DIM + 1 個）
    x0.resize(DIM + 1, 0.0);
    x_new.resize(DIM + 1, 0.0);
    x_input.resize(DIM + 1, 0.0);

    //第1車両の摺動機構
    x0[6] = PAI / 6.0;
    x0[7] = -PAI / 6.0;
    x0[8] = -PAI / 6.0;
    x0[9] = PAI / 6.0;
    x0[11] = PAI;

    // 第2車両の摺動機構
    x0[12] = -5.0 * PAI / 6.0;
    x0[13] = 5.0 * PAI / 6.0 ;
    x0[14] = -7.0 * PAI / 6.0;
    x0[15] = 7.0 * PAI / 6.0 ;
    x0[16] =0.0;
    x0[17] = 0.0;

    // 第3車両の摺動機構
    x0[18] = -7.0 * PAI / 6.0;
    x0[19] = 7.0 * PAI / 6.0;
    x0[20] = -5.0 * PAI / 6.0;
    x0[21] = 5.0 * PAI / 6.0;
    x0[22] = 0.0;
    x0[23] =0.0;


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
}
