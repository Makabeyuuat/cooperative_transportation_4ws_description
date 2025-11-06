#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_ud_pd_t_1_()
{
double ret;
ret = -((w1*(calc_SX_23_1_()*u_act(0)*calc_pd_g11_pd_X_23_() + u_act(11)*calc_pd_g11_pd_X_22_() + calc_SX_21_1_()*u_act(0)*calc_pd_g11_pd_X_21_() + u_act(10)*calc_pd_g11_pd_X_20_() + calc_SX_19_1_()*u_act(0)*calc_pd_g11_pd_X_19_() + u_act(9)*calc_pd_g11_pd_X_18_() + calc_SX_17_1_()*u_act(0)*calc_pd_g11_pd_X_17_() + u_act(8)*calc_pd_g11_pd_X_16_() + calc_SX_15_1_()*u_act(0)*calc_pd_g11_pd_X_15_() + u_act(7)*calc_pd_g11_pd_X_14_() + calc_SX_13_1_()*u_act(0)*calc_pd_g11_pd_X_13_() + u_act(6)*calc_pd_g11_pd_X_12_() + calc_SX_11_1_()*u_act(0)*calc_pd_g11_pd_X_11_() + u_act(5)*calc_pd_g11_pd_X_10_() + calc_SX_9_1_()*u_act(0)*calc_pd_g11_pd_X_9_() + u_act(4)*calc_pd_g11_pd_X_8_() + calc_SX_7_1_()*u_act(0)*calc_pd_g11_pd_X_7_() + u_act(3)*calc_pd_g11_pd_X_6_() + calc_SX_5_1_()*u_act(0)*calc_pd_g11_pd_X_5_() + u_act(2)*calc_pd_g11_pd_X_4_() + (calc_SX_3_1_()*u_act(0)+u_act(1))*calc_pd_g11_pd_X_3_() + calc_SX_2_1_()*u_act(0)*calc_pd_g11_pd_X_2_() + calc_SX_1_1_()*u_act(0)*calc_pd_g11_pd_X_1_()))/Power(calc_SX_1_1_(),2));
return ret;
}
