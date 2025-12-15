#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_ud_pd_X_3_13_()
{
double ret;
ret = -(((calc_alpha_3_2_()*(-(w1*calc_alpha_2_1_()) + w2) + calc_alpha_2_2_()*(w1*calc_alpha_3_1_() - w3))*(-(calc_alpha_3_3_()*calc_pd_alpha2_pd_X_2_13_()) + calc_alpha_3_2_()*calc_pd_alpha2_pd_X_3_13_() + calc_alpha_2_3_()*calc_pd_alpha3_pd_X_2_13_() - calc_alpha_2_2_()*calc_pd_alpha3_pd_X_3_13_()))/Power(calc_alpha_2_3_()*calc_alpha_3_2_() - calc_alpha_2_2_()*calc_alpha_3_3_(),2)) + ((w1*calc_alpha_3_1_() - w3)*calc_pd_alpha2_pd_X_2_13_() + (-(w1*calc_alpha_2_1_()) + w2)*calc_pd_alpha3_pd_X_2_13_() + calc_alpha_3_2_()*(-(w1*calc_pd_alpha2_pd_X_1_13_()) + calc_pd_W_pd_X_2_12_()) + calc_alpha_2_2_()*(w1*calc_pd_alpha3_pd_X_1_13_() - calc_pd_W_pd_X_3_12_()))/(calc_alpha_2_3_()*calc_alpha_3_2_() - calc_alpha_2_2_()*calc_alpha_3_3_());
return ret;
}
