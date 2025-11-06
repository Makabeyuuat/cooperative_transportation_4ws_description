#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_10_18_()
{
double ret;
ret = -((k17 + k18)*calc_pd_Z10_pd_X_1_18_()) - (k17*k18*calc_pd_Z10_pd_X_2_18_())/a0;
return ret;
}
