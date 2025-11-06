#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_2_21_()
{
double ret;
ret = -((k1 + k2)*calc_pd_Z2_pd_X_1_21_()) - (k1*k2*calc_pd_Z2_pd_X_2_21_())/a0;
return ret;
}
