#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_3_21_()
{
double ret;
ret = -((k3 + k4)*calc_pd_Z3_pd_X_1_21_()) - (k3*k4*calc_pd_Z3_pd_X_2_21_())/a0;
return ret;
}
