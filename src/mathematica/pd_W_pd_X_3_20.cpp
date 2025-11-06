#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_3_20_()
{
double ret;
ret = -((k3 + k4)*calc_pd_Z3_pd_X_1_20_()) - (k3*k4*calc_pd_Z3_pd_X_2_20_())/a0;
return ret;
}
