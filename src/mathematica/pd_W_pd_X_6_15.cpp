#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_6_15_()
{
double ret;
ret = -((k10 + k9)*calc_pd_Z6_pd_X_1_15_()) - (k10*k9*calc_pd_Z6_pd_X_2_15_())/a0;
return ret;
}
