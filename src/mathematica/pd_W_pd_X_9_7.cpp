#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_9_7_()
{
double ret;
ret = -((k15 + k16)*calc_pd_Z9_pd_X_1_7_()) - (k15*k16*calc_pd_Z9_pd_X_2_7_())/a0;
return ret;
}
