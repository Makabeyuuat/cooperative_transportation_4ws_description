#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_9_4_()
{
double ret;
ret = -((k15 + k16)*calc_pd_Z9_pd_X_1_4_()) - (k15*k16*calc_pd_Z9_pd_X_2_4_())/a0;
return ret;
}
