#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_4_3_()
{
double ret;
ret = -((k5 + k6)*calc_pd_Z4_pd_X_1_3_()) - (k5*k6*calc_pd_Z4_pd_X_2_3_())/a0;
return ret;
}
