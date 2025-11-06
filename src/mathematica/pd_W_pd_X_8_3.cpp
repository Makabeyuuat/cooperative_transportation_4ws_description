#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_8_3_()
{
double ret;
ret = -((k13 + k14)*calc_pd_Z8_pd_X_1_3_()) - (k13*k14*calc_pd_Z8_pd_X_2_3_())/a0;
return ret;
}
