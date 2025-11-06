#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_7_20_()
{
double ret;
ret = -((k11 + k12)*calc_pd_Z7_pd_X_1_20_()) - (k11*k12*calc_pd_Z7_pd_X_2_20_())/a0;
return ret;
}
