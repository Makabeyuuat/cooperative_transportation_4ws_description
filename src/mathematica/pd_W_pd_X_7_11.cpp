#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_7_11_()
{
double ret;
ret = -((k11 + k12)*calc_pd_Z7_pd_X_1_11_()) - (k11*k12*calc_pd_Z7_pd_X_2_11_())/a0;
return ret;
}
