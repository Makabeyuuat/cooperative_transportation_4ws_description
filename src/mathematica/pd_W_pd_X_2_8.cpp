#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_2_8_()
{
double ret;
ret = -((k1 + k2)*calc_pd_Z2_pd_X_1_8_()) - (k1*k2*calc_pd_Z2_pd_X_2_8_())/a0;
return ret;
}
