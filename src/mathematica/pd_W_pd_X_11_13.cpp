#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_11_13_()
{
double ret;
ret = -((k19 + k20)*calc_pd_Z11_pd_X_1_12_()) - (k19*k20*calc_pd_Z11_pd_X_2_13_())/a0;
return ret;
}
