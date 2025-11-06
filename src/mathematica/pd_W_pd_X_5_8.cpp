#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_5_8_()
{
double ret;
ret = -((k7 + k8)*calc_pd_Z5_pd_X_1_8_()) - (k7*k8*calc_pd_Z5_pd_X_2_8_())/a0;
return ret;
}
