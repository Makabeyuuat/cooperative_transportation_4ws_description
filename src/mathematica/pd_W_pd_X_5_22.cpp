#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_W_pd_X_5_22_()
{
double ret;
ret = -((k7 + k8)*calc_pd_Z5_pd_X_1_22_()) - (k7*k8*calc_pd_Z5_pd_X_2_22_())/a0;
return ret;
}
