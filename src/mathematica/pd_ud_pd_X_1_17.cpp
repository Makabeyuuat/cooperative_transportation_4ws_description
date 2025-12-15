#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_ud_pd_X_1_17_()
{
double ret;
ret = -((w1*calc_pd_g11_pd_X_17_())/Power(calc_SX_1_1_(),2));
return ret;
}
