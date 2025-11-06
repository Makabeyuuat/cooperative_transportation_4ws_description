#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_aqd_1_()
{
double ret;
ret = Cos(Thetap0 + thetaT)*nu(0) - Sin(Thetap0 + thetaT)*u_act(0)*(u_act(1) + sr.Cs*calc_SX_1_1_()*u_act(0));
return ret;
}
