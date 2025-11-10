#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_2_1_()
{
double ret;
ret = Cos(Thetap0)*(-((sr.Cs*Cos(Thetap0)*u_act(0))/(1 - sr.Cs*sr.d)) + u_act(1));
return ret;
}
