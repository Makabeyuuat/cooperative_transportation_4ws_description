#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_1_1_()
{
double ret;
ret = -(((u2 - (u1*sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d))*Sin(Thetap0))/(1 - sr.Cs*sr.d)) - (Cos(Thetap0)*(-(u1*sr.Cs*Sin(Thetap0)) - (u1*Cos(Thetap0)*sr.d*sr.Cs1)/(1 - sr.Cs*sr.d)))/Power(1 - sr.Cs*sr.d,2);
return ret;
}
