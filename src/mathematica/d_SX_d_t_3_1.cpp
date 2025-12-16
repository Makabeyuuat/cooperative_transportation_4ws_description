#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_3_1_()
{
double ret;
ret = (sr.Cs*Sin(Thetap0)*(-((sr.Cs*Cos(Thetap0)*u_act(0))/(1 - sr.Cs*sr.d)) + u_act(1)))/(1 - sr.Cs*sr.d) - (Power(Cos(Thetap0),2)*u_act(0)*sr.Cs1)/Power(1 - sr.Cs*sr.d,2) + (sr.Cs*Cos(Thetap0)*(-(sr.Cs*Sin(Thetap0)*u_act(0)) - (Cos(Thetap0)*sr.d*u_act(0)*sr.Cs1)/(1 - sr.Cs*sr.d)))/Power(1 - sr.Cs*sr.d,2);
return ret;
}
