#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_5_1_()
{
double ret;
ret = (sr.Cs*Sin(Thetap0)*(-((sr.Cs*Cos(Thetap0)*u_act(0))/(1 - sr.Cs*sr.d)) + u_act(1)))/(1 - sr.Cs*sr.d) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])*u_act(2))/l1 + (Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*((sr.Cs*Cos(Thetap0)*u_act(0))/(1 - sr.Cs*sr.d) + (-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*u_act(0) - u_act(1) + u_act(2)))/l1 - (Power(Cos(Thetap0),2)*u_act(0)*sr.Cs1)/Power(1 - sr.Cs*sr.d,2) + (sr.Cs*Cos(Thetap0)*(-(sr.Cs*Sin(Thetap0)*u_act(0)) - (Cos(Thetap0)*sr.d*u_act(0)*sr.Cs1)/(1 - sr.Cs*sr.d)))/Power(1 - sr.Cs*sr.d,2);
return ret;
}
