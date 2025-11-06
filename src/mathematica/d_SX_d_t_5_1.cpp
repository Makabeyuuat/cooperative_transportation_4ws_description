#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_d_SX_d_t_5_1_()
{
double ret;
ret = (sr.Cs*(u2 - (u1*sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d))*Sin(Thetap0))/(1 - sr.Cs*sr.d) + (Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*(-u2 + u3 + (u1*sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d) + u1*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)))/l1 + (u3*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1 - (u1*Power(Cos(Thetap0),2)*sr.Cs1)/Power(1 - sr.Cs*sr.d,2) + (sr.Cs*Cos(Thetap0)*(-(u1*sr.Cs*Sin(Thetap0)) - (u1*Cos(Thetap0)*sr.d*sr.Cs1)/(1 - sr.Cs*sr.d)))/Power(1 - sr.Cs*sr.d,2);
return ret;
}
