#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha3_pd_X_1_2_()
{
double ret;
ret = -((Power(sr.Cs,2)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*Sec(Thetap0))/l1) - (2*sr.Cs*Cos(x_old[4] - Thetap0 + Thetap1)*(1 - sr.Cs*sr.d)*Sec(x_old[4])*Power(Sec(Thetap0),2)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))/l1 - sr.Cs*(-(Power(sr.Cs,2)/(1 - sr.Cs*sr.d)) - sr.Cs*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))*Tan(Thetap0) - sr.Cs*(-(sr.Cs*Sec(Thetap0)*(-((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1) + (sr.Cs*Sin(Thetap0))/(1 - sr.Cs*sr.d))) - sr.Cs*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*Tan(Thetap0)) + (Power(sr.Cs,2)*sr.d*sr.Cs1)/Power(1 - sr.Cs*sr.d,2) - Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*sr.Cs1 + (1 - sr.Cs*sr.d)*Sec(Thetap0)*((-2*Power(sr.Cs,2)*Cos(Thetap0)*sr.d*sr.Cs1)/Power(1 - sr.Cs*sr.d,3) - (2*sr.Cs*Cos(Thetap0)*sr.Cs1)/Power(1 - sr.Cs*sr.d,2)) - sr.Cs*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0)*sr.d*sr.Cs1)/Power(1 - sr.Cs*sr.d,2)) - (Cos(Thetap0)*sr.Cs1)/(1 - sr.Cs*sr.d));
return ret;
}
