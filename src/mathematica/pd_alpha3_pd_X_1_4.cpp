#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha3_pd_X_1_4_()
{
double ret;
ret = -((Power(1 - sr.Cs*sr.d,2)*Sec(x_old[4])*Power(Sec(Thetap0),2)*Sin(x_old[4] - Thetap0 + Thetap1)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))/l1) + (Cos(x_old[4] - Thetap0 + Thetap1)*Power(1 - sr.Cs*sr.d,2)*Sec(x_old[4])*Power(Sec(Thetap0),2)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*Tan(x_old[4]))/l1 + (Cos(x_old[4] - Thetap0 + Thetap1)*Power(1 - sr.Cs*sr.d,2)*Sec(x_old[4])*Power(Sec(Thetap0),2)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1))/l1 - sr.Cs*(1 - sr.Cs*sr.d)*Sec(Thetap0)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1)*Tan(Thetap0) - sr.Cs*((1 - sr.Cs*sr.d)*Sec(Thetap0)*((Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1 - (Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4])*Tan(x_old[4]))/l1) + (1 - sr.Cs*sr.d)*Sec(Thetap0)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1)*Tan(Thetap0)) - sr.d*Sec(Thetap0)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1)*sr.Cs1;
return ret;
}
