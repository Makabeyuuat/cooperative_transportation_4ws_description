#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha3_pd_X_1_5_()
{
double ret;
ret = (Power(Cos(x_old[4] - Thetap0 + Thetap1),2)*Power(1 - sr.Cs*sr.d,2)*Power(Sec(x_old[4]),2)*Power(Sec(Thetap0),2))/Power(l1,2) - (Power(1 - sr.Cs*sr.d,2)*Sec(x_old[4])*Power(Sec(Thetap0),2)*Sin(x_old[4] - Thetap0 + Thetap1)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1))/l1 - (sr.Cs*Cos(x_old[4] - Thetap0 + Thetap1)*(1 - sr.Cs*sr.d)*Sec(x_old[4])*Sec(Thetap0)*Tan(Thetap0))/l1 - sr.Cs*(((1 - sr.Cs*sr.d)*Sec(x_old[4])*Sec(Thetap0)*Sin(x_old[4] - Thetap0 + Thetap1))/l1 + (Cos(x_old[4] - Thetap0 + Thetap1)*(1 - sr.Cs*sr.d)*Sec(x_old[4])*Sec(Thetap0)*Tan(Thetap0))/l1) - (Cos(x_old[4] - Thetap0 + Thetap1)*sr.d*Sec(x_old[4])*Sec(Thetap0)*sr.Cs1)/l1;
return ret;
}
