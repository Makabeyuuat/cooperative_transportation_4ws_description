#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z7_pd_X_1_1_()
{
double ret;
ret = -(sr.d*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[12])*(-2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 - 2*Cos(PAI/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2)*sr.Cs1) + (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0)*sr.d*sr.Cs1)/Power(1 - sr.Cs*sr.d,2)) - (Cos(Thetap0)*sr.Cs1)/(1 - sr.Cs*sr.d));
return ret;
}
