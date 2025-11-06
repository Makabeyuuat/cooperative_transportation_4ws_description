#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z7_pd_X_1_3_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Sec(Thetap0)*((sr.Cs*Sin(Thetap0))/(1 - sr.Cs*sr.d) + (Sec(x_old[12])*(2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1) + 2*Cos(PAI/6. - Thetap0 + Thetap1)*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1) + Cos(x_old[12] - Thetap0 + Thetap5)*(1 - 2*Cos(PAI/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)) - 2*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[12] - Thetap0 + Thetap5) - (2*Cos(PAI/6. - Thetap0 + Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2) + (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[12])*(-2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 - 2*Cos(PAI/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[12] - Thetap0 + Thetap5)))/l2)*Tan(Thetap0);
return ret;
}
