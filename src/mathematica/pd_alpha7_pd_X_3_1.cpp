#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha7_pd_X_3_1_()
{
double ret;
ret = -((sr.d*Sec(x_old[12])*Sec(Thetap0)*(-2*Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1) - 2*Cos(x_old[12] - Thetap0 + Thetap5)*Sec(x_old[4])*Sin(PAI/6. - Thetap0 + Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]) - Sin(x_old[12] - Thetap0 + Thetap5)*(-2*Cos(PAI/6. - Thetap0 + Thetap1)*Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]) - 2*Cos(PAI/6. - Thetap0 + Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*sr.Cs1)/l2);
return ret;
}
