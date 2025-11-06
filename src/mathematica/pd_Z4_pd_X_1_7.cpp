#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z4_pd_X_1_7_()
{
double ret;
ret = ((1 - sr.Cs*sr.d)*Sec(x_old[6])*Sec(Thetap0)*(-(Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))))/l2;
return ret;
}
