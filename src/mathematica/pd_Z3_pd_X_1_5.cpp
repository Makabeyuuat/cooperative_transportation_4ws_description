#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z3_pd_X_1_5_()
{
double ret;
ret = (Cos(x_old[4] - Thetap0 + Thetap1)*(1 - sr.Cs*sr.d)*Sec(x_old[4])*Sec(Thetap0))/l1;
return ret;
}
