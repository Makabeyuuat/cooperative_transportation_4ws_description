#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z3_pd_X_1_4_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Sec(Thetap0)*((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1 + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))/l1);
return ret;
}
