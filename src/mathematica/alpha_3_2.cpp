#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_alpha_3_2_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((Cos(x_old[4] - Thetap0 + Thetap1)*Sec(x_old[4]))/l1) + (sr.Cs*Sin(Thetap0))/(1 - sr.Cs*sr.d)) + (1 - sr.Cs*sr.d)*Sec(Thetap0)*(-((sr.Cs*Cos(Thetap0))/(1 - sr.Cs*sr.d)) + (Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))/l1)*Tan(Thetap0);
return ret;
}
