#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha2_pd_X_2_3_()
{
double ret;
ret = 2*(1 - sr.Cs*sr.d)*Power(Sec(Thetap0),2)*Tan(Thetap0);
return ret;
}
