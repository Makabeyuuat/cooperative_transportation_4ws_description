#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_g11_pd_X_3_()
{
double ret;
ret = -(Sin(Thetap0)/(1 - sr.Cs*sr.d));
return ret;
}
