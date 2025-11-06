#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha2_pd_X_2_2_()
{
double ret;
ret = -(sr.Cs*Power(Sec(Thetap0),2));
return ret;
}
