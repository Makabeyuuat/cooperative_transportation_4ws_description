#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_Axi_6_10_()
{
double ret;
ret = -(lv*Cos(q_map(14) + q_map(13))*Sin(q_map(13)))/2. + (lv*Cos(q_map(13))*Sin(q_map(14) + q_map(13)))/2.;
return ret;
}
