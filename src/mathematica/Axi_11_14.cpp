#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_Axi_11_14_()
{
double ret;
ret = -(lv*Cos(q_map(21))*Cos(q_map(24) + q_map(21)))/2. - (lv*Sin(q_map(21))*Sin(q_map(24) + q_map(21)))/2.;
return ret;
}
