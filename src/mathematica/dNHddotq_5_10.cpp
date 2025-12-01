#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHddotq_5_10_()
{
double ret;
ret = (lv*Cos(q_map(9))*Cos(q_map(19) + q_map(9)))/2. + (lv*Sin(q_map(9))*Sin(q_map(19) + q_map(9)))/2.;
return ret;
}
