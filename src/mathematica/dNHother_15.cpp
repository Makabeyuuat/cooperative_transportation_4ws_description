#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_15_()
{
double ret;
ret = 2*lv*Sin(PAI/6. + q_map(2))*Power(qdot_map(2),2) - 2*Cos(q_map(13))*qdot_map(18)*qdot_map(13) + q_map(18)*Sin(q_map(13))*Power(qdot_map(13),2);
return ret;
}
