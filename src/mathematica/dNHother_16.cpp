#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_16_()
{
double ret;
ret = -2*lv*Cos(PAI/6. + q_map(2))*Power(qdot_map(2),2) - 2*Sin(q_map(13))*qdot_map(18)*qdot_map(13) - Cos(q_map(13))*q_map(18)*Power(qdot_map(13),2);
return ret;
}
