#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_6_()
{
double ret;
ret = -(lv*Cos(q_map(13))*Cos(q_map(14) + q_map(13))*Power(qdot_map(13),2))/2. - (lv*Sin(q_map(13))*Sin(q_map(14) + q_map(13))*Power(qdot_map(13),2))/2. + Sin(q_map(14) + q_map(13))*(qdot_map(14) + qdot_map(13))*((lv*Sin(q_map(13))*qdot_map(13))/2. + qdot_map(11)) - Cos(q_map(14) + q_map(13))*(qdot_map(14) + qdot_map(13))*(-(lv*Cos(q_map(13))*qdot_map(13))/2. + qdot_map(12));
return ret;
}
