#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_11_()
{
double ret;
ret = (lv*Cos(q_map(24) + q_map(21))*Sin(q_map(21))*Power(qdot_map(21),2))/2. - (lv*Cos(q_map(21))*Sin(q_map(24) + q_map(21))*Power(qdot_map(21),2))/2. + Cos(q_map(24) + q_map(21))*(qdot_map(24) + qdot_map(21))*(-(lv*Sin(q_map(21))*qdot_map(21))/2. + qdot_map(19)) + Sin(q_map(24) + q_map(21))*(qdot_map(24) + qdot_map(21))*((lv*Cos(q_map(21))*qdot_map(21))/2. + qdot_map(20));
return ret;
}
