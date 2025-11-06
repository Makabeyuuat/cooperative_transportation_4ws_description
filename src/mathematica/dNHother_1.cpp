#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_1_()
{
double ret;
ret = -(lv*Cos(q_map(6) + q_map(5))*Sin(q_map(5))*Power(qdot_map(5),2))/2. + (lv*Cos(q_map(5))*Sin(q_map(6) + q_map(5))*Power(qdot_map(5),2))/2. + Cos(q_map(6) + q_map(5))*(qdot_map(6) + qdot_map(5))*((lv*Sin(q_map(5))*qdot_map(5))/2. + qdot_map(3)) + Sin(q_map(6) + q_map(5))*(qdot_map(6) + qdot_map(5))*(-(lv*Cos(q_map(5))*qdot_map(5))/2. + qdot_map(4));
return ret;
}
