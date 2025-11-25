#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_7_()
{
double ret;
ret = (lv*Sin(q_map(21))*qdot_map(21)*qdot_map(5))/2. + (lv*Cos(q_map(21) + q_map(9))*Sin(q_map(9))*Power(qdot_map(9),2))/2. - (lv*Cos(q_map(9))*Sin(q_map(21) + q_map(9))*Power(qdot_map(9),2))/2. + Cos(q_map(21) + q_map(9))*(qdot_map(21) + qdot_map(9))*(-(lv*Sin(q_map(9))*qdot_map(9))/2. + qdot_map(7)) + Sin(q_map(21) + q_map(9))*(qdot_map(21) + qdot_map(9))*((lv*Cos(q_map(9))*qdot_map(9))/2. + qdot_map(8));
return ret;
}
