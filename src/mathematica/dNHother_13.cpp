#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_13_()
{
double ret;
ret = -2*lv*Cos(q_map(2))*Power(qdot_map(2),2) - 2*Cos(q_map(5))*qdot_map(10)*qdot_map(5) + q_map(10)*Sin(q_map(5))*Power(qdot_map(5),2);
return ret;
}
