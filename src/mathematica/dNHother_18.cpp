#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_18_()
{
double ret;
ret = 2*lv*Cos(PAI/6. - q_map(2))*Power(qdot_map(2),2) - 2*Sin(q_map(21))*qdot_map(26)*qdot_map(21) - Cos(q_map(21))*q_map(26)*Power(qdot_map(21),2);
return ret;
}
