#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHother_15_()
{
double ret;
ret = 2*l1*Sin(PAI/6. + q_map(2))*Power(qdot_map(2),2) + 2*Cos(q_map(9))*qdot_map(10)*qdot_map(9) - q_map(10)*Sin(q_map(9))*Power(qdot_map(9),2);
return ret;
}
