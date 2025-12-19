#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHddotq_2_6_()
{
double ret;
ret = wheelRadius*((lv*Cos(q_map(15) + q_map(5))*Sin(q_map(5)))/2. - (lv*Cos(q_map(5))*Sin(q_map(15) + q_map(5)))/2.)*qdot_map(16);
return ret;
}
