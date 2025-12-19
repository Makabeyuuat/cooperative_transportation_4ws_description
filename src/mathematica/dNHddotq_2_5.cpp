#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_dNHddotq_2_5_()
{
double ret;
ret = wheelRadius*Sin(q_map(15) + q_map(5))*qdot_map(16);
return ret;
}
