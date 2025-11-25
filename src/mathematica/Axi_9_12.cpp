#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_Axi_9_12_()
{
double ret;
ret = Sin(q_map(23) + q_map(13));
return ret;
}
