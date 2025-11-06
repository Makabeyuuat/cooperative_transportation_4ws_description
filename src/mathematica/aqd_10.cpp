#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_aqd_10_()
{
double ret;
ret = athetap7d + asd*sr.Cs + sr.Cs1*Power(calc_SX_1_1_()*u_act(0),2);
return ret;
}
