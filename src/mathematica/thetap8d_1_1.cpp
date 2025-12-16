#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_thetap8d_1_1_()
{
double ret;
ret = (5*PAI)/6. + ArcTan(4*Sqrt(1 - Power(lv + 2*l3*Sin(PAI/6. - Thetap9 + thetap10d),2)/(16.*Power(l2,2))),(lv + 2*l3*Sin(PAI/6. - Thetap9 + thetap10d))/l2) + thetap10d;
return ret;
}
