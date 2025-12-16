#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_thetap9d_1_1_()
{
double ret;
ret = (7*PAI)/6. - ArcTan(-2*l3*delta3d,l3*lv) + ArcTan(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2),Sqrt(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2) - Power(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2),2))) + thetap10d;
return ret;
}
