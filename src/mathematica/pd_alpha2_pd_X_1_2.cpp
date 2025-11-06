#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha2_pd_X_1_2_()
{
double ret;
ret = Power(sr.Cs,2)*Power(Sec(Thetap0),2) + Power(sr.Cs,2)*Power(Tan(Thetap0),2) - Tan(Thetap0)*sr.Cs1;
return ret;
}
