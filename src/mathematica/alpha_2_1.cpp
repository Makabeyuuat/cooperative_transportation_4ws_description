#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_alpha_2_1_()
{
double ret;
ret = -(sr.Cs*(1 - sr.Cs*sr.d)*Power(Sec(Thetap0),2)) - sr.Cs*(1 - sr.Cs*sr.d)*Power(Tan(Thetap0),2) - sr.d*Tan(Thetap0)*sr.Cs1;
return ret;
}
