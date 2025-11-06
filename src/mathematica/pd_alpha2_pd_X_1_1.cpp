#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha2_pd_X_1_1_()
{
double ret;
ret = sr.Cs*sr.d*Power(Sec(Thetap0),2)*sr.Cs1 - (1 - sr.Cs*sr.d)*Power(Sec(Thetap0),2)*sr.Cs1 + sr.Cs*sr.d*Power(Tan(Thetap0),2)*sr.Cs1 - (1 - sr.Cs*sr.d)*Power(Tan(Thetap0),2)*sr.Cs1 - sr.d*Tan(Thetap0)*sr.Cs2;
return ret;
}
