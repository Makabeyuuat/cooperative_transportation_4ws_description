#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_Z10_pd_X_1_18_()
{
double ret;
ret = (1 - sr.Cs*sr.d)*Sec(Thetap0)*((Sec(x_old[18])*(-(Cos(x_old[18] - Thetap0 + Thetap8)*(1 + 2*Cos(PAI/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))) + 2*Sec(x_old[4])*Sin(PAI/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[18] - Thetap0 + Thetap8)))/l2 + (Sec(x_old[18])*(-2*Cos(x_old[18] - Thetap0 + Thetap8)*Sec(x_old[4])*Sin(PAI/6. + Thetap0 - Thetap1)*Sin(x_old[4] - Thetap0 + Thetap1) - (1 + 2*Cos(PAI/6. + Thetap0 - Thetap1)*Sec(x_old[4])*Sin(x_old[4] - Thetap0 + Thetap1))*Sin(x_old[18] - Thetap0 + Thetap8))*Tan(x_old[18]))/l2);
return ret;
}
