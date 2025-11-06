#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_pd_alpha5_pd_X_3_1_()
{
double ret;
ret = -((sr.d*Sec(x_old[8])*Sec(Thetap0)*(Cos(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2) - Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(-Sin(x_old[4] - Thetap0 + Thetap1) + 2*Power(Sec(x_old[4]),2)*Sin(x_old[4] - Thetap0 + Thetap1) + 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4]))) - Sin(x_old[6] - x_old[8] + Thetap2 - Thetap3)*(-(Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*Sin(x_old[4] - Thetap0 + Thetap1)) - Cos(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(-Sin(x_old[4] - Thetap0 + Thetap1) + 2*Power(Sec(x_old[4]),2)*Sin(x_old[4] - Thetap0 + Thetap1) + 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) - 2*(Cos(x_old[4] - Thetap0 + Thetap1)*Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2) - Sin(x_old[4] - Thetap0 + Thetap1)*Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2) + Sin(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(-Sin(x_old[4] - Thetap0 + Thetap1) + 2*Power(Sec(x_old[4]),2)*Sin(x_old[4] - Thetap0 + Thetap1) + 2*Cos(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])) + Cos(x_old[4] - x_old[6] + Thetap1 - Thetap2)*(Cos(x_old[4] - Thetap0 + Thetap1) + 2*Sin(x_old[4] - Thetap0 + Thetap1)*Tan(x_old[4])))*Tan(x_old[6])))*sr.Cs1)/l3);
return ret;
}
