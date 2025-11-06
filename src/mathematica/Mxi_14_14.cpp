#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_Mxi_14_14_()
{
double ret;
ret = I_theta3 + (Power(lv,2)*m_w)/2.;
return ret;
}
