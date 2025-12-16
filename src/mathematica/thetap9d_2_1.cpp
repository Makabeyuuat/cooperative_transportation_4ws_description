#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_thetap9d_2_1_()
{
double ret;
ret = (-2*Power(l3,2)*lv*Ddelta3d)/(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2)) - (2*delta3d*Sqrt(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2) - Power(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2),2))*Ddelta3d)/(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2)) + ((-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2))*(8*Power(l3,2)*delta3d*Ddelta3d - 4*delta3d*(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2))*Ddelta3d))/(2.*(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2))*Sqrt(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta3d,2) - Power(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta3d,2),2))) + dthetap10d;
return ret;
}
