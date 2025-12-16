#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include "cooperative_transportation_4ws_backstepping/mathFunc.h"
#include <array>
#include <iostream>

double KinematicsSolver::calc_thetap6d_2_1_()
{
double ret;
ret = (-2*Power(l3,2)*lv*Ddelta2d)/(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta2d,2)) - (2*delta2d*Sqrt(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta2d,2) - Power(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta2d,2),2))*Ddelta2d)/(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta2d,2)) + ((-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta2d,2))*(8*Power(l3,2)*delta2d*Ddelta2d - 4*delta2d*(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta2d,2))*Ddelta2d))/(2.*(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta2d,2))*Sqrt(Power(l3,2)*Power(lv,2) + 4*Power(l3,2)*Power(delta2d,2) - Power(-4*Power(l2,2) + Power(l3,2) + Power(lv,2)/4. + Power(delta2d,2),2))) + dthetap7d;
return ret;
}
