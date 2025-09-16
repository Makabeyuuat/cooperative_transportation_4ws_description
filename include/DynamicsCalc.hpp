#ifndef DYNAMICS_CALCULATOR_HPP
#define DYNAMICS_CALCULATOR_HPP

#include "initial.hpp"
#include "differential_equations.hpp"

#include <vector>
#include <cmath>

class DynamicsCalculator {
public:

    // コンストラクタ（内部状態は保持しないので空）
    DynamicsCalculator();

    void calcXold(std::vector<double>& x_old);

    void computeCoefficients(const std::vector<double>& x_old);
  
    void calculate(const std::vector<double>& x_old, double t_max, double l1, double l2, double l3, int sr_j);

    double calc_thetavj_from_deltai(double deltai, int vehicle_sliding_mechanism_sign);
    double calc_thetavi_from_thetavj(double thetavj, int vehicle_sliding_mechanism_sign);
};

#endif // DYNAMICS_CALCULATOR_HPP
