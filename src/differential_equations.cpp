#include "differential_equations.hpp"
#include "initial.hpp"    // u1…u12, l1…lv, ai の extern 宣言
#include <cmath>


double f0 (const std::vector<double>&) { 
    return 1.0; 
}

double f1 (const std::vector<double>& x) { 
    return u1 * std::cos(x[3]); 
}

double f2 (const std::vector<double>& x) { 
    return u1 * std::sin(x[3]); 
}

double f3 (const std::vector<double>&) { 
    return u2; 
}

double f4 (const std::vector<double>&) {  
    return u3; 
}

double f5 (const std::vector<double>& x) {
    return -(ai.a1n1/(l1*std::cos(x[4]))) * u1;
}

double f6 (const std::vector<double>&) { 
    return u4; 
}

double f7 (const std::vector<double>& x) {
    return -(ai.a2n/(l2*std::cos(x[6]))) * u1;
}

double f8 (const std::vector<double>&) { 
    return u5; 
}

double f9 (const std::vector<double>& x) {
    return -(ai.a3n/(l3*std::cos(x[8]))) * u1;
}

double f10(const std::vector<double>&) { 
    return u6; 
}

double f11(const std::vector<double>& x) {
    return ((ai.a4t*std::tan(x[10]) - ai.V3t*std::sin(x[8]+x[9]-x[11]))/lv)*u1;
}

double f12(const std::vector<double>&) { 
    return u7; 
}

double f13(const std::vector<double>& x) {
    return (ai.a5n/(l2*std::cos(x[12]))) * u1;
}

double f14(const std::vector<double>&) { 
    return u8; 
}

double f15(const std::vector<double>& x){
    return (ai.a6n/(l3*std::cos(x[14]))) * u1;
}

double f16(const std::vector<double>&) { 
    return u9; 
}

double f17(const std::vector<double>& x) {
    return ((-ai.a7t*std::tan(x[16]) + ai.V6t*std::sin(x[14]+x[15]-x[17]))/lv)*u1;
}

double f18(const std::vector<double>&) { 
    return u10; 
}

double f19(const std::vector<double>& x) {
    return (ai.a8n/(l2*std::cos(x[18]))) * u1;
}

double f20(const std::vector<double>&) { 
    return u11; 
}

double f21(const std::vector<double>& x) {
    return (ai.a9n/(l3*std::cos(x[20]))) * u1;
}

double f22(const std::vector<double>&) { 
    return u12; 
}

double f23(const std::vector<double>& x) {
    return ((-ai.a10t*std::tan(x[22]) + ai.V9t*std::sin(x[20]+x[21]-x[23]))/lv)*u1;
}

const std::array<FunctionPtr, DIM+1> fAll = {{
    f0, f1, f2, f3, f4, f5, f6, f7, f8, f9,
    f10,f11,f12,f13,f14,f15,f16,f17,f18,f19,
    f20,f21,f22,f23
}};
