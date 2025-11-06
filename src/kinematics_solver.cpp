#include "cooperative_transportation_4ws_backstepping/kinematics_solver.hpp"

// ---- 2D テーブル → 行列化（メンバ関数ポインタ 2D 配列） --------------------
template <size_t R, size_t C>
static Eigen::Matrix<double, R, C>
fill_from_tbl(KinematicsSolver* self, double (KinematicsSolver::*const (&tbl)[R][C])())
{
    Eigen::Matrix<double, R, C> M;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            M(r, c) = (self->*tbl[r][c])();
    return M;
}

// ---- 1D テーブル → ベクトル化（std::array 版） ------------------------------
template <size_t N>
static Eigen::Matrix<double, N, 1>
fill_from_vec(KinematicsSolver* self, double (KinematicsSolver::*const (&tbl)[N])())
{
    Eigen::Matrix<double, N, 1> v;
    for (size_t i = 0; i < N; ++i) v(i) = (self->*tbl[i])();
    return v;
}

// ---- 行列/ベクトル 実装 ------------------------------------------------------
Eigen::Matrix<double,27,27> KinematicsSolver::Cxi_mat(){
    return fill_from_tbl<27,27>(this, Cxi_tbl);
}

Eigen::Matrix<double,27,27> KinematicsSolver::Mxi_mat(){
    return fill_from_tbl<27,27>(this, Mxi_tbl);
}

Eigen::Matrix<double,18,27> KinematicsSolver::Axi_mat(){ 
    return fill_from_tbl<18,27>(this, Axi_tbl);
}

Eigen::Matrix<double,27,1> KinematicsSolver::Kxi_vec(){
    return fill_from_vec<27>(this, Kxi_tbl);
}


Eigen::Matrix<double,12,1> KinematicsSolver::pd_ud_vec(){
    return fill_from_vec<12>(this, pdud_tbl);
}

Eigen::Matrix<double,27,1> KinematicsSolver::aqd_vec(){
    return fill_from_vec<27>(this, aqd_tbl);
}


Eigen::Matrix<double,23,12> KinematicsSolver::SX_mat(){
    return fill_from_tbl<23,12>(this, SX_tbl);
}

Eigen::Matrix<double,23,12> KinematicsSolver::dSXdt_mat(){
    return fill_from_tbl<23,12>(this, dSXdt_tbl);
}

