#include "common/types.hpp"

struct MPCSettings
{
    MatrixXd w_x;          // 状态权重
    MatrixXd w_u;          // 输入权重
    MatrixXd w_foot_pos;   // 腿的平移权重
    MatrixXd w_fly_high;   // fly_high目标权重
    double fly_high_slope; // fly_high斜率
    MatrixXd w_zmp;        // ZMP权重
    MatrixXd w_arm;        // 机械臂末端期望接触位姿跟踪权重

    double mu = 0.8;                          // Friction coefficient
    Vector3d gravity = Vector3d(0, 0, -9.81); // Gravity
    int force_size = 3;                       // 接触力维度

    int T = 90; // MPC steps
    double timestep = 0.01; // MPC timestep

    int n_qs = 5; // number of stance phases
    int n_ds = 40; // number of double support phases

    int nc = 5; // contact number

};