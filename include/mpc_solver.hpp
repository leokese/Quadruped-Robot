#pragma once

#include "common/types.hpp"

struct MPCSettings
{
    MatrixXd w_x;        // 状态权重
    MatrixXd w_u;        // 输入权重
    MatrixXd w_foot;     // 腿的平移权重
    MatrixXd w_cent_mom; // 角动量导数权重
    MatrixXd w_arm;      // 机械臂末端位姿权重

    double dt = 20e-3;                        // Timestep
    double mu = 0.8;                          // Friction coefficient
    Vector3d gravity = Vector3d(0, 0, -9.81); // Gravity
    int force_size = 3;                       // 接触力维度
};

class MPCSolver
{
private:
    MPCSettings mpc_settings;
    const MultibodyPhaseSpace &space;
    std::vector<SE3> arm_contact_places;
    std::vector<std::vector<Vector3d>>& contact_poses;
    std::vector<std::vector<bool>>& contact_states;
    std::vector<FrameIndex> contact_ids;
    VectorXd x0;
    VectorXd u0;
    int nu;
    int nsteps;

    StageModel createStage(int k);

public:
    MPCSolver(const MultibodyPhaseSpace &space_, int nsteps_, int nu_, VectorXd x0_, VectorXd u0_,
        std::vector<FrameIndex> contact_ids_,
        std::vector<SE3> arm_contact_place_,
        std::vector<std::vector<Vector3d>> &contact_poses_,
        std::vector<std::vector<bool>> &contact_states_);
    
    std::pair<std::vector<VectorXd>, std::vector<VectorXd>> solve();

};