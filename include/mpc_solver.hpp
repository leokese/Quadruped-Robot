#pragma once

#include "common/types.hpp"
#include "joint_coordinate_residual.hpp"
#include "zmp_residual_cost.hpp"
#include "joint_coordinate_residual.hpp"
#include "yaml_loader.hpp"

using Motion = pinocchio::MotionTpl<double>;
using JointCoordinateResidual = aligator::JointCoordinateResidualTpl<double>;

struct MPCSettings
{
    MatrixXd w_x;          // 状态权重
    MatrixXd w_u;          // 输入权重
    MatrixXd w_foot;       // 腿的平移权重
    MatrixXd w_cent_mom;   // 角动量导数权重
    MatrixXd w_arm_pos;    // 机械臂末端位姿权重
    MatrixXd w_arm_vel;    // 机械臂末端速度权重
    MatrixXd w_fly_high;   // fly_high目标权重
    double fly_high_slope; // fly_high斜率
    double w_zmp;

    double dt = 20e-3;                        // Timestep
    double mu = 0.8;                          // Friction coefficient
    Vector3d gravity = Vector3d(0, 0, -9.81); // Gravity
    int force_size = 3;                       // 接触力维度
};

class MPCSolver
{
private:
    MPCSettings mpc_settings_;
    const MultibodyPhaseSpace &space;
    std::vector<SE3> arm_contact_places;
    std::vector<std::vector<Vector3d>> &contact_poses;
    std::vector<std::vector<bool>> &contact_states;
    std::vector<FrameIndex> contact_ids;
    VectorXd x0;
    VectorXd u0;
    int nu;
    int nsteps;
    std::unique_ptr<TrajOptProblem> problem_;

    void initCostWeight(const YamlLoader &yaml_loader);
    StageModel createStage(int k);
    void createProblem(const VectorXd &x0, const VectorXd &x_ref_term);

public:
    MPCSolver(const MultibodyPhaseSpace &space_, int nsteps_, int nu_, VectorXd x0_, VectorXd u0_,
              std::vector<FrameIndex> contact_ids_,
              std::vector<SE3> arm_contact_place_,
              std::vector<std::vector<Vector3d>> &contact_poses_,
              std::vector<std::vector<bool>> &contact_states_,
              const YamlLoader &yaml_loader_);

    std::pair<std::vector<VectorXd>, std::vector<VectorXd>> solve(const VectorXd &x0);
};