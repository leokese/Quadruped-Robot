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
    MatrixXd w_foot_pos;   // 腿的平移权重
    MatrixXd w_fly_high;   // fly_high目标权重
    double fly_high_slope; // fly_high斜率
    MatrixXd w_zmp;

    double dt = 20e-3;                        // Timestep
    double mu = 0.8;                          // Friction coefficient
    Vector3d gravity = Vector3d(0, 0, -9.81); // Gravity
    int force_size = 3;                       // 接触力维度
};

class MPCSolver
{
private:
    MPCSettings mpc_settings_;
    const MultibodyPhaseSpace &space_;
    std::vector<std::vector<Vector3d>> foot_contact_poses_;
    std::vector<std::vector<bool>> contact_states_;
    std::vector<FrameIndex> contact_id_;
    std::vector<VectorXd> x_ref_;
    std::vector<VectorXd> u_ref_;
    int nu_;
    int nsteps_;
    std::unique_ptr<TrajOptProblem> problem_;

    void initCostWeight(const YamlLoader &yaml_loader);
    StageModel createStage(int k);
    void createProblem(const VectorXd &x0, const VectorXd &x_ref_term);

public:
    MPCSolver(const MultibodyPhaseSpace &space,
              int nsteps,
              int nu,
              const VectorXd &x0,
              const std::vector<VectorXd> &x_ref,
              const std::vector<VectorXd> &u_ref,
              std::vector<FrameIndex> contact_id,
              const std::vector<std::vector<Vector3d>> &foot_contact_poses,
              const std::vector<std::vector<bool>> &contact_states,
              const YamlLoader &yaml_loader);

    std::pair<std::vector<VectorXd>, std::vector<VectorXd>> solve(const VectorXd &x0,
                                                                  const std::vector<VectorXd> &x_init,
                                                                  const std::vector<VectorXd> &u_init);
};