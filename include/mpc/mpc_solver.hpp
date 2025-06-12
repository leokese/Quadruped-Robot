#pragma once

#include "common/types.hpp"
#include "mpc/zmp_residual.hpp"
#include "mpc/mpc_settings.hpp"

using Motion = pinocchio::MotionTpl<double>;

class MPCSolver
{
private:
    MPCSettings mpc_settings_;
    const MultibodyPhaseSpace &space_;
    std::vector<std::vector<Vector3d>> foot_contact_poses_;
    std::vector<SE3> arm_contact_places_;
    std::vector<std::vector<bool>> contact_states_;
    std::vector<FrameIndex> contact_id_;
    std::vector<VectorXd> x_ref_;
    std::vector<VectorXd> u_ref_;
    int nu_;
    int nsteps_;
    std::unique_ptr<TrajOptProblem> problem_;

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
              const std::vector<SE3> &arm_contact_places,
              const std::vector<std::vector<bool>> &contact_states,
              const MPCSettings &mpc_settings);

    std::pair<std::vector<VectorXd>, std::vector<VectorXd>> solve(const VectorXd &x0,
                                                                  const std::vector<VectorXd> &x_init,
                                                                  const std::vector<VectorXd> &u_init,
                                                                  size_t max_iters);
};