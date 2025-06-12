#include "mpc/mpc_solver.hpp"

MPCSolver::MPCSolver(const MultibodyPhaseSpace &space,
                     int nsteps,
                     int nu,
                     const VectorXd &x0,
                     const std::vector<VectorXd> &x_ref,
                     const std::vector<VectorXd> &u_ref,
                     std::vector<FrameIndex> contact_id,
                     const std::vector<std::vector<Vector3d>> &foot_contact_poses,
                     const std::vector<SE3> &arm_contact_places,
                     const std::vector<std::vector<bool>> &contact_states,
                     const MPCSettings &mpc_settings)
    : space_(space),
      nsteps_(nsteps),
      nu_(nu),
      x_ref_(x_ref),
      u_ref_(u_ref),
      contact_id_(contact_id),
      foot_contact_poses_(foot_contact_poses),
      arm_contact_places_(arm_contact_places),
      contact_states_(contact_states),
      mpc_settings_(mpc_settings)
{
    createProblem(x0, x_ref.back());
}

StageModel MPCSolver::createStage(int k)
{
    const Model &model = space_.getModel();

    CostStack cost(space_, nu_);

    cost.addCost(QuadraticStateCost(space_, nu_, x_ref_[k], mpc_settings_.w_x));
    cost.addCost(QuadraticControlCost(space_, u_ref_[k], mpc_settings_.w_u));

    for (size_t i = 0; i < 4; i++)
    {
        if (!contact_states_[k][i]) // 只考虑摆动腿轨迹跟踪
        {
            // FlyHighResidual fly_res(space_.ndx(), model, contact_id_[i], mpc_settings_.fly_high_slope, nu_);
            // cost.addCost(QuadraticResidualCost(space_, fly_res, mpc_settings_.w_fly_high));

            FrameTranslationResidual frame_res(space_.ndx(), nu_, model, foot_contact_poses_[k][i], contact_id_[i]);
            cost.addCost(QuadraticResidualCost(space_, frame_res, mpc_settings_.w_foot_pos));
        }
    }

    // 添加机械臂末端期望接触位姿跟踪目标
    FramePlacementResidual arm_frame_res(space_.ndx(), nu_, model, arm_contact_places_[k], contact_id_[4]);
    cost.addCost(QuadraticResidualCost(space_, arm_frame_res, mpc_settings_.w_arm));

    size_t num_true = std::count(contact_states_[k].begin(), contact_states_[k].end(), true);

    if (num_true == 2 + 1)  // 2个脚接触 + 机械臂末端接触
    {
        ZmpResidual zmp_residual(space_.ndx(), nu_, model, contact_id_, contact_states_[k], mpc_settings_.force_size);
        QuadraticResidualCost zmp_cost(space_, zmp_residual, mpc_settings_.w_zmp);
        cost.addCost(zmp_cost);
    }

    // ZmpResidual zmp_residual(space_.ndx(), nu_, model, contact_id_, contact_states_[k], mpc_settings_.force_size);
    // QuadraticResidualCost zmp_cost(space_, zmp_residual, mpc_settings_.w_zmp);
    // cost.addCost(zmp_cost);
   
    KinodynamicsFwdDynamics ode(space_, model, mpc_settings_.gravity, contact_states_[k], contact_id_, mpc_settings_.force_size);
    IntegratorEuler dyn_model(ode, mpc_settings_.timestep);
    StageModel stage_model(cost, dyn_model);

    for (size_t i = 0; i < 4; i++)
    {
        if (contact_states_[k][i])
        {
            // // 添加速度约束
            // Motion zero_velocity = Motion::Zero();
            // FrameVelocityResidual vel_residual(space_.ndx(), nu_, model, zero_velocity, contact_id_[i], pinocchio::LOCAL);
            // std::vector<int> linear_vel_id = {0, 1}; // 只考虑x,y速度，因为z速度与高度约束重叠了
            // FunctionSliceXpr vel_slice = FunctionSliceXpr(vel_residual, linear_vel_id);
            // stage_model.addConstraint(vel_slice, EqualityConstraint());

            // 添加接触力约束
            CentroidalFrictionConeResidual friction_residual(space_.ndx(), nu_, i, mpc_settings_.mu, 1e-5);
            stage_model.addConstraint(friction_residual, NegativeOrthant());

            // // 添加高度约束
            // FrameTranslationResidual foot_trans_res(space_.ndx(), nu_, model, Vector3d::Zero(), contact_id_[i]);
            // std::vector<int> height_id = {2};
            // FunctionSliceXpr height_res = FunctionSliceXpr(foot_trans_res, height_id);
            // stage_model.addConstraint(height_res, EqualityConstraint());

            // 添加落足点约束
            FrameTranslationResidual frame_res(space_.ndx(), nu_, model, foot_contact_poses_[k][i], contact_id_[i]);
            stage_model.addConstraint(frame_res, EqualityConstraint());

            // // 添加高度约束
            // FrameTranslationResidual frame_res(space_.ndx(), nu_, model, foot_contact_poses_[k][i], contact_id_[i]);
            // std::vector<int> height_id = {2};
            // FunctionSliceXpr height_res = FunctionSliceXpr(frame_res, height_id);
            // stage_model.addConstraint(frame_res, EqualityConstraint());
        }
    }

    return stage_model;
}

void MPCSolver::createProblem(const VectorXd &x0, const VectorXd &x_ref_term)
{
    CostStack term_cost(space_, nu_);
    term_cost.addCost(QuadraticStateCost(space_, nu_, x_ref_term, 10 * mpc_settings_.w_x));

    std::vector<xyz::polymorphic<StageModel>> stages;
    for (size_t i = 0; i < nsteps_; i++)
    {
        stages.push_back(createStage(i));
    }

    problem_ = std::make_unique<TrajOptProblem>(x0, stages, term_cost);
}

std::pair<std::vector<VectorXd>, std::vector<VectorXd>> MPCSolver::solve(const VectorXd &x0,
                                                                         const std::vector<VectorXd> &x_init,
                                                                         const std::vector<VectorXd> &u_init,
                                                                         size_t max_iters)
{

    double TOL = 1e-5;
    double mu_init = 1e-8;
    // double TOL = 1e-3;
    // double mu_init = 1e-5;

    SolverProxDDP solver(TOL, mu_init, max_iters, proxsuite::nlp::VERBOSE);
    solver.rollout_type_ = aligator::RolloutType::LINEAR;
    // solver.sa_strategy_ = aligator::StepAcceptanceStrategy::FILTER;
    solver.force_initial_condition_ = true;
    // solver.filter_.beta_ = 1e-5;
    solver.setNumThreads(4);
    solver.setup(*problem_);

    solver.run(*problem_, x_init, u_init);

    std::vector<VectorXd> xs = solver.results_.xs;
    std::vector<VectorXd> us = solver.results_.us;

    return {xs, us};
}
