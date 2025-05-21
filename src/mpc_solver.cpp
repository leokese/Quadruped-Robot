#include "mpc_solver.hpp"

MPCSolver::MPCSolver(const MultibodyPhaseSpace &space,
                     int nsteps,
                     int nu,
                     const VectorXd &x0,
                     const std::vector<VectorXd> &x_ref,
                     const std::vector<VectorXd> &u_ref,
                     std::vector<FrameIndex> contact_id,
                     const std::vector<std::vector<Vector3d>> &foot_contact_poses,
                     const std::vector<std::vector<bool>> &contact_states,
                     const YamlLoader &yaml_loader)
    : space_(space),
      nsteps_(nsteps),
      nu_(nu),
      x_ref_(x_ref),
      u_ref_(u_ref),
      contact_id_(contact_id),
      foot_contact_poses_(foot_contact_poses),
      contact_states_(contact_states)
{
    initCostWeight(yaml_loader);
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
            FlyHighResidual fly_res(space_.ndx(), model, contact_id_[i], mpc_settings_.fly_high_slope, nu_);
            cost.addCost(QuadraticResidualCost(space_, fly_res, mpc_settings_.w_fly_high));

            // FrameTranslationResidual frame_res(space_.ndx(), nu_, model, foot_contact_poses_[k][i], contact_id_[i]);
            // cost.addCost(QuadraticResidualCost(space_, frame_res, mpc_settings_.w_foot_pos));
        }
    }

    // ZmpResidualCost zmp_residual(space_, nu_, contact_id_, contact_states_[k], mpc_settings_.force_size, mpc_settings_.w_zmp);
    // CostFiniteDifference zmp_fini_diff(zmp_residual, 1e-6);
    // cost.addCost("zmp_residual_cost", zmp_fini_diff);
    ZmpResidual zmp_residual(space_.ndx(), nu_, model, contact_id_, contact_states_[k], mpc_settings_.force_size);
    QuadraticResidualCost zmp_cost(space_, zmp_residual, mpc_settings_.w_zmp);
    cost.addCost(zmp_cost);

    KinodynamicsFwdDynamics ode(space_, model, mpc_settings_.gravity, contact_states_[k], contact_id_, mpc_settings_.force_size);
    IntegratorEuler dyn_model(ode, mpc_settings_.dt);
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
                                                                         const std::vector<VectorXd> &u_init)
{

    double TOL = 1e-5;
    double mu_init = 1e-8;
    size_t max_iters = 1000;

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

void MPCSolver::initCostWeight(const YamlLoader &yaml_loader)
{
    // State Cost
    Eigen::VectorXd w_x_diag(space_.ndx());
    w_x_diag << yaml_loader.w_x_body_pos,
        yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos,
        yaml_loader.w_x_body_vel,
        yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel;
    mpc_settings_.w_x = w_x_diag.asDiagonal();

    // Control Cost
    Eigen::VectorXd w_u_diag(nu_);
    w_u_diag << yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force,
        yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc;
    mpc_settings_.w_u = w_u_diag.asDiagonal();

    // Fly High Cost
    mpc_settings_.w_fly_high = yaml_loader.w_fly_high.asDiagonal();
    mpc_settings_.fly_high_slope = yaml_loader.fly_high_slope;

    // ZMP Cost
    mpc_settings_.w_zmp = yaml_loader.w_zmp.asDiagonal();

    // Foot pos Cost
    mpc_settings_.w_foot_pos = yaml_loader.w_foot_pos.asDiagonal();
}
