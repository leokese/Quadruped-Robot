#include "mpc_solver.hpp"

MPCSolver::MPCSolver(const MultibodyPhaseSpace &space_, int nsteps_, int nu_, VectorXd x0_, VectorXd u0_,
                     std::vector<FrameIndex> contact_ids_,
                     std::vector<FrameIndex> else_ids_,
                     std::vector<SE3> arm_contact_place_,
                     std::vector<std::vector<Vector3d>> &contact_poses_,
                     std::vector<std::vector<bool>> &contact_states_,
                     double mass_, Vector3d arm_force_, double z_ref_, const YamlLoader &yaml_loader_)
    : space(space_),
      nsteps(nsteps_),
      nu(nu_), x0(x0_), u0(u0_),
      contact_ids(contact_ids_),
      else_ids(else_ids_),
      arm_contact_places(arm_contact_place_),
      contact_poses(contact_poses_),
      contact_states(contact_states_),
      mass(mass_), arm_force(arm_force_), z_ref(z_ref_)
{
    initCostWeight(yaml_loader_);
}

StageModel MPCSolver::createStage(int k)
{
    const Model &model = space.getModel();

    CostStack cost(space, nu);

    cost.addCost(QuadraticStateCost(space, nu, x0, mpc_settings_.w_x));
    cost.addCost(QuadraticControlCost(space, u0, mpc_settings_.w_u));

    for (size_t i = 0; i < 4; i++)
    {
        if (!contact_states[k][i]) // 只考虑摆动腿轨迹跟踪
        {
            FlyHighResidual fly_res(space.ndx(), model, contact_ids[i], mpc_settings_.fly_high_slope, nu);
            cost.addCost(QuadraticResidualCost(space, fly_res, mpc_settings_.w_fly_high));
        }
    }

    std::vector<int> contact_feet_id;
    for (size_t i = 0; i < 4; i++)
    {
        if (contact_states[k][i])
        {
            contact_feet_id.push_back(i);
        }
    }

    // todo: 改成只有当接触状态由摆动腿变为支撑腿时才出现
    ZmpResidualCost zmp_residual(space, nu, contact_feet_id, mpc_settings_.w_zmp);
    CostFiniteDifference zmp_fini_diff(zmp_residual, 1e-6);
    cost.addCost("zmp_residual_cost", zmp_fini_diff);

    FramePlacementResidual arm_ee_pos(space.ndx(), nu, model, arm_contact_places[k], contact_ids[4]);
    cost.addCost(QuadraticResidualCost(space, arm_ee_pos, mpc_settings_.w_arm_pos));

    KinodynamicsFwdDynamics ode(space, model, mpc_settings_.gravity, contact_states[k], contact_ids, mpc_settings_.force_size);
    IntegratorEuler dyn_model(ode, mpc_settings_.dt);
    StageModel stage_model(cost, dyn_model);

    for (size_t i = 0; i < 4; i++)
    {
        if (contact_states[k][i])
        {
            // 添加速度约束
            Motion zero_velocity = Motion::Zero();
            FrameVelocityResidual vel_residual(space.ndx(), nu, model, zero_velocity, contact_ids[i], pinocchio::WORLD);
            stage_model.addConstraint(vel_residual, EqualityConstraint());

            // 添加接触力约束
            CentroidalFrictionConeResidual friction_residual(space.ndx(), nu, i, mpc_settings_.mu, 1e-5);
            stage_model.addConstraint(friction_residual, NegativeOrthant());

            // 添加高度约束
            std::vector<int> height_id = {2};
            FrameTranslationResidual foot_trans_res(space.ndx(), nu, model, Vector3d::Zero(), contact_ids[i]);
            FunctionSliceXpr height_res = FunctionSliceXpr(foot_trans_res, height_id);
            stage_model.addConstraint(height_res, EqualityConstraint());
        }
    }

    return stage_model;
}

std::pair<std::vector<VectorXd>, std::vector<VectorXd>> MPCSolver::solve()
{
    const Model &model = space.getModel();

    CostStack term_cost(space, nu);
    // term_cost.addCost(QuadraticStateCost(space, nu, x0, 10 * mpc_settings.w_x));
    term_cost.addCost(QuadraticStateCost(space, nu, x0, 0 * mpc_settings_.w_x));

    std::vector<xyz::polymorphic<StageModel>> stages;
    for (size_t i = 0; i < nsteps; i++)
    {
        stages.push_back(createStage(i));
    }

    TrajOptProblem problem(x0, stages, term_cost);

    // double TOL = 1e-4;
    double TOL = 1e-3;
    // double mu_init = 1e-8;
    double mu_init = 1e-5;
    size_t max_iters = 100;

    SolverProxDDP solver(TOL, mu_init, max_iters, proxsuite::nlp::VERBOSE);
    solver.rollout_type_ = aligator::RolloutType::LINEAR;
    solver.sa_strategy_ = aligator::StepAcceptanceStrategy::FILTER;
    solver.force_initial_condition_ = true;
    solver.filter_.beta_ = 1e-5;
    solver.setNumThreads(4);
    solver.setup(problem);

    std::vector<VectorXd> xs_init(nsteps + 1, x0);
    std::vector<VectorXd> us_init(nsteps, u0);

    solver.run(problem, xs_init, us_init);

    std::vector<VectorXd> xs = solver.results_.xs;
    std::vector<VectorXd> us = solver.results_.us;

    return {xs, us};
}

void MPCSolver::initCostWeight(const YamlLoader &yaml_loader)
{
    // State Cost
    Eigen::VectorXd w_x_diag(space.ndx());
    w_x_diag << yaml_loader.w_x_body_pos,
        yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos,
        yaml_loader.w_x_arm_pos,
        yaml_loader.w_x_body_vel,
        yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel,
        yaml_loader.w_x_arm_vel;
    mpc_settings_.w_x = w_x_diag.asDiagonal();

    // Control Cost
    Eigen::VectorXd w_u_diag(nu);
    w_u_diag << yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force,
        yaml_loader.w_u_arm_force,
        yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc,
        yaml_loader.w_u_arm_acc;
    mpc_settings_.w_u = w_u_diag.asDiagonal();

    // Fly High Cost
    mpc_settings_.w_fly_high = yaml_loader.w_fly_high.asDiagonal();
    mpc_settings_.fly_high_slope = yaml_loader.fly_high_slope;

    // ZMP Cost
    mpc_settings_.w_zmp = yaml_loader.w_zmp;

    // Arm EE Cost
    mpc_settings_.w_arm_pos = yaml_loader.w_arm_pos.asDiagonal();
    mpc_settings_.w_arm_vel = yaml_loader.w_arm_vel.asDiagonal();
}
