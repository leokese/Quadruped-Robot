#include "mpc_solver.hpp"

MPCSolver::MPCSolver(const MultibodyPhaseSpace &space_, int nsteps_, int nu_, VectorXd x0_, VectorXd u0_,
    std::vector<FrameIndex> contact_ids_,
    std::vector<SE3> arm_contact_places_,
    std::vector<std::vector<Vector3d>> &contact_poses_,
    std::vector<std::vector<bool>> &contact_states_)
    : space(space_),
    nsteps(nsteps_), 
    nu(nu_), x0(x0_), u0(u0_),
    contact_ids(contact_ids_),
    arm_contact_places(arm_contact_places_),
    contact_poses(contact_poses_),     
    contact_states(contact_states_)
    {}

StageModel MPCSolver::createStage(int k)
{
    const Model &model = space.getModel();

    CostStack cost(space, nu);
    // CentroidalMomentumDerivativeResidual cent_mom_deriv(space.ndx(), model, mpc_settings.gravity,
    //                                                     contact_states[k], contact_ids, mpc_settings.force_size);

    cost.addCost(QuadraticStateCost(space, nu, x0, mpc_settings.w_x));
    cost.addCost(QuadraticControlCost(space, u0, mpc_settings.w_u));
    // cost.addCost(QuadraticResidualCost(space, cent_mom_deriv, mpc_settings.w_cent_mom)); // 不重要

    for (size_t i = 0; i < 4; i++)
    {
        if (!contact_states[k][i]) // 只考虑摆动腿轨迹跟踪
        {
            FrameTranslationResidual frame_res(space.ndx(), nu, model, contact_poses[k][i], contact_ids[i]);
            cost.addCost(QuadraticResidualCost(space, frame_res, mpc_settings.w_foot));
        }
    }

    FramePlacementResidual frame_res(space.ndx(), nu, model, arm_contact_places[k], contact_ids[4]);
    cost.addCost(QuadraticResidualCost(space, frame_res, mpc_settings.w_arm));

    KinodynamicsFwdDynamics ode(space, model, mpc_settings.gravity, contact_states[k], contact_ids, mpc_settings.force_size);
    IntegratorEuler dyn_model(ode, mpc_settings.dt);
    StageModel stage_model(cost, dyn_model);

    for (size_t i = 0; i < 4; i++)
    {
        if (contact_states[k][i])
        {
            CentroidalFrictionConeResidual friction_residual(space.ndx(), nu, i, mpc_settings.mu, 1e-5);
            stage_model.addConstraint(friction_residual, NegativeOrthant());
            FrameTranslationResidual frame_res(space.ndx(), nu, model, contact_poses[k][i], contact_ids[i]);
            stage_model.addConstraint(frame_res, EqualityConstraint());
        }
    }
    return stage_model;
}


std::pair<std::vector<VectorXd>, std::vector<VectorXd>> MPCSolver::solve()
{
    const Model &model = space.getModel();

    ////////////////////////// 生成权重矩阵 //////////////////////////////   
    VectorXd wx_diag = VectorXd::Ones(space.ndx()) * 1e-2;
    wx_diag.head(3).setZero();
    wx_diag.segment(3, 3).setOnes();
    VectorXd w_foot_diag = VectorXd::Ones(3) * 100;
    VectorXd w_arm_diag = VectorXd::Ones(6) * 10;
    VectorXd w_cent_mom_diag = VectorXd::Ones(6) * 1e-3;
    mpc_settings.w_x = wx_diag.asDiagonal();
    mpc_settings.w_u = 1e-5 * MatrixXd::Identity(nu, nu);
    mpc_settings.w_u(12, 12) = 100;
    mpc_settings.w_u(13, 13) = 100;
    mpc_settings.w_u(14, 14) = 100;
    mpc_settings.w_foot = w_foot_diag.asDiagonal();
    mpc_settings.w_cent_mom = w_cent_mom_diag.asDiagonal();
    mpc_settings.w_arm = w_arm_diag.asDiagonal();

    CostStack term_cost(space, nu);
    std::cout << "0" << std::endl;
    //term_cost.addCost(QuadraticStateCost(space, nu, x0, 10 * mpc_settings.w_x));
    term_cost.addCost(QuadraticStateCost(space, nu, x0, 0 * mpc_settings.w_x));

    std::vector<xyz::polymorphic<StageModel>> stages;
    for (size_t i = 0; i < nsteps; i++)
    {
        stages.push_back(createStage(i));
    }

    TrajOptProblem problem(x0, stages, term_cost);

    double TOL = 1e-4;
    double mu_init = 1e-8;
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

