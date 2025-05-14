#include "mpc_solver.hpp"

MPCSolver::MPCSolver(const MultibodyPhaseSpace &space_, int nsteps_, int nu_, VectorXd x0_, VectorXd u0_,
    std::vector<FrameIndex> contact_ids_,
    std::vector<FrameIndex> else_ids_,
    std::vector<SE3> arm_contact_place_,
    std::vector<std::vector<Vector3d>> &contact_poses_,
    std::vector<std::vector<bool>> &contact_states_,
    double mass_, Vector3d arm_force_, double z_ref_, double slope_)
    : space(space_),
    nsteps(nsteps_), 
    nu(nu_), x0(x0_), u0(u0_),
    contact_ids(contact_ids_),
    else_ids(else_ids_),
    arm_contact_places(arm_contact_place_),
    contact_poses(contact_poses_),     
    contact_states(contact_states_),
    mass(mass_), arm_force(arm_force_), z_ref(z_ref_), slope(slope_)
    {}

StageModel MPCSolver::createStage(int k)
{
    const Model &model = space.getModel();

    CostStack cost(space, nu);

    cost.addCost(QuadraticStateCost(space, nu, x0, mpc_settings.w_x));
    cost.addCost(QuadraticControlCost(space, u0, mpc_settings.w_u));

    
    for (size_t i = 0; i < 4; i++)
    {
        if (!contact_states[k][i]) // 只考虑摆动腿轨迹跟踪
        {
            FlyHighResidual fly_res(space.ndx(), model, contact_ids[i], slope, nu);
            cost.addCost(QuadraticResidualCost(space, fly_res, mpc_settings.w_fly));
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

    ZmpResidualCost zmp_residual(space, nu, contact_feet_id, mpc_settings.w_zmp);
    CostFiniteDifference zmp_fini_diff(zmp_residual, 1e-4);
    cost.addCost("zmp_residual_cost", zmp_fini_diff);

    FramePlacementResidual frame_res(space.ndx(), nu, model, arm_contact_places[k], contact_ids[4]);
    cost.addCost(QuadraticResidualCost(space, frame_res, mpc_settings.w_arm));

    KinodynamicsFwdDynamics ode(space, model, mpc_settings.gravity, contact_states[k], contact_ids, mpc_settings.force_size);
    IntegratorEuler dyn_model(ode, mpc_settings.dt);
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
            CentroidalFrictionConeResidual friction_residual(space.ndx(), nu, i, mpc_settings.mu, 1e-5);
            stage_model.addConstraint(friction_residual, NegativeOrthant()); 
            
            // 添加高度约束
            JointCoordinateResidual height_residual(space.ndx(), nu, model, contact_ids[i], 2);
            stage_model.addConstraint(height_residual, EqualityConstraint());

        }
    }


    return stage_model;
}


std::pair<std::vector<VectorXd>, std::vector<VectorXd>> MPCSolver::solve()
{
    const Model &model = space.getModel();

    ////////////////////////// 生成权重矩阵 //////////////////////////////   
    VectorXd wx_diag = VectorXd::Ones(space.ndx()) * 1e-6;
    wx_diag.head(3).setZero();
    wx_diag.segment(3, 3).setOnes();
    VectorXd w_foot_diag = VectorXd::Ones(3) * 100;
    //VectorXd w_joint_diff_diag = VectorXd::Ones(1) * 10;
    VectorXd w_arm_diag = VectorXd::Ones(6) * 1000;
    VectorXd w_cent_mom_diag = VectorXd::Ones(6) * 1e-3;
    VectorXd w_fly_diag = VectorXd::Ones(2) * 10;
    mpc_settings.w_x = wx_diag.asDiagonal();
    int w_temp1 = 500; // 高度
    int w_temp2 = 100; // 角度
    int w_temp3 = 100; // 机械臂关节
    int w_temp4 = 10; // 腿部关节
    mpc_settings.w_x(2, 2) = w_temp1;
    mpc_settings.w_x(3, 3) = w_temp2;
    mpc_settings.w_x(4, 4) = w_temp2;
    mpc_settings.w_x(5, 5) = w_temp2;
    mpc_settings.w_x(6, 6) = w_temp2;
    mpc_settings.w_x(19, 19) = w_temp3;
    mpc_settings.w_x(20, 20) = w_temp3;
    mpc_settings.w_x(21, 21) = w_temp3;
    mpc_settings.w_x(22, 22) = w_temp3;
    mpc_settings.w_x(23, 23) = w_temp3;
    mpc_settings.w_x(24, 24) = w_temp3;

    for (size_t i = 0; i < 12; i++)
    {
        mpc_settings.w_x(i+7,i+7) = w_temp4;
    }

    mpc_settings.w_u = 1e-3 * MatrixXd::Identity(nu, nu);
    mpc_settings.w_u(12, 12) = 1000;
    mpc_settings.w_u(13, 13) = 1000;
    mpc_settings.w_u(14, 14) = 1000;
    mpc_settings.w_foot = w_foot_diag.asDiagonal();
    //mpc_settings.w_joint_diff = w_joint_diff_diag.asDiagonal();
    mpc_settings.w_cent_mom = w_cent_mom_diag.asDiagonal();
    mpc_settings.w_arm = w_arm_diag.asDiagonal();
    mpc_settings.w_fly = w_fly_diag.asDiagonal();
    mpc_settings.w_zmp = 100;

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

    //double TOL = 1e-4;
    double TOL = 1e-3;
    //double mu_init = 1e-8;
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

