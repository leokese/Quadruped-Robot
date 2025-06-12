#include "mpc/mpc.hpp"

MPC::MPC(Model model, VectorXd x0, Vector3d f_pull, 
    std::vector<SE3> arm_contact_places)
    :space_(model), x0_(x0), f_pull_(f_pull), arm_contact_places_(arm_contact_places)
{
    std::cout << "1" << std::endl;
    std::cout << "2" << std::endl;
    model_ = model; // 复制模型
    data_ = pinocchio::Data(model_);
    x0_init_ = x0; // 记录初始状态（腿关节作为后续参考）
    nq_ = space_.getModel().nq;
    nv_ = space_.getModel().nv;
    nc_ = mpc_settings_.nc;
    nu_ = nc_ * mpc_settings_.force_size + nv_ - 6; 

    initMPC(YamlLoader("/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/config/parameters.yaml"));

    double mass = pinocchio::computeTotalMass(model_);
    f_foot_ref_ << 0, 0, -mass * mpc_settings_.gravity[2] / 4.0;

    const FrameIndex FL_id = model_.getFrameId("FL_foot", pinocchio::BODY);
    const FrameIndex FR_id = model_.getFrameId("FR_foot", pinocchio::BODY);
    const FrameIndex HL_id = model_.getFrameId("RL_foot", pinocchio::BODY);
    const FrameIndex HR_id = model_.getFrameId("RR_foot", pinocchio::BODY);
    const FrameIndex body_id = model_.getFrameId("base_link", pinocchio::BODY);
    const FrameIndex arm_id = model_.getFrameId("link8", pinocchio::BODY);

    const FrameIndex FL_hip_id = model_.getFrameId("FL_thigh", pinocchio::BODY);
    const FrameIndex FR_hip_id = model_.getFrameId("FR_thigh", pinocchio::BODY);
    const FrameIndex HL_hip_id = model_.getFrameId("RL_thigh", pinocchio::BODY);      
    const FrameIndex HR_hip_id = model_.getFrameId("RR_thigh", pinocchio::BODY);

    const FrameIndex base_id_ = model_.getFrameId("base_link", pinocchio::BODY);

    contact_ids_ = {FL_id, FR_id, HL_id, HR_id, arm_id};   

    hip_ids_ = {FL_hip_id, FR_hip_id, HL_hip_id, HR_hip_id};

    Gait gait = Gait(1, mpc_settings_.n_qs, mpc_settings_.n_ds);
    feet_contact_states_ = gait.generateFootStates();

    // 定义contact_states_
    for (size_t i = 0; i < feet_contact_states_.size(); ++i)
    {
        std::array<bool, 5> combined{};
        // 拷贝四足部分
        for (int j = 0; j < 4; ++j)
        {
            combined[j] = feet_contact_states_[i][j];
        }
        // 加上机械臂部分
        combined[4] = true; // 机械臂末端始终接触

        // 转换为 std::vector<bool> 并插入
        std::vector<bool> combined_vec(combined.begin(), combined.end());
        contact_states_.push_back(combined_vec);
    }   
    
    // 清空并初始化足端位姿数组
    velocity_base_.setZero();
    init_pose_.setZero();
    next_pose_.setZero();
    twist_vect_.setZero();
    feet_contact_poses_.clear();
    feet_contact_poses_.resize(mpc_settings_.T, std::vector<Vector3d>(nc_ - 1, Vector3d::Zero()));

    updateContactFeetPoses();  //  更新落足的位置

    // 第一次求解
    setX_U_ref();  // 设置期望状态和输入
    setX_U_init_0(); // 设置第一次热启动的期望状态和输入

    // for (const auto &id : contact_ids_) {
    //     std::cout << id << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "feet_contact_poses_ size: " 
    //       << feet_contact_poses_.size() << " x " 
    //       << (feet_contact_poses_.empty() ? 0 : feet_contact_poses_[0].size()) 
    //       << std::endl;

    // for (size_t t = 0; t < feet_contact_poses_.size(); ++t)
    // {
    //     std::cout << "t = " << t << ": ";
    //     for (size_t j = 0; j < feet_contact_poses_[t].size(); ++j)
    //     {
    //         const auto &pos = feet_contact_poses_[t][j];
    //         std::cout << "(" << pos[0] << ", " << pos[1] << ", " << pos[2] << ") ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "arm_contact_places_ size: " << arm_contact_places_.size() << std::endl;
    // for (size_t i = 0; i < arm_contact_places_.size(); ++i)
    // {
    // const auto &pose = arm_contact_places_[i];
    // std::cout << "[" << i << "]: " << pose.translation().transpose() << std::endl;
    // }


    // for (size_t t = 0; t < contact_states_.size(); ++t) {
    //     std::cout << "contact_states_[" << t << "]: ";
    //     for (const auto &state : contact_states_[t]) {
    //         std::cout << state << " ";
    //     }
    //     std::cout << std::endl;
    // }
    
    
    mpc_solver_ = std::make_unique<MPCSolver>(
        space_, mpc_settings_.T, nu_, 
        x0_, x_ref_, u_ref_, contact_ids_, feet_contact_poses_,
        arm_contact_places_, contact_states_, mpc_settings_
    );
    
    std::cout << "5" << std::endl;

    auto result = mpc_solver_->solve(x0_, x_init_, u_init_, 500);
    xs_ = result.first;
    us_ = result.second;

    std::cout << "MPC initialized successfully." << std::endl;


}

void MPC::iterate(const ConstVectorRef &x, std::vector<SE3> arm_contact_places, double current_time)
{
    // 更新Pinocchio信息
    updatePinocchioInfo(x.head(nq_), x.tail(nv_));

    // 更新状态变量
    x0_ = x; 

    // 更新接触状态
    updateContactStates();

    // 更新接触位姿
    updateContactFeetPoses();
    arm_contact_places_ = arm_contact_places; 

    // 设置期望状态和输入
    setX_U_ref();
    
    // 设置热启动的期望状态和输入
    setX_U_init();

    // 求解MPC问题
    mpc_solver_ = std::make_unique<MPCSolver>(
        space_, mpc_settings_.T, nu_, 
        x0_, x_ref_, u_ref_, contact_ids_, feet_contact_poses_,
        arm_contact_places_, contact_states_, mpc_settings_
    );
    auto result = mpc_solver_->solve(x0_, x_init_, u_init_, 500);
    xs_ = result.first;
    us_ = result.second;
}

void MPC::updatePinocchioInfo(const VectorXd &q, const VectorXd &v)
{
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
}

void MPC::updateContactStates()
{
    // 更新接触状态
    updateContactFeetStates();
    
    // 更新机械臂末端接触状态
    contact_states_.back()[4] = true; // 只修改末端接触状态，不改变其他
}

void MPC::updateContactFeetStates()
{
    if (!contact_states_.empty())
    {
        // 将第一个元素移动到末尾，实现循环队列行为
        std::vector<bool> first = contact_states_.front();
        contact_states_.erase(contact_states_.begin());
        contact_states_.push_back(first);
    }
}

void MPC::updateContactFeetPoses() 
{
    const int T = mpc_settings_.T;
    const int num_legs = contact_ids_.size() - 1; // 不包括机械臂末端

    for (int leg = 0; leg < num_legs; ++leg)
    {
        bool contact0 = contact_states_[0][leg];

        if (!contact0) // 如果当前步态下该腿不接触地面 （注：第一次的步态一定是四足全接触，否则有问题）
        {
            for (int t = 1; t < T; ++t)
            {
                bool contact = contact_states_[t][leg];
        
                if (contact)
                {
                    // 第一次触地时，固定当前姿态为落足点
                    feet_contact_poses_[t][leg] = next_pose_; // 直接用已有的
                }
                else
                {
                    feet_contact_poses_[t][leg].setZero(); // 摆动腿的落足点设为零
                }
        
            }
        }
        else // 如果当前步态下该腿接触地面
        {
            // 计算当前落足点
            init_pose_ = data_.oMf[contact_ids_[leg]].translation();

            // 计算下一落足点
            Vector3d foot_pos_ref = data_.oMf[hip_ids_[leg]].translation();
            foot_pos_ref[2] =  0.0; // 保持落足点高度为0

            Vector3d base_pos_ref = data_.oMf[base_id_].translation();

            velocity_base_ = x0_.segment(nq_, 6);

            twist_vect_[0] = -(foot_pos_ref[1] - base_pos_ref[1]);
            twist_vect_[1] = foot_pos_ref[0] - base_pos_ref[0];

            next_pose_.head<2>() = foot_pos_ref.head<2>();
            double step_duration = (mpc_settings_.n_qs + mpc_settings_.n_ds) * mpc_settings_.timestep;
            next_pose_.head<2>() += (velocity_base_.head<2>() + velocity_base_[5] * twist_vect_) * step_duration;
            next_pose_[2] = foot_pos_ref[2];  // 保持落足点高度

            for (int t = 0; t < T; ++t)
            {
                if (t == 0)
                {
                    feet_contact_poses_[t][leg] = init_pose_; // 初始位置
                    continue;
                }

                bool contact = contact_states_[t][leg];
                bool prev_contact = contact_states_[t - 1][leg];
        
                if (contact && prev_contact)
                {
                    // 支撑期继续用当前 stance 位
                    feet_contact_poses_[t][leg] = feet_contact_poses_[t - 1][leg];
                }
                else if (contact && !prev_contact)
                {
                    // 摆动 -> 支撑：计算新的落足点
                    feet_contact_poses_[t][leg] = next_pose_;
                }
                else
                {
                    feet_contact_poses_[t][leg].setZero();
                }
        
            }
        }
        
    }
}


// void MPC::updateContactFeetPoses() // 逻辑有问题
// {
//     const int T = mpc_settings_.T;
//     const int num_legs = contact_ids_.size() - 1; // 不包括机械臂末端

//     for (int leg = 0; leg < num_legs; ++leg)
//     {
//         bool contact0 = contact_states_[0][leg];

//         if (!contact0)
//         {
//             // 如果起始时是摆动腿，不更新任何东西，保留原轨迹
//             continue;
//         }

//         // 起始是支撑腿：获取当前支撑位置
//         Vector3d stance_pos = data_.oMf[contact_ids_[leg]].translation();
//         Vector3d next_contact_pos = stance_pos;

//         for (int t = 0; t < T; ++t)
//         {
//             bool contact = contact_states_[t][leg];
//             if (t == 0)
//             {
//                 feet_contact_poses_[t][leg] = stance_pos;
//                 continue;
//             }

//             bool prev_contact = contact_states_[t - 1][leg];

//             if (contact && !prev_contact)
//             {
//                 // 摆动 -> 支撑：计算新的落足点 （修改）
//                 VectorXd q0 = x0_.head(nq_);
//                 VectorXd v0 = x0_.segment(nq_, nv_);

//                 Vector3d foot_pos_ref = data_.oMf[hip_ids_[leg]].translation();
//                 foot_pos_ref[2] =  0.0; // 保持落足点高度为0

//                 Vector3d base_pos_ref = data_.oMf[base_id_].translation();

//                 velocity_base_ = x0_.segment(nq_, 6);

//                 twist_vect_[0] = -(foot_pos_ref[1] - base_pos_ref[1]);
//                 twist_vect_[1] = foot_pos_ref[0] - base_pos_ref[0];

//                 next_pose_.head<2>() = foot_pos_ref.head<2>();
//                 double step_duration = (mpc_settings_.n_qs + mpc_settings_.n_ds) * mpc_settings_.timestep;
//                 next_pose_.head<2>() += (velocity_base_.head<2>() + velocity_base_[5] * twist_vect_) * step_duration;
//                 next_pose_[2] = foot_pos_ref[2];  // 保持落足点高度

//                 next_contact_pos = next_pose_;
//                 stance_pos = next_contact_pos;

//                 feet_contact_poses_[t][leg] = next_contact_pos;
//             }
//             else if (contact && prev_contact)
//             {
//                 // 支撑期继续用当前 stance 位
//                 feet_contact_poses_[t][leg] = stance_pos;
//             }
//             else
//             {
//                 // 摆动期不更新，保留原数据
//                 // 你也可以选择置零：feet_contact_poses_[t][leg].setZero();
//             }
//         }
//     }
// }

void MPC::setX_U_ref()
{
    int T = mpc_settings_.T;
    double dt = mpc_settings_.timestep;

    x_ref_.resize(T, VectorXd::Zero(nq_ + nv_));
    u_ref_.resize(T, VectorXd::Zero(nu_));

    // 将当前状态拆分为位姿和速度
    VectorXd q0 = x0_.head(nq_);
    VectorXd v0 = x0_.tail(nv_);

    // 使用当前的 base 速度作为参考速度
    VectorXd q_ref = q0;
    VectorXd v_ref = v0;
    for (int i = 0; i < T; ++i)
    {
        // 积分位姿（只更新 base 的部分）
        pinocchio::integrate(space_.getModel(), q_ref, v_ref * dt, q_ref);
        x_ref_[i].head(nq_) = q_ref;
        x_ref_[i].tail(nv_) = v_ref;
    }

    for (int i = 0; i < T; ++i)
    {
        x_ref_[i].segment(19,6). setZero(); // 机械臂关节角度正则化，而不跟踪
        x_ref_[i].segment(7,12) = x0_init_.segment(7,12); // 保持腿部关节为初始值

        x_ref_[i].tail(nv_).setZero();
    }

    for (int i = 0; i < T; ++i)
    {
        u_ref_[i].setZero(); 
        u_ref_[i].segment(4 * mpc_settings_.force_size, mpc_settings_.force_size) = f_pull_;
    }
}

void MPC::setX_U_init_0()
{
    int T = mpc_settings_.T;
    std::cout << "T: " << T << std::endl;

    x_init_.resize(T + 1, VectorXd::Zero(nq_ + nv_));
    u_init_.resize(T, VectorXd::Zero(nu_));

    for (int i = 0; i < T + 1; ++i) {
        x_init_[i] = x0_;
    }

    for (int i = 0; i < T; ++i) {
        for (int j = 0; j < 4; j++)
        {
            u_init_[i].segment(j * mpc_settings_.force_size, mpc_settings_.force_size) = f_foot_ref_;
        }
        u_init_[i].segment(4 * mpc_settings_.force_size, mpc_settings_.force_size) = f_pull_;
    }

}

void MPC::setX_U_init()
{
    int T = mpc_settings_.T;

    x_init_.resize(T + 1, VectorXd::Zero(nq_ + nv_));
    u_init_.resize(T, VectorXd::Zero(nu_));

    xs_.erase(xs_.begin());
    xs_[0] = x0_;
    xs_.push_back(xs_.back());

    us_.erase(us_.begin());
    us_.push_back(us_.back());

    x_init_ = xs_;
    u_init_ = us_;
}

void MPC::initMPC(const YamlLoader &yaml_loader)
{
    // State Cost
    Eigen::VectorXd w_x_diag(space_.ndx());
    std::cout << space_.ndx() << std::endl;
    w_x_diag << yaml_loader.w_x_body_pos,
        yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos, yaml_loader.w_x_leg_pos,
        yaml_loader.w_x_arm_pos,
        yaml_loader.w_x_body_vel,
        yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel, yaml_loader.w_x_leg_vel,
        yaml_loader.w_x_arm_vel;
    mpc_settings_.w_x = w_x_diag.asDiagonal();

    // Control Cost
    Eigen::VectorXd w_u_diag(nu_);
    std::cout << nu_ << std::endl;
    w_u_diag << yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force, yaml_loader.w_u_foot_force,
        yaml_loader.w_u_arm_force,
        yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc, yaml_loader.w_u_leg_acc,
        yaml_loader.w_u_arm_acc;
    mpc_settings_.w_u = w_u_diag.asDiagonal();
    

    // Fly High Cost
    mpc_settings_.w_fly_high = yaml_loader.w_fly_high.asDiagonal();
    mpc_settings_.fly_high_slope = yaml_loader.fly_high_slope;

    // ZMP Cost
    mpc_settings_.w_zmp = yaml_loader.w_zmp.asDiagonal();

    // Foot pos Cost
    mpc_settings_.w_foot_pos = yaml_loader.w_foot_pos.asDiagonal();

    // Arm EE Cost
    mpc_settings_.w_arm = yaml_loader.w_arm_pos.asDiagonal();
}