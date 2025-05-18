#include <cmath>
#include "gait_schedule.hpp"
#include "arm_schedule.hpp"
#include "mpc_solver.hpp"

void saveVectorsToCsv(const std::string &filename, const std::vector<Eigen::VectorXd> &vectors)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        throw "Unable to open file for writing";
    }

    for (const auto &vec : vectors)
    {
        for (int i = 0; i < vec.size(); ++i)
        {
            file << vec(i);
            if (i < vec.size() - 1)
                file << ",";
        }
        file << "\n";
    }
    file.close();
    std::cout << "Results saved to " << filename << std::endl;
}

std::vector<SE3> generateSE3Trajectory(const SE3 &init_pose, const SE3 &final_pose, int nsteps)
{
    std::vector<SE3> poses;
    for (int i = 0; i < nsteps; ++i)
    {
        double alpha = static_cast<double>(i) / (nsteps - 1); // 插值因子
        poses.push_back(pinocchio::SE3::Interpolate(init_pose, final_pose, alpha));
    }
    return poses;
}

int main(int argc, char const *argv[])
{
    ////////////////////////// 生成模型 //////////////////////////////
    std::string urdf_path = "/home/zishang/cpp_workspace/Quadruped-Robot/robot/galileo_mini_x5_description/galileo_mini_x5.urdf";
    std::string yaml_path = "/home/zishang/cpp_workspace/Quadruped-Robot/config/parameters.yaml";
    // std::string urdf_path = "/home/robot/文档/vs_project/quadruped_mpc_5/robot/galileo_mini/robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    Data data(model);
    MPCSettings mpc_settings;
    const int nq = model.nq;
    const int nv = model.nv;
    const int force_size = mpc_settings.force_size;
    const int nc = 5;                        // contact number
    const int nu = nv - 6 + nc * force_size; // input number
    MultibodyPhaseSpace space(model);
    const int ndx = space.ndx();
    YamlLoader yaml_loader(yaml_path);

    ////////////////////////// 生成初始状态 //////////////////////////////
    VectorXd q0(model.nq);
    VectorXd q_base_leg(19);
    VectorXd q_arm(6);
    // std::cout << "model.nq = " << model.nq << std::endl;
    // std::cout << "model.nv = " << model.nv << std::endl;
    // std::cout << "q_base_leg.size() = " << q_base_leg.size() << std::endl;
    // std::cout << "q_arm.size() = " << q_arm.size() << std::endl;

    q_base_leg << 0, 0, 0.38, 0, 0, 0, 1,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44;
    q_arm << 0, M_PI * 3 / 4, M_PI * 1 / 2, M_PI / 4, 0, 0;
    q0 << q_base_leg, q_arm;

    VectorXd x0(nq + nv);
    x0 << q0, VectorXd::Zero(nv);
    // VectorXd u0 = VectorXd::Zero(nu);
    VectorXd u0(nu);
    double mass = pinocchio::computeTotalMass(model);
    Vector3d f_ref(0, 0, -mass * mpc_settings.gravity[2] / 4.0);
    Vector3d f_pull(50, 0, 0);
    for (int i = 0; i < 4; ++i)
    {
        u0.segment(i * mpc_settings.force_size, mpc_settings.force_size) = f_ref;
    }
    u0.segment(4 * mpc_settings.force_size, 3) = f_pull;
    u0.segment(5 * mpc_settings.force_size, model.nv - 6).setZero();

    Vector3d com0 = pinocchio::centerOfMass(model, data, x0.head(nq));

    const FrameIndex FL_id = model.getFrameId("FL_foot_link", pinocchio::BODY);
    const FrameIndex FR_id = model.getFrameId("FR_foot_link", pinocchio::BODY);
    const FrameIndex HL_id = model.getFrameId("HL_foot_link", pinocchio::BODY);
    const FrameIndex HR_id = model.getFrameId("HR_foot_link", pinocchio::BODY);
    const FrameIndex arm_id = model.getFrameId("link8", pinocchio::BODY);
    const FrameIndex link1_id = model.getFrameId("FL_hip_link", pinocchio::BODY);
    const FrameIndex link2_id = model.getFrameId("FR_hip_link", pinocchio::BODY);
    const FrameIndex link3_id = model.getFrameId("HL_hip_link", pinocchio::BODY);
    const FrameIndex link4_id = model.getFrameId("HR_hip_link", pinocchio::BODY);
    const FrameIndex body_id = model.getFrameId("base_link", pinocchio::BODY);

    // std::cout << "FL_foot_link Frame Index: " << FL_id << std::endl;
    // std::cout << "FR_foot_link Frame Index: " << FR_id << std::endl;
    // std::cout << "HL_foot_link Frame Index: " << HL_id << std::endl;
    // std::cout << "HR_foot_link Frame Index: " << HR_id << std::endl;
    // std::cout << "link8 Frame Index: " << arm_id << std::endl;

    std::vector<FrameIndex> contact_ids = {FL_id, FR_id, HL_id, HR_id, arm_id};
    std::vector<FrameIndex> else_ids = {link1_id, link2_id, link3_id, link4_id};

    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateFramePlacements(model, data);

    SE3 FL_pose = data.oMf[FL_id];
    SE3 FR_pose = data.oMf[FR_id];
    SE3 HL_pose = data.oMf[HL_id];
    SE3 HR_pose = data.oMf[HR_id];

    std::vector<Vector3d> init_foot_pos = {FL_pose.translation(), FR_pose.translation(),
                                           HL_pose.translation(), HR_pose.translation()};

    SE3 init_arm_pose = data.oMf[arm_id];

    ////////////////////////// 生成腿部接触状态与位姿 //////////////////////////////
    std::vector<std::vector<Vector3d>> feet_contact_poses;
    std::vector<std::vector<bool>> feet_contact_states;

    const int n_qs = 5;  // 离散时刻的全接触支持数量
    const int n_ds = 40; // 离散时刻的双足接触支持数量
    const int steps = 2; // 生成多少组步态

    double swing_apex = 0.05; // 抬腿高度
    // double swing_apex = 0.0; // 抬腿高度
    double x_forward = -0.2; // 前进距离
    // double x_forward = 0.0;   // 前进距离

    // 最终生成 steps*(2*n_qs + 2*n_ds) 个离散时刻的足端接触状态与位姿
    Gait gait = Gait(steps, n_qs, n_ds, init_foot_pos, swing_apex, x_forward);
    int nsteps = gait.nsteps; // 离散时刻的数量
    feet_contact_states = gait.generateFootStates();
    feet_contact_poses = gait.generateFootTrajectory();

    ////////////////////////// 生成机械臂末端接触状态与位姿 //////////////////////////////
    std::vector<std::vector<bool>> arm_contact_states;
    std::vector<SE3> arm_contact_poses;
    SE3 final_arm_pose = SE3(init_arm_pose.rotation(),
                             init_arm_pose.translation() + Vector3d(-0.6, 0, 0));
    arm_contact_poses = generateSE3Trajectory(init_arm_pose, final_arm_pose, nsteps);

    for (size_t i = 0; i < feet_contact_states.size(); ++i)
    {
        arm_contact_states.push_back({true});
    }

    ////////////////////////// 生成总的接触状态 //////////////////////////////
    std::vector<std::vector<bool>> contact_states;
    for (size_t i = 0; i < feet_contact_states.size(); ++i)
    {
        std::array<bool, 5> combined{};
        // 拷贝四足部分
        for (int j = 0; j < 4; ++j)
        {
            combined[j] = feet_contact_states[i][j];
        }
        // 加上机械臂部分
        combined[4] = arm_contact_states[i][0];

        // 转换为 std::vector<bool> 并插入
        std::vector<bool> combined_vec(combined.begin(), combined.end());
        contact_states.push_back(combined_vec);
    }


    MPCSolver mpc_solver(space, nsteps, nu, x0, u0, contact_ids,
                         arm_contact_poses, feet_contact_poses, contact_states, yaml_loader);

    auto result = mpc_solver.solve();
    auto xs = result.first;
    auto us = result.second;

    std::vector<VectorXd> us_selected;

    for (const auto &u : us)
    {
        // 检查维度是否足够
        if (u.size() >= 15)
        {
            Eigen::Vector3d u_part = u.segment<3>(12); // 从第12个开始取3个元素（索引12,13,14）
            us_selected.push_back(u_part);
        }
        else
        {
            std::cerr << "Warning: u vector size is too small: " << u.size() << std::endl;
        }
    }

    saveVectorsToCsv("solo_kinodynamics_result_xs.csv", xs);
    saveVectorsToCsv("solo_kinodynamics_result_us.csv", us_selected);

    return 0;
}
