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
    std::string urdf_path = "/home/zishang/cpp_workspace/Quadruped-Robot/robot/galileo_mini/robot.urdf";
    std::string yaml_path = "/home/zishang/cpp_workspace/Quadruped-Robot/config/parameters.yaml";
    // std::string urdf_path = "/home/robot/文档/vs_project/quadruped_mpc_5/robot/galileo_mini/robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    Data data(model);
    MPCSettings mpc_settings;
    const int nq = model.nq;
    const int nv = model.nv;
    const int force_size = mpc_settings.force_size;
    const int nc = 4;                        // contact number
    const int nu = nc * force_size + nv - 6; // input number
    MultibodyPhaseSpace space(model);
    const int ndx = space.ndx();
    YamlLoader yaml_loader(yaml_path);

    ////////////////////////// 生成初始状态 //////////////////////////////
    VectorXd q0(model.nq);
    q0 << 0, 0, 0.38, 0, 0, 0, 1,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44;

    VectorXd x0(nq + nv);
    x0 << q0, VectorXd::Zero(nv);
    VectorXd u0(nu);
    double mass = pinocchio::computeTotalMass(model);
    Vector3d f_foot_ref(0, 0, -mass * mpc_settings.gravity[2] / 4.0);
    // u0 << f_foot_ref, f_foot_ref, f_foot_ref, f_foot_ref,
    //     VectorXd::Zero(nv - 6);
    u0.setZero();

    const FrameIndex FL_id = model.getFrameId("FL_foot_link", pinocchio::BODY);
    const FrameIndex FR_id = model.getFrameId("FR_foot_link", pinocchio::BODY);
    const FrameIndex HL_id = model.getFrameId("HL_foot_link", pinocchio::BODY);
    const FrameIndex HR_id = model.getFrameId("HR_foot_link", pinocchio::BODY);
    const FrameIndex body_id = model.getFrameId("base_link", pinocchio::BODY);

    std::vector<FrameIndex> contact_ids = {FL_id, FR_id, HL_id, HR_id};

    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateFramePlacements(model, data);

    SE3 FL_pose = data.oMf[FL_id];
    SE3 FR_pose = data.oMf[FR_id];
    SE3 HL_pose = data.oMf[HL_id];
    SE3 HR_pose = data.oMf[HR_id];

    std::vector<Vector3d> init_foot_pos = {FL_pose.translation(), FR_pose.translation(),
                                           HL_pose.translation(), HR_pose.translation()};

    SE3 init_body_pose = data.oMf[body_id];

    ////////////////////////// 生成腿部接触状态与位姿 //////////////////////////////
    std::vector<std::vector<Vector3d>> feet_contact_poses;
    std::vector<std::vector<bool>> feet_contact_states;

    const int n_qs = 5;  // 离散时刻的全接触支持数量
    const int n_ds = 40; // 离散时刻的双足接触支持数量
    const int steps = 2; // 生成多少组步态

    double swing_apex = 0.05; // 抬腿高度
    double x_forward = -0.2;  // 前进距离

    // 最终生成 steps*(2*n_qs + 2*n_ds) 个离散时刻的足端接触状态与位姿
    Gait gait = Gait(steps, n_qs, n_ds, init_foot_pos, swing_apex, x_forward);
    int nsteps = gait.nsteps; // 离散时刻的数量
    feet_contact_states = gait.generateFootStates();
    feet_contact_poses = gait.generateFootTrajectory();

    ////////////////////////// 生成身体期望接触位姿 //////////////////////////////
    SE3 final_body_pose = SE3(init_body_pose.rotation(),
                              init_body_pose.translation() + Vector3d(-0.6, 0, 0));
    std::vector<SE3> body_pose = generateSE3Trajectory(init_body_pose, final_body_pose, nsteps);

    ////////////////////////// 生成期望状态 //////////////////////////////
    std::vector<VectorXd> x_ref(nsteps, x0);
    for (size_t i = 0; i < nsteps; i++)
    {
        x_ref[i].head(3) = body_pose[i].translation();
        x_ref[i].segment(3, 4) = Eigen::Quaterniond(body_pose[i].rotation()).coeffs();
    }
    std::vector<VectorXd> u_ref(nsteps, u0);

    /////////////////////////// 热启动设置 //////////////////////////////
    std::vector<VectorXd> x_init(nsteps + 1, x0);
    VectorXd u_nom = VectorXd::Zero(nu);
    for (int i = 0; i < 4; ++i)
    {
        u_nom.segment(i * force_size, force_size) = f_foot_ref;
    }
    std::vector<VectorXd> us_init(nsteps, u_nom);

    MPCSolver mpc_solver(space, nsteps, nu, x0, x_ref, u_ref, contact_ids,
                         feet_contact_poses, feet_contact_states, yaml_loader);

    auto result = mpc_solver.solve(x0, x_init, us_init);
    auto xs = result.first;
    auto us = result.second;

    saveVectorsToCsv("solo_kinodynamics_result_xs.csv", xs);
    saveVectorsToCsv("solo_kinodynamics_result_us.csv", us);

    /////////////////////////// 记录zmp位置 //////////////////////////////
    std::vector<VectorXd> zmp_positions(nsteps);
    std::vector<VectorXd> foot_positions(nsteps, VectorXd::Zero(3 * nc));
    std::vector<VectorXd> contact_state(nsteps, VectorXd::Zero(4));

    int nf = contact_ids.size() * force_size;
    for (int i = 0; i < nsteps; ++i)
    {
        pinocchio::forwardKinematics(model, data, xs[i].head(nq));
        pinocchio::updateFramePlacements(model, data);
        zmp_positions[i] = calcZmpPosition<double>(data, us[i].head(nf), feet_contact_states[i], contact_ids, force_size);
        for (int k = 0; k < 4; k++)
        {
            foot_positions[i].segment(k * 3, 3) = data.oMf[contact_ids[k]].translation();
            contact_state[i](k) = feet_contact_states[i][k];
        }
    }
    saveVectorsToCsv("solo_kinodynamics_result_zmp_pos.csv", zmp_positions);
    saveVectorsToCsv("solo_kinodynamics_result_feet_pos.csv", foot_positions);
    saveVectorsToCsv("solo_kinodynamics_result_contact_state.csv", contact_state);

    return 0;
}
