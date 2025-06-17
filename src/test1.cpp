#include <cmath>
#include "gait_schedule.hpp"
#include "arm_schedule.hpp"
#include "mpc/mpc_solver.hpp"
#include "mpc/mpc.hpp"
#include "mpc/interpolator.hpp"
#include "utils/logger.hpp"
#include "webots_interface.hpp"
#include "wbc/weighted_wbc.hpp"
#include "wbc/relaxed_wbc.hpp"

std::vector<SE3> predictArmContactTrajectory(
    const Vector3d &object_position,
    const Matrix3d &object_rotation,
    const Vector3d &object_linear_velocity,
    const Vector3d &object_angular_velocity,
    const Vector3d &hand_offset,
    const Matrix3d &hand_rotation_offset,
    int T,
    double timestep)
{
    std::vector<SE3> arm_contact_places;
    arm_contact_places.reserve(T); // 预留空间提升效率

    // 初始位姿与速度
    SE3 object_pose(object_rotation, object_position);

    Vector6d object_velocity_vec;
    object_velocity_vec.head<3>() = object_linear_velocity;
    object_velocity_vec.tail<3>() = object_angular_velocity;
    Motion object_velocity(object_velocity_vec);

    // 预测每一步的手臂末端位姿
    for (int i = 0; i < T; ++i)
    {
        // 使用 exp6 对速度进行积分，更新物体位姿
        object_pose = pinocchio::exp6(object_velocity * timestep) * object_pose;

        const Matrix3d &R = object_pose.rotation();
        const Vector3d &p = object_pose.translation();

        Vector3d hand_position_world = R * hand_offset + p;
        Matrix3d hand_rotation_world = R * hand_rotation_offset;

        SE3 hand_pose(hand_rotation_world, hand_position_world);
        arm_contact_places.push_back(hand_pose);
    }

    return arm_contact_places;
}


int main(int argc, char const *argv[])
{
    // 生成模型
    std::string urdf_path = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/robot/galileo_v1d6_x5_description/galileo_v1d6_x5.urdf";
    // std::string urdf_path = "/home/robot/文档/vs_project/quadruped_mpc_5/robot/galileo_mini/robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);

    // 读取规划轨迹
    std::vector<Waypoint> object_trajectory;
    std::string filepath = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/trajectory.txt";
    if (!readTrajectoryFromFile(filepath, object_trajectory)) {
        std::cerr << "Failed to read trajectory from file." << std::endl;
        return -1;
    }
 
    VectorXd object_p_ref(3), object_v_ref(3);

    // 物体，机械臂末端信息
    int cube_size = 5; // 立方体的边长
    Vector3d hand_offset(-cube_size / 2.0, 0, cube_size); // 手臂末端相对于立方体中心的偏移
    Matrix3d hand_rotation_offset = Matrix3d::Identity();
    ObjectPlanner object_planner(1.0, 1.0);

    // 初始化webots
    WebotsInterface webots;

    // 接收仿真初始化模型参数
    VectorXd q0(25), v0(24);
    VectorXd q_body(19),q_arm(6);
    q_body << -1.1, 0, 0.41, 0, 0, 0, 1,
              0, 0.72, -1.44,
              0, 0.72, -1.44,
              0, 0.72, -1.44,
              0, 0.72, -1.44;
    q_arm << 0, 2.4, 1.5, 0.4, 0, 0;
    q0 << q_body, q_arm;
    v0.setZero();
    v0(0) = -0.4; // base x velocity
    


    VectorXd x_measure(49);
    x_measure << q0, v0;

    Vector3d force(0, 0, 0); // 用于施加在物体上的力

    std::vector<SE3> arm_contact_places;
    const FrameIndex arm_id = model.getFrameId("link8", pinocchio::BODY);

    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateFramePlacements(model, data);
    SE3 arm_contact_place = data.oMf[arm_id];

    std::cout << "arm_contact_place: " << arm_contact_place.translation().transpose() << std::endl;
    

    arm_contact_places.assign(90, arm_contact_place);


    // 初始化MPC求解
    int T = 90; // MPC预测窗口长度
    int timestep = 0.01; // MPC时间步长
    std::cout << x_measure.transpose() << std::endl;
    MPC mpc(model, x_measure, force, arm_contact_places);

    // 初始化WBC
    int force_size = mpc.mpc_settings_.force_size;

    RelaxedWbcSettings Rwbc_settings;
    Rwbc_settings.contact_ids = mpc.getFeetIds();
    Rwbc_settings.mu = mpc.mpc_settings_.mu;
    Rwbc_settings.force_size = force_size;
    Rwbc_settings.w_acc = 1;
    Rwbc_settings.w_force = 10;
    Rwbc_settings.verbose = false;
    RelaxedWbc relaxed_wbc(Rwbc_settings, model);


    // 进入循环
    Interpolator interpolator(model);

    int itr = 0;
    int itr_mpc = 0;
    const double dt = 0.001; // Time step for integration

    while (webots.isRunning())
    {
        double current_time = webots.current_time();

        // 得到反馈值
        Vector3d object_position, object_linear_velocity, object_angular_velocity;
        Matrix3d object_rotation;
        Vector3d force(0, 0, 0); // 用于施加在物体上的力
        webots.recvState(x_measure, object_position, object_linear_velocity, object_angular_velocity, object_rotation);

        // 物体QP求解
        int idx = findNearestWaypointIndex(object_trajectory, current_time);

        if (idx >= static_cast<int>(object_trajectory.size()) - 1) {
            std::cout << "Reached the end of trajectory at time: " << current_time << std::endl;
            break;
        }

        object_p_ref = object_trajectory[idx].position_;
        object_v_ref = object_trajectory[idx].velocity_;

        VectorXd solution = object_planner.solve(object_p_ref, object_v_ref, object_position, object_linear_velocity);

        VectorXd object_acc = solution.head(3);
        force = solution.tail(3);

        if (int(itr % 10) == 0)
        {
            // 预测窗口内的手臂末端位姿（步长为mpc_settings.T=90 应该先写mpc_settings初始化，在赋值给mpc，在mpc初始化时赋值权重）
            // 之后修改，现在不用管
            std::vector<SE3> arm_contact_places;           

            arm_contact_places = predictArmContactTrajectory(
                object_position, object_rotation, object_linear_velocity, object_angular_velocity,
                hand_offset, hand_rotation_offset, T, timestep);

            // 更新MPC
            mpc.iterate(x_measure, force, arm_contact_places, current_time);

            itr = 0;
            itr_mpc++;            

            std::cout << "itr_mpc: " << itr_mpc << std::endl;
            std::cout << "--------------------------" << std::endl;
        }

        double delay = itr * dt;

        VectorXd acc_interp, u_interp;
        interpolator.interpolateLinear(delay, timestep, mpc.getAs(), acc_interp);
        interpolator.interpolateLinear(delay, timestep, mpc.getUs(), u_interp);

        ////////////////////// 松弛WBC //////////////////////
        
        relaxed_wbc.solveQP(mpc.getContactStates(0),
            x_measure.head(mpc.getNq()),
            x_measure.tail(mpc.getNv()),
            acc_interp,
            VectorXd::Zero(12),
            u_interp.head(mpc.getNk() * force_size));
        webots.sendCmd(relaxed_wbc.solved_torque_, force);

        itr++;

    }

    


    return 0;
}
