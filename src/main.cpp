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

    // 读取规划轨迹
    std::vector<Waypoint> object_trajectory;
    std::string filepath = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/trajectory.txt";
    if (!readTrajectoryFromFile(filepath, object_trajectory)) {
        std::cerr << "Failed to read trajectory from file." << std::endl;
        return -1;
    }
    std::cout << "Trajectory loaded, points count: " << object_trajectory.size() << std::endl;


    // 初始化webots
    WebotsInterface webots;
    webots.step();  // 先执行一次step，确保webots初始化完成

    // 接收仿真初始化模型参数
    VectorXd x_measure(49);
    Vector3d object_position, object_linear_velocity, object_angular_velocity;
    Matrix3d object_rotation;
    Vector3d force(0, 0, 0); // 用于施加在物体上的力
    webots.recvState(x_measure, object_position, object_linear_velocity, object_angular_velocity, object_rotation);

    // 求解机械臂末端位姿
    int cube_size = 5; // 立方体的边长
    Vector3d hand_offset(-cube_size / 2.0, 0, cube_size); // 手臂末端相对于立方体中心的偏移
    Matrix3d hand_rotation_offset = Matrix3d::Identity();
    
    // 预测窗口内的手臂末端位姿（步长为mpc_settings.T=90 应该先写mpc_settings初始化，在赋值给mpc，在mpc初始化时赋值权重）
    // 之后修改，现在不用管
    std::vector<SE3> arm_contact_places;
    int T = 90; // MPC预测窗口长度
    int timestep = 0.01; // MPC时间步长

    arm_contact_places = predictArmContactTrajectory(
        object_position, object_rotation, object_linear_velocity, object_angular_velocity,
        hand_offset, hand_rotation_offset, T, timestep);

        
    // 初始化MPC求解
    std::cout << "force: " << force.transpose() << std::endl;
    MPC mpc(model, x_measure, force, arm_contact_places);

    // 获取mpc初始化的解
    std::vector<VectorXd> xs = mpc.getXs();
    std::vector<VectorXd> us = mpc.getUs();
    saveVectorsToCsv("/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/solo_kinodynamics_result_xs.csv", xs);
    saveVectorsToCsv("/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/solo_kinodynamics_result_us.csv", us);


    // // 初始化WBC求解


    // // 进入循环
    // const double dt = 0.001; // Time step for integration

    // while (webots.isRunning())
    // {
        


    //     if (int(itr % 10) == 0)
    //     {
    //         itr = 0;
    //         itr_mpc++;            

    //         std::cout << "itr_mpc: " << itr_mpc << std::endl;

    //     }
        

    // }

    


    return 0;
}
