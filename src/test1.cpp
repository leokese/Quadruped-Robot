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


int main(int argc, char const *argv[])
{
    // 生成模型
    std::string urdf_path = "/home/robot/文档/vs_project/Quadruped-Robot/Quadruped-Robot-3/robot/galileo_v1d6_x5_description/galileo_v1d6_x5.urdf";
    // std::string urdf_path = "/home/robot/文档/vs_project/quadruped_mpc_5/robot/galileo_mini/robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Data data(model);


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
    std::cout << x_measure.transpose() << std::endl;
    MPC mpc(model, x_measure, force, arm_contact_places);

    // const FrameIndex FL_id = model.getFrameId("FL_foot", pinocchio::BODY);
    // const FrameIndex FR_id = model.getFrameId("FR_foot", pinocchio::BODY);
    // const FrameIndex HL_id = model.getFrameId("RL_foot", pinocchio::BODY);
    // const FrameIndex HR_id = model.getFrameId("RR_foot", pinocchio::BODY);

    // const FrameIndex FL_hip_id = model.getFrameId("FL_thigh", pinocchio::BODY);
    // const FrameIndex FR_hip_id = model.getFrameId("FR_thigh", pinocchio::BODY);
    // const FrameIndex HL_hip_id = model.getFrameId("RL_thigh", pinocchio::BODY);      
    // const FrameIndex HR_hip_id = model.getFrameId("RR_thigh", pinocchio::BODY);

    // const FrameIndex base_id_ = model.getFrameId("base_link", pinocchio::BODY);

    // std::vector<FrameIndex> contact_ids;
    // std::vector<FrameIndex> hip_ids;

    // contact_ids = {FL_id, FR_id, HL_id, HR_id, arm_id};   

    // hip_ids = {FL_hip_id, FR_hip_id, HL_hip_id, HR_hip_id};

    // pinocchio::forwardKinematics(model, data, q0);
    // pinocchio::updateFramePlacements(model, data);

    // for (size_t i = 0; i < contact_ids.size() - 1; ++i)
    // {
    //     std::cout << "contact_id: " << contact_ids[i] << ", position: " 
    //               << data.oMf[contact_ids[i]].translation().transpose() << std::endl;
    // }

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
