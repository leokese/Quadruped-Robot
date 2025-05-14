#pragma once

#include "common/types.hpp"

struct Arm
{
    int nsteps;
    const SE3& init_arm_place;
    const SE3& end_arm_place;
    
    Arm(int nsteps_, const SE3& init_arm_place_, const SE3& end_arm_place_)
        : nsteps(nsteps_), init_arm_place(init_arm_place_), end_arm_place(end_arm_place_) {}

    std::vector<SE3> generateArmTrajectory()
    {
        std::vector<SE3> arm_contact_places;
        for (int i = 0; i < nsteps; ++i)
        {
            double alpha = static_cast<double>(i) / (nsteps - 1);  // 插值因子
    
            // 插值平移部分
            Eigen::Vector3d trans = (1 - alpha) * init_arm_place.translation() + alpha * end_arm_place.translation();
    
            // 插值旋转部分（使用四元数）
            Eigen::Quaterniond quat_init(init_arm_place.rotation());
            Eigen::Quaterniond quat_end(end_arm_place.rotation());
            Eigen::Quaterniond quat_interp = quat_init.slerp(alpha, quat_end);
    
            // 构造新的 SE3 位姿
            SE3 pose_interp(quat_interp.toRotationMatrix(), trans);
            arm_contact_places.push_back(pose_interp);
        }
    
        return arm_contact_places;
    }

};