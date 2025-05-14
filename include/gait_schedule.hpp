#pragma once

#include "common/types.hpp"

struct Gait
{
    int steps;
    int n_qs;
    int n_ds;
    int nsteps;
    double swing_apex; // 抬腿高度
    double x_forward;   // 前进距离
    std::vector<Vector3d> init_foot_pos; 
    
    Gait(int steps_, int n_qs_, int n_ds_, const std::vector<Vector3d>& init_foot_pos_, double swing_apex_,double x_forward_)
        : steps(steps_), n_qs(n_qs_), n_ds(n_ds_), init_foot_pos(init_foot_pos_), swing_apex(swing_apex_), x_forward(x_forward_) 
        {
            nsteps = steps * (2 * n_qs + 2 * n_ds);
        }

    // t_ss 为完整步长周期，ts 为当前步长
    double ztraj(double swing_apex, double t_ss, double ts)
    {
        return swing_apex * std::sin(ts / t_ss * M_PI);
    }

    double xtraj(double x_forward, double t_ss, double ts)
    {
        return x_forward * ts / t_ss;
    }

    std::vector<std::vector<bool>> generateFootStates()
    {
        std::vector<std::vector<bool>> contact_states;

        for (int i = 0; i < steps; ++i)
        {
            // 第一阶段：全接触支持
            for (int j = 0; j < n_qs; ++j)
            {
                contact_states.push_back({true, true, true, true});
            }
            // 第二阶段：第一组双足接触（例如 swing 第 0 和 3 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                contact_states.push_back({false, true, true, false});       
            }

            // 第三阶段：全接触支持
            for (int j = 0; j < n_qs; ++j)
            {
                contact_states.push_back({true, true, true, true});
            }

            // 第四阶段：第二组双足接触（例如 swing 第 1 和 2 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                contact_states.push_back({true, false, false, true});
            }
        }
        
        return contact_states;

    }

    std::vector<std::vector<Vector3d>> generateFootTrajectory()
    {
        std::vector<std::vector<Vector3d>> contact_poses;

        std::vector<Vector3d> current_feet_pos = init_foot_pos;

        for (int i = 0; i < steps; ++i)
        {
            // 第一阶段：全接触支持
            for (int j = 0; j < n_qs; ++j)
            {
                contact_poses.push_back(current_feet_pos); // 全接触支持时，各足平移不变
            }
            // 第二阶段：第一组双足接触（例如 swing 第 0 和 3 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                std::vector<Eigen::Vector3d> target_feet_pos = current_feet_pos;
                target_feet_pos[0](0) = xtraj(x_forward, n_ds, j) + current_feet_pos[0](0);
                target_feet_pos[0](2) = ztraj(swing_apex, n_ds, j) + current_feet_pos[0](2);
                target_feet_pos[3](0) = xtraj(x_forward, n_ds, j) + current_feet_pos[3](0);
                target_feet_pos[3](2) = ztraj(swing_apex, n_ds, j) + current_feet_pos[3](2);
                contact_poses.push_back(target_feet_pos);
                if (j == n_ds - 1)
                {
                    current_feet_pos = target_feet_pos;
                }
            }

            // 第三阶段：全接触支持
            for (int j = 0; j < n_qs; ++j)
            {
                contact_poses.push_back(current_feet_pos);
            }

            // 第四阶段：第二组双足接触（例如 swing 第 1 和 2 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                std::vector<Eigen::Vector3d> target_feet_pos = current_feet_pos;
                target_feet_pos[1](0) = xtraj(x_forward, n_ds, j) + current_feet_pos[1](0);
                target_feet_pos[1](2) = ztraj(swing_apex, n_ds, j) + current_feet_pos[1](2);
                target_feet_pos[2](0) = xtraj(x_forward, n_ds, j) + current_feet_pos[2](0);
                target_feet_pos[2](2) = ztraj(swing_apex, n_ds, j) + current_feet_pos[2](2);
                contact_poses.push_back(target_feet_pos);
                if (j == n_ds - 1)
                {
                    current_feet_pos = target_feet_pos;
                }
            }
        }

        return contact_poses;
    }

};