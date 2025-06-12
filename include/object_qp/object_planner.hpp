#pragma once
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "qp_control.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ObjectPlanner
{
public:
    ObjectPlanner(double mass,double mu);
    VectorXd solve(const VectorXd &p_ref, const VectorXd &v_ref,
        const VectorXd &p, const VectorXd &v);

private:
    Task_qp buildDynamicsEquationTask(const VectorXd &p, const VectorXd &v);
    Task_qp buildForceLimitTask();   // f的限制
    Task_qp buildForceMinTask();   // 力的最小化任务
    Task_qp buildAccTrackingTask(const VectorXd &p_ref, const VectorXd &v_ref,
                                const VectorXd &p, const VectorXd &v);

    std::unique_ptr<QPControl> qp_control_;

    int n_dof_;
    int force_size_;
    double mass_;
    double mu_;
    double gravity_ = 9.81;
    double w1_ = 1.0; // 加速度跟踪权重
    double w2_ = 1.0; // 力正则化权重

    double fx_limit_ = 50.0; 
    double fy_limit_ = 50.0; 
    double fz_limit_; 

    int num_decision_;
    VectorXd kp_, kd_;
};
