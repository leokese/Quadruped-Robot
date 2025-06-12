#include "object_qp/object_planner.hpp"

ObjectPlanner::ObjectPlanner(double mass, double mu)
{
    n_dof_ = 3;
    force_size_ = 3;
    mass_ = mass;
    mu_ = mu;

    kp_.resize(n_dof_);
    kd_.resize(n_dof_);
    kp_ << 100, 100, 100;
    kd_ << 50, 50, 50;

    num_decision_ = n_dof_ + force_size_; // 决策变量为加速度和力矩

    qp_control_ = std::make_unique<QPControl>(num_decision_);
}

VectorXd ObjectPlanner::solve(const VectorXd &p_ref, const VectorXd &v_ref,
                                 const VectorXd &p, const VectorXd &v)
{
    Task_qp dynamics_task = buildDynamicsEquationTask(p, v);
    Task_qp acc_tracking_task = buildAccTrackingTask(p_ref, v_ref, p, v);
    Task_qp force_limit_task = buildForceLimitTask();
    Task_qp force_min_task = buildForceMinTask(); 
    Task_qp cost_task = acc_tracking_task * w1_ + force_min_task * w2_;

    qp_control_->setCostTask(cost_task);
    qp_control_->setConstraintTask(dynamics_task, force_limit_task);

    VectorXd solution = qp_control_->solve();

    return solution; // 返回加速度和操作力
}

Task_qp ObjectPlanner::buildDynamicsEquationTask(const VectorXd &p, const VectorXd &v)
{
    MatrixXd A(n_dof_, num_decision_);
    VectorXd b(n_dof_);
    VectorXd c(n_dof_);

    A.leftCols(n_dof_) = mass_ * Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    A.rightCols(n_dof_) = -Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    
    double v_norm = v.norm() + 1e-6; // 防止除以0
    b = -mu_ * mass_ * gravity_ / v_norm * v;
    c = -mu_ / v_norm * v;

    A.col(num_decision_ - 1) += c;

    return Task_qp(A, b);
}

Task_qp ObjectPlanner::buildForceLimitTask()
{
    MatrixXd A(n_dof_, num_decision_);
    VectorXd b(n_dof_);

    A.leftCols(n_dof_).setZero();
    A.rightCols(n_dof_) = Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    
    fz_limit_ = mass_ * gravity_; // z方向的力限制为重力

    b << fx_limit_, fy_limit_, fz_limit_;

    return Task_qp(A, b);
}

Task_qp ObjectPlanner::buildForceMinTask()
{
    MatrixXd A(n_dof_, num_decision_);
    VectorXd b(n_dof_);

    A.setZero();
    A.rightCols(n_dof_).setIdentity();
    
    b.setZero();

    return Task_qp(A, b);
}

Task_qp ObjectPlanner::buildAccTrackingTask(const VectorXd &p_ref, const VectorXd &v_ref,
                                            const VectorXd &p, const VectorXd &v)
{

    MatrixXd A(n_dof_, num_decision_);
    VectorXd b(n_dof_);

    std::cout << std::fixed << std::setprecision(2);  // 设置全局浮点数格式（保留两位小数）
    std::cout << "p_ref: " << p_ref.transpose() << std::endl;
    std::cout << "v_ref: " << v_ref.transpose() << std::endl;
    std::cout << "p: " << p.transpose() << std::endl;
    std::cout << "v: " << v.transpose() << std::endl;
    VectorXd a_ref = kp_.cwiseProduct(p_ref - p) + kd_.cwiseProduct(v_ref - v);
    //std::cout << "a_ref: " << a_ref.transpose() << std::endl;

    A.leftCols(n_dof_) = Eigen::MatrixXd::Identity(n_dof_, n_dof_);
    A.rightCols(n_dof_).setZero();
    b = a_ref;

    return Task_qp(A, b);
}
