#include "object_qp/qp_control.hpp"
#include <iostream>

QPControl::QPControl(int num_decision)
    : num_decision_(num_decision)
{
    H_.setIdentity(num_decision, num_decision);
    g_.setZero(num_decision);
    qp_sol_.setZero(num_decision);
}

void QPControl::setCostTask(const Task_qp &cost_task)
{
    const auto &A = cost_task.A();
    const auto &b = cost_task.b();

    H_.noalias() = A.transpose() * A;
    g_.noalias() = -A.transpose() * b;
}

void QPControl::setConstraintTask(const Task_qp &eq_task, const Task_qp &ineq_task)
{
    const int eq_size = eq_task.b().size();
    const int ineq_size = ineq_task.b().size();
    num_constraint_ = eq_size + ineq_size;

    // 只有在需要时才重新分配内存
    if (A_.rows() < num_constraint_ || A_.cols() != num_decision_)
    {
        A_.resize(num_constraint_, num_decision_);
        lbA_.resize(num_constraint_);
        ubA_.resize(num_constraint_);
    }

    if (eq_size > 0)
    {
        A_.topRows(eq_size) = eq_task.A();
        lbA_.head(eq_size) = eq_task.b();
        ubA_.head(eq_size) = eq_task.b();
    }

    if (ineq_size > 0)
    {
        A_.bottomRows(ineq_size) = ineq_task.A();
        lbA_.tail(ineq_size).setConstant(-qpOASES::INFTY);
        ubA_.tail(ineq_size) = ineq_task.b();
    }
}

Eigen::VectorXd QPControl::solve()
{
    auto qpProblem = qpOASES::QProblem(num_decision_, num_constraint_);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);

    int nWSR = 20;

    qpProblem.init(H_.data(), g_.data(), A_.data(), nullptr, nullptr, lbA_.data(), ubA_.data(), nWSR);
    qpProblem.getPrimalSolution(qp_sol_.data());

    return qp_sol_;
}
