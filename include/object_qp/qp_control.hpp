#pragma once

#include "task.hpp"
#include <iostream>
#include <qpOASES.hpp>
#include <memory>

class QPControl
{
public:
    QPControl(int num_decision);

    void setCostTask(const Task_qp &cost_task);
    void setConstraintTask(const Task_qp &eq_task, const Task_qp &ineq_task);

    Eigen::VectorXd solve();

    private:
    int num_decision_;
    int num_constraint_;
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_, A_;
    Eigen::VectorXd g_, lbA_, ubA_;
    Eigen::VectorXd qp_sol_;
};
