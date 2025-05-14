#pragma once
#include <aligator/core/cost-abstract.hpp>
#include <proxsuite-nlp/modelling/spaces/multibody.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "common/types.hpp"

using CostAbstract = aligator::CostAbstractTpl<double>;
using MultibodyPhaseSpace = proxsuite::nlp::MultibodyPhaseSpace<double>;
using CostData = aligator::CostDataAbstractTpl<double>;


struct ZmpResidualCostData;

struct ZmpResidualCost : CostAbstract
{

    ZmpResidualCost(MultibodyPhaseSpace space,
                          int nu,
                          std::vector<int> contact_feet_id,
                          double cf);

    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                  CostData &data) const override;

    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u,
                          CostData &data) const override;

    void computeHessians(const ConstVectorRef &x, const ConstVectorRef &u,
                         CostData &data) const override;

    std::shared_ptr<CostData> createData() const override;

    MultibodyPhaseSpace space_;
    std::vector<int> contact_feet_id_;
    double cf_ = 1.0;

    std::vector<int> foot_frame_ids_{11, 19, 27, 35};
};

struct ZmpResidualCostData : CostData
{
    ZmpResidualCostData(const ZmpResidualCost &cost);

    pinocchio::Data data_;
};