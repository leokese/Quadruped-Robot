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

    ZmpResidualCost(MultibodyPhaseSpace space, int nu,
                    std::vector<FrameIndex> contact_frame_id,
                    std::vector<bool> contact_state,
                    int force_size,
                    MatrixXd weight);

    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                  CostData &data) const override;

    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u,
                          CostData &data) const override;

    void computeHessians(const ConstVectorRef &x, const ConstVectorRef &u,
                         CostData &data) const override;

    std::shared_ptr<CostData> createData() const override;

    MultibodyPhaseSpace space_;
    std::vector<FrameIndex> contact_frame_id_;
    std::vector<bool> contact_state_;
    int force_size_;
    MatrixXd weight_;
};

struct ZmpResidualCostData : CostData
{
    ZmpResidualCostData(const ZmpResidualCost &cost);

    pinocchio::Data data_;
};