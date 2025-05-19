#pragma once
#include "common/types.hpp"

#include <aligator/core/function-abstract.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/autodiff/cppad.hpp>

using StageFunction = aligator::StageFunctionTpl<double>;
using StageFunctionData = aligator::StageFunctionDataTpl<double>;

struct ZmpResidual : StageFunction
{
    ZmpResidual(int ndx, int nu, const Model &model,
                std::vector<FrameIndex> contact_frame_id,
                std::vector<bool> contact_state,
                int force_size);

    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                  StageFunctionData &data) const;

    void computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u,
                          StageFunctionData &data) const;

    std::shared_ptr<StageFunctionData> createData() const;

    pinocchio::Model model_;
    std::vector<FrameIndex> contact_frame_id_;
    std::vector<bool> contact_state_;
    int force_size_;
    int num_contact_frame_;
};

struct ZmpResidualData : StageFunctionData
{
    ZmpResidualData(const ZmpResidual &resdl);

    pinocchio::Data data_;
    CppAD::ADFun<double> ad_zmp_residual_; // todo: 使用cppad求导
};