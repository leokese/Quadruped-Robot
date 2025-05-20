#pragma once
#include "common/types.hpp"

#include <aligator/core/function-abstract.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/autodiff/cppad.hpp>

using StageFunction = aligator::StageFunctionTpl<double>;
using StageFunctionData = aligator::StageFunctionDataTpl<double>;

template <typename Scalar>
Vector3<Scalar> calcZmpPosition(const pinocchio::DataTpl<Scalar> &data,
                                const VectorX<Scalar> &f_contact,
                                const std::vector<bool> &contact_state,
                                const std::vector<FrameIndex> &contact_frame_id,
                                int force_size)
{
    Vector3<Scalar> F_c = Vector3<Scalar>::Zero();
    Vector3<Scalar> M_c = Vector3<Scalar>::Zero();
    Vector3<Scalar> f_i;
    for (size_t i = 0; i < contact_frame_id.size(); i++)
    {
        if (contact_state[i])
        {
            f_i = f_contact.segment(i * force_size, force_size);
            F_c += f_i;
            M_c += (data.oMf[contact_frame_id[i]].translation()).cross(f_i);
        }
    }
    Vector3<Scalar> n(Scalar(0), Scalar(0), Scalar(1));
    Vector3<Scalar> pos_zmp = n.cross(M_c) / (n.transpose() * F_c);

    return pos_zmp;
}

template <typename Scalar>
Vector3<Scalar> calcContactCenterPosition(const pinocchio::DataTpl<Scalar> &data,
                                          const std::vector<bool> &contact_state,
                                          const std::vector<FrameIndex> &contact_frame_id)
{
    // 计算接触点中心
    Vector3<Scalar> pos_center = Vector3<Scalar>::Zero();
    int num_contact = 0;
    for (size_t i = 0; i < contact_state.size(); i++)
    {
        if (contact_state[i]) // 只考虑支撑腿
        {
            pos_center += data.oMf[contact_frame_id[i]].translation();
            num_contact++;
        }
    }
    pos_center /= num_contact;

    return pos_center;
}

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