#pragma once

#include "common/types.hpp"
#include "aligator/core/unary-function.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/frame.hpp>

namespace aligator {

template <typename _Scalar> struct JointCoordinateDataTpl;

template <typename _Scalar>
struct JointCoordinateResidualTpl : UnaryFunctionTpl<_Scalar>, frame_api {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Scalar = _Scalar;
  ALIGATOR_DYNAMIC_TYPEDEFS(Scalar);
  ALIGATOR_UNARY_FUNCTION_INTERFACE(Scalar);

  using BaseData = typename Base::Data;
  using Model = pinocchio::ModelTpl<Scalar>;
  using Data = JointCoordinateDataTpl<Scalar>;

  Model pin_model_;

  pinocchio::FrameIndex frame_id_;
  int dimension_;  // 哪个维度 (0:x, 1:y, 2:z)

  JointCoordinateResidualTpl(const int ndx, const int nu, const Model &model,
                              const pinocchio::FrameIndex frame_id,
                              const int dimension);

  void evaluate(const ConstVectorRef &x, BaseData &data) const;
  void computeJacobians(const ConstVectorRef &x, BaseData &data) const;

  shared_ptr<BaseData> createData() const {
    return std::make_shared<Data>(*this);
  }

protected:
  Vector3s computeFramePosition(const pinocchio::DataTpl<Scalar> &pdata, pinocchio::FrameIndex frame_id) const;
};

template <typename Scalar>
struct JointCoordinateDataTpl : StageFunctionDataTpl<Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base =  StageFunctionDataTpl<Scalar>;
  using PinData = pinocchio::DataTpl<Scalar>;

  PinData pin_data_;

  typename math_types<Scalar>::Matrix6Xs fJf_;

  JointCoordinateDataTpl(const JointCoordinateResidualTpl<Scalar> &model);
  
};

} // namespace aligator

// !!! 这里加上 .txx 文件的引用 !!!
#include "joint_coordinate_residual.txx"
