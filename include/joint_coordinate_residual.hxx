#pragma once

#include "joint_coordinate_residual.hpp"

namespace aligator {

template <typename Scalar>
JointCoordinateResidualTpl<Scalar>::JointCoordinateResidualTpl(
    const int ndx, const int nu, const Model &model,
    const pinocchio::FrameIndex frame_id,
    const int dimension)
    : Base(ndx, nu, 1), pin_model_(model), frame_id_(frame_id), dimension_(dimension) {}

template <typename Scalar>
void JointCoordinateResidualTpl<Scalar>::evaluate(const ConstVectorRef &x, BaseData &data) const {
  Data &d = static_cast<Data &>(data);
  pinocchio::DataTpl<Scalar> &pdata = d.pin_data_;
  const ConstVectorRef q = x.head(pin_model_.nq);
  pinocchio::forwardKinematics(pin_model_, pdata, q);
  pinocchio::updateFramePlacements(pin_model_, pdata);
  
  Vector3s p = computeFramePosition(pdata, frame_id_);

  d.value_[0] = p(dimension_);
}

template <typename Scalar>
void JointCoordinateResidualTpl<Scalar>::computeJacobians(const ConstVectorRef &x, BaseData &data) const {
  Data &d = static_cast<Data &>(data);
  pinocchio::DataTpl<Scalar> &pdata = d.pin_data_;
  pinocchio::computeJointJacobians(pin_model_, pdata);
  
  pinocchio::getFrameJacobian(pin_model_, pdata, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, d.fJf_);
 
  d.Jx_.setZero();
  d.Jx_.leftCols(pin_model_.nv) = d.fJf_.row(dimension_);
}

template <typename Scalar>
typename JointCoordinateResidualTpl<Scalar>::Vector3s
JointCoordinateResidualTpl<Scalar>::computeFramePosition(const pinocchio::DataTpl<Scalar> &pdata, pinocchio::FrameIndex frame_id) const {
  return pdata.oMf[frame_id].translation();
}

template <typename Scalar>
JointCoordinateDataTpl<Scalar>::JointCoordinateDataTpl(
    const JointCoordinateResidualTpl<Scalar> &model)
    : Base(model.ndx1, model.nu, 1), pin_data_(model.pin_model_),
      fJf_(6, model.pin_model_.nv){
  fJf_.setZero();
} // namespace aligator

}