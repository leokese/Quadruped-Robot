#pragma once

#include "joint_coordinate_residual.hpp"
namespace aligator {

extern template struct JointCoordinateResidualTpl<context::Scalar>;
extern template struct JointCoordinateDataTpl<context::Scalar>;

} // namespace aligator
