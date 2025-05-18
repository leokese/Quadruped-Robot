#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <aligator/core/stage-model.hpp>
#include <aligator/modelling/dynamics/kinodynamics-fwd.hpp>
#include <aligator/modelling/dynamics/integrator-euler.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/multibody/centroidal-momentum-derivative.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/multibody/frame-translation.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <aligator/modelling/centroidal/centroidal-friction-cone.hpp>
#include <aligator/modelling/multibody/fly-high.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/modelling/autodiff/finite-difference.hpp>
#include <aligator/modelling/autodiff/cost-finite-difference.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>
#include <proxsuite-nlp/modelling/constraints/negative-orthant.hpp>
#include <proxsuite-nlp/modelling/constraints/equality-constraint.hpp>
#include <aligator/modelling/function-xpr-slice.hpp>

#include "joint_coordinate_residual.hpp"

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorX;
using Eigen::Vector3;

using pinocchio::Data;
using pinocchio::FrameIndex;
using pinocchio::Model;
using pinocchio::SE3;
using MultibodyPhaseSpace = proxsuite::nlp::MultibodyPhaseSpace<double>;
using IntegratorEuler = aligator::dynamics::IntegratorEulerTpl<double>;
using KinodynamicsFwdDynamics = aligator::dynamics::KinodynamicsFwdDynamicsTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using CostStack = aligator::CostStackTpl<double>;
using CentroidalMomentumDerivativeResidual = aligator::CentroidalMomentumDerivativeResidualTpl<double>;
using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
using QuadraticResidualCost = aligator::QuadraticResidualCostTpl<double>;
using FrameTranslationResidual = aligator::FrameTranslationResidualTpl<double>;
using FramePlacementResidual = aligator::FramePlacementResidualTpl<double>;
using CentroidalFrictionConeResidual = aligator::CentroidalFrictionConeResidualTpl<double>;
using NegativeOrthant = proxsuite::nlp::NegativeOrthantTpl<double>;
using EqualityConstraint = proxsuite::nlp::EqualityConstraintTpl<double>;
using TrajOptProblem = aligator::TrajOptProblemTpl<double>;
using SolverProxDDP = aligator::SolverProxDDPTpl<double>;
using BoxConstraint = proxsuite::nlp::BoxConstraintTpl<double>;
using FlyHighResidual = aligator::FlyHighResidualTpl<double>;
using CostFiniteDifference = aligator::autodiff::CostFiniteDifferenceHelper<double>;
using FrameVelocityResidual = aligator::FrameVelocityResidualTpl<double>;
using FunctionSliceXpr = aligator::FunctionSliceXprTpl<double>;
