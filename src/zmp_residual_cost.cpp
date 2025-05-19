#include "zmp_residual_cost.hpp"

ZmpResidualCost::ZmpResidualCost(MultibodyPhaseSpace space, int nu,
                                 std::vector<FrameIndex> contact_frame_id,
                                 std::vector<bool> contact_state,
                                 int force_size,
                                 MatrixXd weight)
    : CostAbstract(space, nu), space_(space),
      contact_frame_id_(contact_frame_id), weight_(weight), force_size_(force_size) {}

void ZmpResidualCost::evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                               CostData &data) const
{
    ZmpResidualCostData &d = static_cast<ZmpResidualCostData &>(data);

    const auto &model = space_.getModel();
    const VectorXd q = x.head(model.nq);
    const VectorXd f_contact = u.head(force_size_ * contact_frame_id_.size());

    // 计算zmp位置
    pinocchio::forwardKinematics(model, d.data_, q);
    pinocchio::updateFramePlacements(model, d.data_);

    Vector3d F_c = Vector3d::Zero();
    Vector3d M_c = Vector3d::Zero();
    Vector3d f_i;
    for (size_t i = 0; i < contact_frame_id_.size(); i++)
    {
        f_i = f_contact.segment(i * force_size_, force_size_);
        F_c += f_i;
        M_c += (d.data_.oMf[contact_frame_id_[i]].translation()).cross(f_i);
    }
    Vector3d n(0, 0, 1);
    Vector3d pos_zmp = n.cross(M_c) / (n.transpose() * F_c);

    // 计算接触点中心
    Vector3d pos_center = Vector3d::Zero();
    int num_contact = 0;
    for (size_t i = 0; i < contact_state_.size(); i++)
    {
        if (contact_state_[i]) // 只考虑支撑腿
        {
            pos_center += d.data_.oMf[contact_frame_id_[i]].translation();
            num_contact++;
        }
    }
    pos_center /= num_contact;

    // 计算zmp残差(只考虑x,y方向)
    VectorXd zmp_residual = (pos_zmp - pos_center).head(2);

    data.value_ = zmp_residual.transpose() * weight_ * zmp_residual;
}

void ZmpResidualCost::computeGradients(const ConstVectorRef &x, const ConstVectorRef &u,
                                       CostData &data) const
{
}

void ZmpResidualCost::computeHessians(const ConstVectorRef &x, const ConstVectorRef &u,
                                      CostData &data) const
{
}

std::shared_ptr<CostData> ZmpResidualCost::createData() const
{
    return std::make_shared<ZmpResidualCostData>(*this);
}

ZmpResidualCostData::ZmpResidualCostData(const ZmpResidualCost &cost)
    : CostData(cost)
{
    data_ = pinocchio::Data(cost.space_.getModel());
}