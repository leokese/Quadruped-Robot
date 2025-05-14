#include "zmp_residual_cost.hpp"

ZmpResidualCost::ZmpResidualCost(MultibodyPhaseSpace space,
                                             int nu, 
                                             std::vector<int> contact_feet_id,
                                             double cf) 
    : CostAbstract(space, nu), space_(space), 
      contact_feet_id_(contact_feet_id), cf_(cf) {}

void ZmpResidualCost::evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                                     CostData &data) const
{
    ZmpResidualCostData &d = static_cast<ZmpResidualCostData &>(data);

    const int num_feet = contact_feet_id_.size();
    int total_contacts = num_feet + 1;
    double cost = 0.0;

    const auto &model = space_.getModel();
    const ConstVectorRef q = x.head(model.nq);
    pinocchio::forwardKinematics(model, d.data_, q);
    pinocchio::updateFramePlacements(model, d.data_);

    VectorX<Scalar> f_contact;
	
	f_contact.resize(3 * total_contacts);  

    for (int i = 0; i < num_feet; i++)
    {
        f_contact.template segment<3>(3 * i) = u.segment(contact_feet_id_[i] * 3, 3);
    }
    f_contact.template segment<3>(3 * num_feet) = u.segment(4 * 3, 3);
	
	// calculate contact wrench
    Vector3<Scalar> F_c = Vector3<Scalar>::Zero();
    Vector3<Scalar> M_c = Vector3<Scalar>::Zero();
    Vector3<Scalar> f_i;
    for (size_t i = 0; i < num_feet; i++)
    {
        f_i = f_contact.template segment<3>(3 * i);
        F_c += f_i;
        M_c += (d.data_.oMf[foot_frame_ids_[contact_feet_id_[i]]].translation()).cross(f_i);
    }

    f_i = f_contact.template segment<3>(3 * num_feet);
    F_c += f_i;
    M_c += (d.data_.oMf[53].translation()).cross(f_i); 

	Vector3d n(0, 0, 1);
	Vector3d ZMP_Point(0, 0, 0);
	ZMP_Point = n.cross(M_c) / (n.transpose() * F_c);


	// 收集接触点
	std::vector<Vector3s> p;
	for (size_t i = 0; i < num_feet; i++) {
		p.push_back(d.data_.oMf[foot_frame_ids_[contact_feet_id_[i]]].translation());
	}

	// 求几何中心 p_center（仅x-y平面）
	Vector3s p_center = Vector3s::Zero();
    for (const auto& pi : p) {
        p_center.head<2>() += pi.head<2>();  // 只加前两维 (x, y)
    }
    p_center.head<2>() /= static_cast<Scalar>(p.size());  // 取平均
    p_center.z() = Scalar(0);  // z维设为0
	p_center /= num_feet;

	// 距离（欧几里得距离）
	Vector3s delta = p_center - ZMP_Point;
	cost = delta.norm();

	// 成本设置
    data.value_ = cf_ * cost;


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