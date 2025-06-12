#include "mpc/zmp_residual.hpp"

ZmpResidual::ZmpResidual(int ndx, int nu, const Model &model,
                         std::vector<FrameIndex> contact_frame_id,
                         std::vector<bool> contact_state,
                         int force_size)
    : StageFunction(ndx, nu, 2),
      model_(model), contact_frame_id_(contact_frame_id),
      contact_state_(contact_state), force_size_(force_size),
      num_contact_frame_(contact_frame_id.size()) {}

void ZmpResidual::evaluate(const ConstVectorRef &x, const ConstVectorRef &u,
                           StageFunctionData &data) const
{
    ZmpResidualData &d = static_cast<ZmpResidualData &>(data);

    const VectorXd q = x.head(model_.nq);
    const VectorXd f_contact = u.head(force_size_ * contact_frame_id_.size());

    // 计算zmp位置
    pinocchio::forwardKinematics(model_, d.data_, q);
    pinocchio::updateFramePlacements(model_, d.data_);

    Vector3d pos_zmp = calcZmpPosition<double>(d.data_, f_contact, contact_state_, contact_frame_id_, force_size_);
    Vector3d pos_center = calcContactCenterPosition<double>(d.data_, contact_state_, contact_frame_id_);

    // 计算zmp残差(只考虑x,y方向)
    data.value_ = (pos_zmp - pos_center).head(2);
}

void ZmpResidual::computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u,
                                   StageFunctionData &data) const
{
    ZmpResidualData &d = static_cast<ZmpResidualData &>(data);
    const int nq = model_.nq;
    const int nv = model_.nv;
    const int nf = contact_frame_id_.size() * force_size_;
    const int nr = 2;

    VectorXd X(nq + nv + nf + nv); // q, v, f, dq
    X << x, u.head(nf), Eigen::VectorXd::Zero(nv);

    Eigen::VectorXd dy_dx_vec = d.ad_zmp_residual_.Jacobian(X);
    Eigen::MatrixXd dy_dx = dy_dx_vec.reshaped(X.size(), nr).transpose();
    Eigen::MatrixXd dy_dq = dy_dx.rightCols(nv);
    Eigen::MatrixXd dy_dv = dy_dx.middleCols(nq, nv);
    Eigen::MatrixXd dy_df = dy_dx.middleCols(nq + nv, nf);

    d.Jx_.leftCols(nv) = dy_dq;
    d.Jx_.rightCols(nv) = dy_dv;
    d.Ju_.leftCols(nf) = dy_df;
}

std::shared_ptr<StageFunctionData> ZmpResidual::createData() const
{
    return std::make_shared<ZmpResidualData>(*this);
}

ZmpResidualData::ZmpResidualData(const ZmpResidual &resdl)
    : StageFunctionData(resdl),
      data_(resdl.model_)
{
    //////////////////////////////////////// 定义cppad函数 ////////////////////////////////////////
    using CppAD::AD;
    using ADVectorX = Eigen::VectorX<AD<double>>;

    pinocchio::ModelTpl<AD<double>> ad_model = resdl.model_.cast<AD<double>>();
    pinocchio::DataTpl<AD<double>> ad_data(ad_model);
    int nq = ad_model.nq;
    int nv = ad_model.nv;
    int nf = resdl.contact_frame_id_.size() * resdl.force_size_;

    ADVectorX ad_X(nq + nv + nf + nv); // q, v, f, dq
    ad_X.setZero();
    CppAD::Independent(ad_X);
    ADVectorX ad_Y(nv);

    ADVectorX ad_q = ad_X.head(nq);
    ADVectorX ad_v = ad_X.segment(nq, nv);
    ADVectorX ad_f = ad_X.segment(nq + nv, nf);
    ADVectorX ad_dq = ad_X.tail(nv);

    ADVectorX ad_q_plus = pinocchio::integrate(ad_model, ad_q, ad_dq);
    pinocchio::forwardKinematics(ad_model, ad_data, ad_q_plus, ad_v);
    pinocchio::updateFramePlacements(ad_model, ad_data);

    ADVectorX pos_zmp = calcZmpPosition<AD<double>>(ad_data, ad_f, resdl.contact_state_, resdl.contact_frame_id_, resdl.force_size_);
    ADVectorX pos_center = calcContactCenterPosition<AD<double>>(ad_data, resdl.contact_state_, resdl.contact_frame_id_);
    ad_Y = (pos_zmp - pos_center).head(2);
    ad_zmp_residual_.Dependent(ad_X, ad_Y);
}
