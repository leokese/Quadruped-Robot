#include "zmp_residual.hpp"

int main()
{
    ////////////////////////// 生成模型 //////////////////////////////
    std::string urdf_path = "/home/zishang/cpp_workspace/Quadruped-Robot/robot/galileo_mini/robot.urdf";
    Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    Data data(model);
    const int nq = model.nq;
    const int nv = model.nv;
    const int force_size = 3;
    const int nc = 4;                        // contact number
    const int nu = nc * force_size + nv - 6; // input number
    MultibodyPhaseSpace space(model);
    const int ndx = space.ndx();
    Vector3d gravity(0, 0, -9.81);

    const FrameIndex FL_id = model.getFrameId("FL_foot_link", pinocchio::BODY);
    const FrameIndex FR_id = model.getFrameId("FR_foot_link", pinocchio::BODY);
    const FrameIndex HL_id = model.getFrameId("HL_foot_link", pinocchio::BODY);
    const FrameIndex HR_id = model.getFrameId("HR_foot_link", pinocchio::BODY);
    std::vector<FrameIndex> contact_ids = {FL_id, FR_id, HL_id, HR_id};

    ////////////////////////// 生成初始状态 //////////////////////////////
    VectorXd q0(model.nq);
    q0 << 0, 0, 0.38, 0, 0, 0, 1,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44,
        0, 0.72, -1.44;

    VectorXd x0(nq + nv);
    x0 << q0, VectorXd::Zero(nv);
    VectorXd u0(nu);
    double mass = pinocchio::computeTotalMass(model);
    Vector3d f_foot_ref(0, 0, -mass * gravity[2] / 4.0);
    u0 << 1.5 * f_foot_ref, 1.5 * f_foot_ref, f_foot_ref, f_foot_ref,
        VectorXd::Zero(nv - 6);

    std::vector<bool> contact_state = {false, true, false, true};

    ////////////////////////// 计算 zmp //////////////////////////////
    ZmpResidual zmp_residual(ndx, nu, model, contact_ids,
                             contact_state, force_size);
    auto zmp_data = zmp_residual.createData();

    zmp_residual.evaluate(x0, u0, *zmp_data);

    std::cout << "zmp residual: " << zmp_data->value_.transpose() << std::endl;

    return 0;
}