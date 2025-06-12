#include "common/types.hpp"

#include "mpc/mpc_solver.hpp"
#include "yaml_loader.hpp"
#include "gait_schedule.hpp"

class MPC
{
public:
    MPC(Model model, VectorXd x0, Vector3d f_pull, std::vector<SE3> arm_contact_places);

    void iterate(const ConstVectorRef &x, std::vector<SE3> arm_contact_places, double current_time);

    MPCSettings mpc_settings_;

    std::vector<VectorXd> getXs() const { return xs_; }
    std::vector<VectorXd> getUs() const { return us_; }

private:
    Model model_;
    // 声明为智能指针
    std::unique_ptr<MPCSolver> mpc_solver_;
    MultibodyPhaseSpace space_;
    pinocchio::Data data_;

    int nq_;
    int nv_;
    int nu_; 
    int nc_; 

    std::vector<VectorXd> xs_;
    std::vector<VectorXd> us_;
    std::vector<VectorXd> as_;
    VectorXd x0_;
    VectorXd x0_init_; // 记录初始状态（腿关节作为后续参考）
    VectorXd u0_;
    std::vector<VectorXd> x_ref_;
    std::vector<VectorXd> u_ref_;
    std::vector<VectorXd> x_init_;
    std::vector<VectorXd> u_init_;
    std::vector<FrameIndex> contact_ids_;
    std::vector<FrameIndex> hip_ids_;
    FrameIndex base_id_;
    std::vector<std::vector<bool>> contact_states_;
    std::vector<std::vector<bool>> feet_contact_states_;
    std::vector<std::vector<Vector3d>> feet_contact_poses_;
    std::vector<SE3> arm_contact_places_;
    Vector3d f_pull_; 
    Vector3d f_foot_ref_; // 仅用做热启动，不可以作为参考力

    Eigen::Vector3d init_pose_;
    Eigen::Vector3d next_pose_;
    Eigen::Vector2d twist_vect_;
    Vector6d velocity_base_;



    void initMPC(const YamlLoader &yaml_loader);

    void updatePinocchioInfo(const VectorXd &q, const VectorXd &v);

    void updateContactFeetStates();
    void updateContactStates();

    void updateContactFeetPoses();

    void setX_U_ref();
    void setX_U_init_0(); // 第一次热启动设置
    void setX_U_init(); // 后续热启动设置

};