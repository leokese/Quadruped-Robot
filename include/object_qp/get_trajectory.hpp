#include "qp_control.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Waypoint {
public:
    Waypoint() = default;  // 显式使用编译器默认生成的无参构造函数
    Waypoint(const VectorXd &position, const VectorXd &velocity, double time)
        : position_(position), velocity_(velocity), time_(time) {}

    VectorXd position_;
    VectorXd velocity_;
    double time_;

};

class GetTrajectory {
public:
    GetTrajectory(std::vector<Waypoint> waypoints, Waypoint startpoint, Waypoint endpoint)
        : waypoints_(std::move(waypoints)), startpoint_(std::move(startpoint)),
          endpoint_(std::move(endpoint)){
        // Calculate total time and number of waypoints
        num_waypoints_ = static_cast<int>(waypoints_.size());
        total_time_ = endpoint_.time_ - startpoint_.time_;
        total_steps_ = static_cast<int>(total_time_ / dt_) + 1; // 包括起点和终点
    }

    std::vector<Waypoint> generate();

private:
    std::vector<Waypoint> waypoints_;
    Waypoint startpoint_;
    Waypoint endpoint_;

    double total_time_;
    double dt_ = 0.001; // 1 ms (与仿真时间步一致)
    int num_waypoints_;
    int total_steps_;

};