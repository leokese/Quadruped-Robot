#include "object_qp/get_trajectory.hpp"

std::vector<Waypoint> GetTrajectory::generate() {
    std::vector<Waypoint> trajectory;
    trajectory.resize(total_steps_);

    // 构建完整的关键点序列（含起点、中间点、终点）
    std::vector<Waypoint> all_points;
    all_points.reserve(num_waypoints_ + 2);
    all_points.push_back(startpoint_);
    all_points.insert(all_points.end(), waypoints_.begin(), waypoints_.end());
    all_points.push_back(endpoint_);

    // 先计算每个关键点对应的step编号（time / dt）
    std::vector<int> step_indices(all_points.size());
    for (size_t i = 0; i < all_points.size(); ++i) {
        step_indices[i] = static_cast<int>((all_points[i].time_ - startpoint_.time_) / dt_);
    }

    // 检查 step_indices 是否严格递增（保证时间顺序）
    for (size_t i = 1; i < step_indices.size(); ++i) {
        if (step_indices[i] <= step_indices[i - 1]) {
            std::cerr << "Error: time stamps of waypoints are not strictly increasing." << std::endl;
            return {};
        }
    }

    // 对所有时刻进行速度插值
    for (size_t seg = 0; seg < all_points.size() - 1; ++seg) {
        int start_idx = step_indices[seg];
        int end_idx = step_indices[seg + 1];

        const VectorXd& v_start = all_points[seg].velocity_;
        const VectorXd& v_end = all_points[seg + 1].velocity_;
        int steps = end_idx - start_idx;

        for (int k = 0; k < steps; ++k) {
            double alpha = static_cast<double>(k) / steps;
            VectorXd v_interpolated = (1.0 - alpha) * v_start + alpha * v_end;
            int idx = start_idx + k;
            trajectory[idx].velocity_ = v_interpolated;
            trajectory[idx].time_ = startpoint_.time_ + idx * dt_;
        }
    }

    // 终点速度赋值
    trajectory[step_indices.back()].velocity_ = all_points.back().velocity_;
    trajectory[step_indices.back()].time_ = all_points.back().time_;

    // 位置积分计算，初始位置为 startpoint_ 位置
    trajectory[0].position_ = startpoint_.position_;
    trajectory[0].time_ = startpoint_.time_;

    for (int i = 1; i < total_steps_; ++i) {
        trajectory[i].position_ = trajectory[i - 1].position_ + trajectory[i - 1].velocity_ * dt_;
        trajectory[i].time_ = startpoint_.time_ + i * dt_;
    }

    return trajectory;
}
