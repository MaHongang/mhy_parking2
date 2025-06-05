#pragma once
#include <vector>
#include "common/pnc_point.h" //  TrajectoryPoint 定义在此

class DiscretizedTrajectoryComposition {
public:
    // 添加轨迹点
    void AddPoint(const common::TrajectoryPoint& point) {
        points_.push_back(point);
    }

    // 获取轨迹点（支持只读访问）
    // const common::TrajectoryPoint& GetPoint(size_t index) const {
    //     return points_.at(index);
    // }

    // 获取轨迹点数量
    // size_t Size() const {
    //     return points_.size();
    // }

    // // 支持迭代器（可选，按需实现）
    // auto begin() const { return points_.begin(); }
    // auto end() const { return points_.end(); }

    // // 其他轨迹特定方法，如插值、平滑等
    // void SmoothTrajectory();
    std::vector<common::TrajectoryPoint> GetDiscretizedTrajectory() const
    {
        return points_;
    }

private:
    std::vector<common::TrajectoryPoint> points_; // 组合而非继承
};