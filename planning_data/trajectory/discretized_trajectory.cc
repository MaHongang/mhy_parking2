/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file discretized_trajectory.cc
 * @brief 离散化轨迹类的实现
 *
 * 本文件实现了 DiscretizedTrajectory 类的所有成员函数，
 * 包括轨迹点的插值查询、最近点查询、轨迹统计等功能。
 **/

#include "planning_data/trajectory/discretized_trajectory.h"

#include <limits>

#include "common/math/linear_interpolation.h"
//#include "common/planning_context.h"

/**
 * @brief 从轨迹点向量构造离散化轨迹
 * @param trajectory_points 轨迹点序列
 */
DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<common::TrajectoryPoint> &trajectory_points)
    : std::vector<common::TrajectoryPoint>(trajectory_points) {}
// //ACHECK(!trajectory_points.empty())
//<< "trajectory_points should NOT be empty()";

// DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory &trajectory)
// {
//   assign(trajectory.trajectory_point().begin(),
//          trajectory.trajectory_point().end());
// }

using common::TrajectoryPoint;

/**
 * @brief 基于相对时间查询插值轨迹点
 * @param relative_time 查询的相对时间 [s]
 * @return 插值得到的轨迹点
 *
 * 实现逻辑：
 * 1. 使用二分查找找到第一个 relative_time >= 查询时间的点
 * 2. 如果在边界，返回边界点
 * 3. 否则在相邻两点之间进行线性插值
 *
 * 插值内容包括：位置、航向、曲率、速度、加速度等所有轨迹点属性。
 */
TrajectoryPoint
DiscretizedTrajectory::Evaluate(const double relative_time) const {
  // 比较函数：查找第一个 relative_time >= 目标时间的点
  auto comp = [](const TrajectoryPoint &p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    // 查询时间小于首点时间，返回首点
    return front();
  } else if (it_lower == end()) {
    // 查询时间大于末点时间，输出警告并返回末点
    std::cout << "When evaluate trajectory, relative_time(" << relative_time
              << ") is too large";
    return back();
  }
  // 在相邻两点之间进行线性插值
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

/**
 * @brief 查找相对时间对应的下界索引
 * @param relative_time 查询的相对时间 [s]
 * @param epsilon 时间容差，用于处理浮点精度问题
 * @return 满足条件的轨迹点索引
 *
 * 返回第一个满足 (tp.relative_time + epsilon >= relative_time) 的点的索引。
 * 如果查询时间超过末点，返回末点索引。
 */
size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  // ACHECK(!empty());

  // 如果查询时间超过末点，返回末点索引
  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  
  // 带容差的比较函数
  auto func = [&epsilon](const TrajectoryPoint &tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

/**
 * @brief 查找距离给定位置最近的轨迹点
 * @param position 查询位置 (x, y)
 * @return 最近轨迹点的索引
 *
 * 遍历所有轨迹点，计算欧氏距离的平方（避免开方运算），
 * 返回距离最小的点的索引。
 *
 * 时间复杂度：O(n)
 * 
 * 优化建议：对于长轨迹，可以考虑使用 KD-tree 加速查询
 */
size_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d &position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();  // 最小距离平方
  size_t index_min = 0;  // 最近点索引
  
  for (size_t i = 0; i < size(); ++i) {
    // 提取当前轨迹点的位置
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    // 计算距离平方（避免开方）
    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

/**
 * @brief 带缓冲区的最近点查询
 * @param position 查询位置 (x, y)
 * @param buffer 距离缓冲区
 * @return 最近轨迹点的索引
 *
 * 与 QueryNearestPoint 类似，但在距离比较时增加了缓冲区。
 * 当新点的距离在 (dist_sqr_min + buffer) 范围内时，也会更新。
 * 
 * 这种设计可以在距离相近时优先选择轨迹上更靠后的点，
 * 有助于避免轨迹跟踪时的回退现象。
 */
size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d &position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                         data()[i].path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    // 带缓冲区的比较：允许稍远的点被选中
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

/**
 * @brief 追加轨迹点到末尾
 * @param trajectory_point 要追加的轨迹点
 *
 * 新点的相对时间应大于当前最后一个点的相对时间，
 * 以保证轨迹的时间单调性。
 */
void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint &trajectory_point) {
  if (!empty()) {
    //   CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

/**
 * @brief 按索引获取轨迹点
 * @param index 轨迹点索引
 * @return 对应的轨迹点常量引用
 */
const TrajectoryPoint &
DiscretizedTrajectory::TrajectoryPointAt(const size_t index) const {
  // CHECK_LT(index, NumOfPoints());
  return data()[index];
}

/**
 * @brief 获取轨迹起点
 * @return 第一个轨迹点
 */
TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  // ACHECK(!empty());
  return front();
}

/**
 * @brief 获取轨迹的时间长度
 * @return 时间跨度 [s]
 *
 * 时间长度 = 末点相对时间 - 首点相对时间
 * 通常首点的相对时间为 0，所以时间长度就是末点的相对时间。
 */
double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

/**
 * @brief 获取轨迹的空间长度
 * @return 空间长度 [m]
 *
 * 空间长度 = 末点弧长 - 首点弧长
 * 表示轨迹在空间上覆盖的总距离。
 */
double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}
