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
 * @file discretized_trajectory.h
 * @brief 离散化轨迹类的定义
 *
 * 本文件定义了 DiscretizedTrajectory 类，用于表示离散化的完整轨迹。
 * 与 DiscretizedPath 不同，轨迹不仅包含空间信息，还包含时间信息：
 * - 每个 TrajectoryPoint 包含 PathPoint（位置、航向、曲率）和时间相关信息
 * - 支持基于相对时间的轨迹点插值查询
 * - 支持基于位置的最近点查询
 *
 * 轨迹是规划系统的最终输出，包含车辆在未来一段时间内的完整运动信息。
 **/

#pragma once

#include "common/math/vec2d.h"
#include "common/pnc_point.h"
#include <algorithm>
#include <iostream>
#include <vector>

/**
 * @class DiscretizedTrajectory
 * @brief 离散化轨迹类，存储和管理完整的规划轨迹
 *
 * 该类继承自 std::vector<TrajectoryPoint>，提供了对轨迹的高级操作：
 * - 基于相对时间的轨迹点插值查询
 * - 基于位置的最近点查询（用于轨迹跟踪）
 * - 轨迹的时间和空间长度计算
 * - 轨迹点的追加和预置操作
 *
 * 设计说明：
 * - 轨迹点按相对时间（relative_time）升序排列
 * - 相对时间从 0 开始，表示从规划起点经过的时间
 * - 使用二分查找实现 O(log n) 的快速查询
 */
class DiscretizedTrajectory : public std::vector<common::TrajectoryPoint> {
public:
  /**
   * @brief 默认构造函数，创建空轨迹
   */
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  // explicit DiscretizedTrajectory(const ADCTrajectory &trajectory);

  /**
   * @brief 从轨迹点向量构造离散化轨迹
   * @param trajectory_points 轨迹点序列，应按相对时间升序排列
   */
  explicit DiscretizedTrajectory(
      const std::vector<common::TrajectoryPoint> &trajectory_points);

  /**
   * @brief 设置轨迹点序列（替换现有数据）
   * @param trajectory_points 新的轨迹点序列
   */
  void SetTrajectoryPoints(
      const std::vector<common::TrajectoryPoint> &trajectory_points);

  /**
   * @brief 虚析构函数
   */
  virtual ~DiscretizedTrajectory() = default;

  /**
   * @brief 获取轨迹起点
   * @return 第一个轨迹点
   */
  virtual common::TrajectoryPoint StartPoint() const;

  /**
   * @brief 获取轨迹的时间长度
   * @return 时间跨度 = 末点相对时间 - 首点相对时间 [s]
   */
  virtual double GetTemporalLength() const;

  /**
   * @brief 获取轨迹的空间长度
   * @return 空间长度 = 末点弧长 - 首点弧长 [m]
   */
  virtual double GetSpatialLength() const;

  /**
   * @brief 基于相对时间查询插值轨迹点
   * @param relative_time 查询的相对时间 [s]
   * @return 插值得到的轨迹点
   *
   * 使用线性插值在相邻轨迹点之间估计任意时刻的轨迹点。
   * 如果时间超出范围，返回边界点。
   */
  virtual common::TrajectoryPoint Evaluate(const double relative_time) const;

  /**
   * @brief 查找相对时间对应的下界索引
   * @param relative_time 查询的相对时间 [s]
   * @param epsilon 时间容差，默认 1e-5
   * @return 满足条件的轨迹点索引
   *
   * 返回第一个 relative_time + epsilon >= 查询时间的点的索引。
   * 用于轨迹跟踪时定位当前应该跟踪的轨迹段。
   */
  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1.0e-5) const;

  /**
   * @brief 查找距离给定位置最近的轨迹点索引
   * @param position 查询位置 (x, y)
   * @return 最近轨迹点的索引
   *
   * 遍历所有轨迹点，计算欧氏距离的平方，返回最小值对应的索引。
   * 时间复杂度 O(n)。
   */
  virtual size_t QueryNearestPoint(const common::math::Vec2d &position) const;

  /**
   * @brief 带缓冲区的最近点查询
   * @param position 查询位置 (x, y)
   * @param buffer 距离缓冲区
   * @return 最近轨迹点的索引
   *
   * 与 QueryNearestPoint 类似，但在比较时增加了缓冲区容差。
   * 这可以在距离相近时优先选择后面的点。
   */
  size_t QueryNearestPointWithBuffer(const common::math::Vec2d &position,
                                     const double buffer) const;

  /**
   * @brief 追加一个轨迹点到末尾
   * @param trajectory_point 要追加的轨迹点
   * @note 新点的相对时间应大于当前最后一个点的相对时间
   */
  virtual void
  AppendTrajectoryPoint(const common::TrajectoryPoint &trajectory_point);

  /**
   * @brief 在轨迹开头预置一组轨迹点
   * @param trajectory_points 要预置的轨迹点序列
   *
   * 用于将历史轨迹点添加到当前规划轨迹之前。
   * 预置的点的相对时间应小于当前首点的相对时间。
   */
  void PrependTrajectoryPoints(
      const std::vector<common::TrajectoryPoint> &trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      //    ACHECK(trajectory_points.back().relative_time() <
      //           front().relative_time());
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  /**
   * @brief 按索引获取轨迹点
   * @param index 轨迹点索引
   * @return 对应的轨迹点引用
   */
  const common::TrajectoryPoint &TrajectoryPointAt(const size_t index) const;

  /**
   * @brief 获取轨迹点数量
   * @return 轨迹点总数
   */
  size_t NumOfPoints() const;

  /**
   * @brief 清空轨迹
   */
  virtual void Clear();
};

/**
 * @brief 内联实现：获取轨迹点数量
 */
inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

/**
 * @brief 内联实现：清空轨迹
 */
inline void DiscretizedTrajectory::Clear() { clear(); }