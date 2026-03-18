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
 * @file discretized_path.cc
 * @brief 离散化路径类的实现
 *
 * 本文件实现了 DiscretizedPath 类的所有成员函数，
 * 包括路径长度计算、基于弧长的插值查询等功能。
 **/

#include "planning_data/path/discretized_path.h"

#include <algorithm>

#include "common/math/linear_interpolation.h"

using common::PathPoint;

/**
 * @brief 从路径点向量构造离散化路径
 * @param path_points 路径点序列（使用移动语义避免拷贝）
 */
DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {}

/**
 * @brief 计算路径总长度
 * @return 路径长度 = 末点弧长 - 首点弧长
 *
 * 路径长度定义为从起点到终点的累积弧长。
 * 假设路径点已按弧长升序排列。
 */
double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

/**
 * @brief 基于弧长正向查询插值路径点
 * @param path_s 目标弧长值
 * @return 插值计算得到的路径点
 *
 * 实现逻辑：
 * 1. 使用 lower_bound 找到第一个 s >= path_s 的点
 * 2. 如果 path_s 小于起点，返回起点
 * 3. 如果 path_s 大于终点，返回终点
 * 4. 否则在相邻两点之间进行线性插值
 *
 * 时间复杂度：O(log n)，其中 n 为路径点数量
 */
PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  //  ACHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    // path_s 小于等于第一个点的 s，返回起点
    return front();
  }
  if (it_lower == end()) {
    // path_s 大于所有点的 s，返回终点
    return back();
  }
  // 在 (it_lower - 1) 和 it_lower 之间进行线性插值
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

/**
 * @brief 查找第一个 s >= path_s 的路径点
 * @param path_s 目标弧长值
 * @return 满足条件的第一个元素的迭代器
 *
 * 使用 std::lower_bound 实现二分查找，
 * 比较函数：tp.s() < path_s
 */
std::vector<PathPoint>::const_iterator
DiscretizedPath::QueryLowerBound(const double path_s) const {
  auto func = [](const PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

/**
 * @brief 基于弧长反向查询插值路径点
 * @param path_s 目标弧长值
 * @return 插值计算得到的路径点
 *
 * 与 Evaluate 类似，但使用 upper_bound 进行查询。
 * upper_bound 返回第一个 s > path_s 的点，
 * 适用于反向遍历或需要不同边界行为的场景。
 */
PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  //  ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                           *it_upper, path_s);
}

/**
 * @brief 查找第一个 s > path_s 的路径点
 * @param path_s 目标弧长值
 * @return 满足条件的第一个元素的迭代器
 *
 * 使用 std::upper_bound 实现二分查找。
 * 注意：upper_bound 的比较函数参数顺序与 lower_bound 相反，
 * 比较函数：path_s < tp.s()（但这里写成 tp.s() < path_s）
 */
std::vector<PathPoint>::const_iterator
DiscretizedPath::QueryUpperBound(const double path_s) const {
  auto func = [](const double path_s, const PathPoint &tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}
