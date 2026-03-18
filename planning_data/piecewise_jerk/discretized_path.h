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
 * @file discretized_path.h
 * @brief 离散化路径类的定义
 *
 * 本文件定义了 DiscretizedPath 类，用于表示离散化的路径。
 * 离散化路径由一系列按弧长 s 排序的 PathPoint 组成，支持：
 * - 路径长度计算
 * - 基于弧长的路径点插值查询
 * - 正向/反向插值求值
 *
 * 设计模式：
 * - 继承自 std::vector<PathPoint>，可直接使用 STL 容器操作
 * - 使用二分查找实现 O(log n) 的快速查询
 * - 线性插值保证平滑的路径点估计
 **/

#pragma once

#include <utility>
#include <vector>

#include "common/pnc_point.h"

/**
 * @class DiscretizedPath
 * @brief 离散化路径类，存储和管理一系列路径点
 *
 * 该类继承自 std::vector<PathPoint>，提供了对路径点序列的高级操作：
 * - 支持基于弧长 s 的路径点插值查询
 * - 内部假设路径点按弧长 s 升序排列
 * - 使用线性插值在相邻点之间估计任意 s 值对应的路径点
 *
 * 使用场景：
 * - 存储规划器生成的路径
 * - 轨迹跟踪时的路径点查询
 * - 路径平滑和重采样
 */
class DiscretizedPath : public std::vector<common::PathPoint> {
public:
  /**
   * @brief 默认构造函数，创建空路径
   */
  DiscretizedPath() = default;

  /**
   * @brief 从路径点向量构造离散化路径
   * @param path_points 路径点序列，应按弧长 s 升序排列
   */
  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  /**
   * @brief 获取路径总长度
   * @return 路径总长度（最后一个点的 s 减去第一个点的 s）
   * @note 如果路径为空，返回 0.0
   */
  double Length() const;

  /**
   * @brief 基于弧长 s 正向查询路径点（使用 lower_bound）
   * @param path_s 查询的弧长值
   * @return 插值得到的路径点
   * @note 如果 path_s 超出范围，返回边界点
   *
   * 工作原理：
   * 1. 使用二分查找找到第一个 s >= path_s 的点
   * 2. 在该点与前一个点之间进行线性插值
   */
  common::PathPoint Evaluate(const double path_s) const;

  /**
   * @brief 基于弧长 s 反向查询路径点（使用 upper_bound）
   * @param path_s 查询的弧长值
   * @return 插值得到的路径点
   * @note 如果 path_s 超出范围，返回边界点
   *
   * 与 Evaluate 的区别：
   * - Evaluate 使用 lower_bound，适用于正向遍历
   * - EvaluateReverse 使用 upper_bound，适用于反向遍历
   */
  common::PathPoint EvaluateReverse(const double path_s) const;

protected:
  /**
   * @brief 查找第一个 s >= path_s 的路径点迭代器
   * @param path_s 目标弧长值
   * @return 指向满足条件的第一个元素的迭代器
   * @note 使用 std::lower_bound 实现 O(log n) 复杂度
   */
  std::vector<common::PathPoint>::const_iterator
  QueryLowerBound(const double path_s) const;

  /**
   * @brief 查找第一个 s > path_s 的路径点迭代器
   * @param path_s 目标弧长值
   * @return 指向满足条件的第一个元素的迭代器
   * @note 使用 std::upper_bound 实现 O(log n) 复杂度
   */
  std::vector<common::PathPoint>::const_iterator
  QueryUpperBound(const double path_s) const;
};
