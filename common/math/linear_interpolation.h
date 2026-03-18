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
 * @file linear_interpolation.h
 * @brief 线性插值函数的定义
 *
 * 本文件定义了各种线性插值函数，用于在两个已知点之间估计中间值。
 * 线性插值是路径规划和轨迹跟踪中最常用的插值方法之一：
 * - 简单高效，计算开销小
 * - 结果连续（但不一定平滑）
 * - 适用于采样密度较高的离散数据
 *
 * 主要函数：
 * - lerp: 通用线性插值模板
 * - slerp: 球面线性插值（用于角度插值）
 * - InterpolateUsingLinearApproximation: 针对特定点类型的插值
 */

#pragma once
#include <cmath>
#include <iostream>

#include "common/pnc_point.h"

/**
 * @namespace common::math
 * @brief 通用数学工具命名空间
 */
namespace common {
namespace math {

/**
 * @brief 通用线性插值函数模板
 * @tparam T 插值数据类型（需支持加法和标量乘法）
 * @param x0 第一个点的值
 * @param t0 第一个点的参数
 * @param x1 第二个点的值
 * @param t1 第二个点的参数
 * @param t 目标参数
 * @return 插值结果
 *
 * 线性插值公式：
 * x = x0 + (t - t0) / (t1 - t0) * (x1 - x0)
 *
 * 其中 r = (t - t0) / (t1 - t0) 是归一化的插值比例：
 * - t = t0 时，r = 0，返回 x0
 * - t = t1 时，r = 1，返回 x1
 * - t 在 (t0, t1) 之间时，返回线性组合
 *
 * 使用示例：
 * @code
 *   double v = lerp(0.0, 0.0, 10.0, 1.0, 0.5);  // 返回 5.0
 * @endcode
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  // 检查时间差是否过小，避免除零
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cout << "input time difference is too small";
    return x0;
  }
  // 计算插值比例
  const double r = (t - t0) / (t1 - t0);
  // 线性插值
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief 球面线性插值（用于角度）
 * @param a0 第一个角度值 [rad]，范围 [-π, π)
 * @param t0 第一个角度的参数
 * @param a1 第二个角度值 [rad]，范围 [-π, π)
 * @param t1 第二个角度的参数
 * @param t 目标参数
 * @return 插值得到的角度值 [rad]
 *
 * 与普通线性插值不同，slerp 考虑了角度的周期性。
 * 例如：从 170° 插值到 -170° 时，应该经过 180° 而不是 0°。
 *
 * 实现方式：
 * 1. 计算两角度的差值
 * 2. 将差值归一化到 [-π, π] 范围
 * 3. 对归一化后的差值进行线性插值
 * 4. 将结果归一化到 [-π, π] 范围
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

/**
 * @brief SLPoint 的线性插值
 * @param p0 第一个 SL 点
 * @param p1 第二个 SL 点
 * @param w 插值权重，w=0 返回 p0，w=1 返回 p1
 * @return 插值得到的 SL 点
 *
 * SLPoint 包含 (s, l) 两个分量，分别进行线性插值。
 */
SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w);

/**
 * @brief PathPoint 的线性插值
 * @param p0 第一个路径点
 * @param p1 第二个路径点
 * @param s 目标弧长值（作为插值参数）
 * @return 插值得到的路径点
 *
 * PathPoint 的插值内容：
 * - (x, y): 位置的线性插值
 * - theta: 航向角的球面插值（处理角度跳变）
 * - kappa: 曲率的线性插值
 * - s: 设置为目标弧长 s
 *
 * 注意：dkappa（曲率变化率）通常不进行插值，
 * 因为它是二阶导数，线性插值会引入较大误差。
 */
PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s);

/**
 * @brief TrajectoryPoint 的线性插值
 * @param tp0 第一个轨迹点
 * @param tp1 第二个轨迹点
 * @param t 目标相对时间（作为插值参数）
 * @return 插值得到的轨迹点
 *
 * TrajectoryPoint 的插值内容：
 * - path_point: 调用 PathPoint 的插值函数
 * - v: 速度的线性插值
 * - a: 加速度的线性插值
 * - relative_time: 设置为目标时间 t
 *
 * 这是轨迹跟踪中最常用的插值函数，
 * 用于从离散轨迹中获取任意时刻的目标状态。
 */
TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t);

} // namespace math
} // namespace common
