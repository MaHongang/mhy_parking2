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
 * @file linear_interpolation.cc
 * @brief 线性插值函数的实现
 *
 * 本文件实现了各种线性插值函数，是路径规划和轨迹跟踪的基础工具。
 */

#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include <cmath>

namespace common {
namespace math {

/**
 * @brief 球面线性插值（Spherical Linear Interpolation）
 * @param a0 起始角度 [rad]
 * @param t0 起始参数
 * @param a1 终止角度 [rad]
 * @param t1 终止参数
 * @param t 目标参数
 * @return 插值得到的角度 [rad]，范围 [-π, π]
 *
 * 实现原理：
 * 1. 首先将两个角度归一化到 [-π, π] 范围
 * 2. 计算角度差 d = a1 - a0
 * 3. 如果 |d| > π，则走"短路径"：
 *    - d > π 时，d = d - 2π（改为顺时针方向）
 *    - d < -π 时，d = d + 2π（改为逆时针方向）
 * 4. 计算插值比例 r = (t - t0) / (t1 - t0)
 * 5. 结果角度 a = a0 + d * r
 * 6. 归一化结果到 [-π, π]
 *
 * 示例：从 170° 到 -170° 的插值
 * - 普通线性插值会经过 0°（走 340°）
 * - slerp 会经过 ±180°（走 20°）
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  // 检查时间差是否过小
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    std::cout << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  
  // 归一化角度值到 [-π, π] 范围
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  
  // 计算角度差
  double d = a1_n - a0_n;
  
  // 确保走最短路径（角度差不超过 π）
  if (d > M_PI) {
    d = d - 2 * M_PI;  // 改为顺时针方向
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;  // 改为逆时针方向
  }

  // 计算插值比例
  const double r = (t - t0) / (t1 - t0);
  
  // 线性插值角度
  const double a = a0_n + d * r;
  
  // 归一化结果
  return NormalizeAngle(a);
}

/**
 * @brief SLPoint 的线性插值
 * @param p0 起始 SL 点
 * @param p1 终止 SL 点
 * @param w 插值权重，范围 [0, 1]
 * @return 插值得到的 SL 点
 *
 * SL 坐标系是 Frenet 坐标系的一种表示：
 * - s: 沿参考线的弧长
 * - l: 相对参考线的横向偏移
 *
 * 插值公式：p = (1-w) * p0 + w * p1
 */
SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  // CHECK_GE(w, 0.0);

  SLPoint p;
  p.set_s((1 - w) * p0.s() + w * p1.s());
  p.set_l((1 - w) * p0.l() + w * p1.l());
  return p;
}

/**
 * @brief PathPoint 的线性插值（基于弧长 s）
 * @param p0 起始路径点
 * @param p1 终止路径点
 * @param s 目标弧长值
 * @return 插值得到的路径点
 *
 * 插值内容：
 * - (x, y): 位置坐标，线性插值
 * - theta: 航向角，使用 slerp 球面插值（处理角度跳变）
 * - kappa: 曲率，线性插值
 * - dkappa: 曲率变化率，线性插值
 * - ddkappa: 曲率二阶变化率，线性插值
 * - s: 直接设置为目标值
 *
 * 注意：权重 weight = (s - s0) / (s1 - s0)
 */
PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();

  PathPoint path_point;
  
  // 计算插值权重
  double weight = (s - s0) / (s1 - s0);
  
  // 位置插值：p = p0 + weight * (p1 - p0) = (1-weight)*p0 + weight*p1
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  
  // 航向角使用球面插值（处理角度的周期性）
  double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  
  // 曲率及其导数使用线性插值
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  
  // 设置插值结果
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  
  return path_point;
}

/**
 * @brief TrajectoryPoint 的线性插值（基于相对时间 t）
 * @param tp0 起始轨迹点
 * @param tp1 终止轨迹点
 * @param t 目标相对时间
 * @return 插值得到的轨迹点
 *
 * 插值内容：
 * - v: 速度，线性插值
 * - a: 加速度，线性插值
 * - steer: 方向盘转角，使用 slerp 球面插值
 * - path_point 中的所有属性，基于时间 t 进行插值
 *
 * 这是轨迹跟踪中最核心的插值函数，用于从离散的规划轨迹中
 * 获取任意时刻的目标状态（位置、速度、加速度等）。
 */
TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t) {
  // if (!tp0.has_path_point() || !tp1.has_path_point()) {
  //   TrajectoryPoint p;
  //   p.mutable_path_point()->CopyFrom(PathPoint());
  //   return p;
  // }
  
  // 提取路径点和时间
  const PathPoint pp0 = tp0.path_point();
  const PathPoint pp1 = tp1.path_point();
  double t0 = tp0.relative_time();
  double t1 = tp1.relative_time();

  TrajectoryPoint tp;
  
  // 速度和加速度的线性插值
  tp.set_v(lerp(tp0.v(), t0, tp1.v(), t1, t));
  tp.set_a(lerp(tp0.a(), t0, tp1.a(), t1, t));
  tp.set_relative_time(t);
  
  // 方向盘转角使用球面插值（处理角度跳变）
  tp.set_steer(slerp(tp0.steer(), t0, tp1.steer(), t1, t));

  // 路径点属性的插值
  PathPoint *path_point = tp.mutable_path_point();
  path_point->set_x(lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_theta(slerp(pp0.theta(), t0, pp1.theta(), t1, t));  // 航向角用球面插值
  path_point->set_kappa(lerp(pp0.kappa(), t0, pp1.kappa(), t1, t));
  path_point->set_dkappa(lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));
  path_point->set_s(lerp(pp0.s(), t0, pp1.s(), t1, t));

  return tp;
}

} // namespace math
} // namespace common
