/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file point_factory.h
 * @brief 点工厂类的定义
 *
 * 本文件定义了 PointFactory 工厂类，提供各种点类型的便捷创建方法。
 * 使用工厂模式的优点：
 * - 统一的点创建接口
 * - 减少重复代码
 * - 提供默认参数值
 * - 便于类型转换
 */

#pragma once

#include "common/math/vec2d.h"
//#include "common/proto/geometry.pb.h"
#include "common/pnc_point.h"

namespace common {
namespace util {

/**
 * @class PointFactory
 * @brief 点类型的工厂类
 *
 * 提供静态方法来创建各种点类型的实例：
 * - Vec2d: 二维向量
 * - SLPoint: Frenet 坐标系下的 SL 点
 * - SpeedPoint: 速度曲线上的点
 * - PathPoint: 路径上的点
 *
 * 使用示例：
 * @code
 *   // 创建 SL 点
 *   auto sl = PointFactory::ToSLPoint(10.0, 0.5);
 *
 *   // 创建速度点
 *   auto sp = PointFactory::ToSpeedPoint(10.0, 1.0, 5.0, 0.0, 0.0);
 *
 *   // 从任意 XY 类型转换为 Vec2d
 *   auto vec = PointFactory::ToVec2d(path_point);
 * @endcode
 */
class PointFactory {
public:
  /**
   * @brief 将任意具有 x(), y() 方法的类型转换为 Vec2d
   * @tparam XY 源类型，必须有 x() 和 y() 成员函数
   * @param xy 源对象
   * @return 对应的 Vec2d 对象
   *
   * 这是一个模板方法，可以接受任何具有 x(), y() 接口的类型，
   * 包括 PathPoint, TrajectoryPoint 等。
   */
  template <typename XY> static inline math::Vec2d ToVec2d(const XY &xy) {
    return math::Vec2d(xy.x(), xy.y());
  }

  /**
   * @brief 创建 SLPoint（Frenet 坐标系下的点）
   * @param s 弧长坐标 [m]，沿参考线的距离
   * @param l 横向偏移 [m]，相对参考线的左正右负
   * @return 创建的 SLPoint 对象
   *
   * SL 坐标系说明：
   * - s: Station，沿道路中心线/参考线的累积距离
   * - l: Lateral，相对于参考线的横向偏移，左侧为正，右侧为负
   */
  static inline SLPoint ToSLPoint(const double s, const double l) {
    SLPoint sl;
    sl.set_s(s);
    sl.set_l(l);
    return sl;
  }

  // static inline PointENU ToPointENU(const double x, const double y,
  //                                   const double z = 0) {
  //   PointENU point_enu;
  //   point_enu.set_x(x);
  //   point_enu.set_y(y);
  //   point_enu.set_z(z);
  //   return point_enu;
  // }

  // template <typename XYZ> static inline PointENU ToPointENU(const XYZ &xyz) {
  //   return ToPointENU(xyz.x(), xyz.y(), xyz.z());
  // }

  /**
   * @brief 创建 SpeedPoint（速度曲线上的点）
   * @param s 弧长位置 [m]
   * @param t 时间戳 [s]
   * @param v 速度 [m/s]，默认 0
   * @param a 加速度 [m/s²]，默认 0
   * @param da 加加速度/jerk [m/s³]，默认 0
   * @return 创建的 SpeedPoint 对象
   *
   * SpeedPoint 用于表示速度规划的结果，
   * 描述了在某一时刻的运动状态（位置、速度、加速度）。
   */
  static inline SpeedPoint ToSpeedPoint(const double s, const double t,
                                        const double v = 0, const double a = 0,
                                        const double da = 0) {
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    return speed_point;
  }

  /**
   * @brief 创建 PathPoint（路径上的点）
   * @param x x 坐标 [m]
   * @param y y 坐标 [m]
   * @param z z 坐标（高程）[m]，默认 0
   * @param s 弧长 [m]，默认 0
   * @param theta 航向角 [rad]，默认 0
   * @param kappa 曲率 [1/m]，默认 0
   * @param dkappa 曲率变化率 [1/m²]，默认 0
   * @param ddkappa 曲率二阶变化率 [1/m³]，默认 0
   * @return 创建的 PathPoint 对象
   *
   * PathPoint 是路径规划的基本单元，包含：
   * - 位置信息 (x, y, z, s)
   * - 几何信息 (theta, kappa, dkappa, ddkappa)
   */
  static inline PathPoint ToPathPoint(const double x, const double y,
                                      const double z = 0, const double s = 0,
                                      const double theta = 0,
                                      const double kappa = 0,
                                      const double dkappa = 0,
                                      const double ddkappa = 0) {
    PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_z(z);
    path_point.set_s(s);
    path_point.set_theta(theta);
    path_point.set_kappa(kappa);
    path_point.set_dkappa(dkappa);
    path_point.set_ddkappa(ddkappa);
    return path_point;
  }
};

} // namespace util
} // namespace common
