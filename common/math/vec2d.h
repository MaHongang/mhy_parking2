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
 * @file vec2d.h
 * @brief 二维向量类的定义
 *
 * 本文件定义了 Vec2d 类，实现了二维平面向量的各种运算。
 * Vec2d 是整个规划系统中最基础的数学类，广泛用于：
 * - 点的位置表示
 * - 方向向量表示
 * - 几何计算（距离、角度、投影等）
 * - 坐标变换
 */

#pragma once

#include "absl/strings/str_cat.h"
#include <cmath>
#include <string>

/**
 * @namespace common::math
 * @brief 通用数学工具命名空间
 *
 * 包含基础的数学类和函数：
 * - Vec2d: 二维向量
 * - LineSegment2d: 二维线段
 * - Box2d: 二维矩形框
 * - Polygon2d: 二维多边形
 */
namespace common {
namespace math {

/**
 * @brief 数学精度常量
 *
 * 用于浮点数比较时的容差判断，避免浮点精度问题。
 * 当两个浮点数的差值小于 kMathEpsilon 时，认为它们相等。
 */
constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 * @brief 二维向量类
 *
 * 实现了二维平面向量的基本运算，包括：
 * - 向量加减乘除
 * - 向量长度、角度计算
 * - 点积（内积）和叉积
 * - 向量旋转和归一化
 * - 两点间距离计算
 *
 * 数学背景：
 * - 向量表示为 (x, y)，可以表示位置或方向
 * - 点积: a·b = |a||b|cos(θ)，用于计算投影和角度
 * - 叉积: a×b = |a||b|sin(θ)，用于计算面积和判断左右关系
 *
 * 使用示例：
 * @code
 *   Vec2d p1(1.0, 2.0);
 *   Vec2d p2(3.0, 4.0);
 *   double dist = p1.DistanceTo(p2);  // 计算两点距离
 *   double angle = (p2 - p1).Angle(); // 计算方向角
 * @endcode
 */
class Vec2d {
public:
  /**
   * @brief 带参数的构造函数
   * @param x x 坐标分量
   * @param y y 坐标分量
   */
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  /**
   * @brief 默认构造函数，创建零向量 (0, 0)
   */
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  /**
   * @brief 创建指定角度的单位向量
   * @param angle 角度值（弧度），相对于 x 轴正方向
   * @return 单位向量 (cos(angle), sin(angle))
   *
   * 用于从角度创建方向向量，常用于：
   * - 根据航向角创建朝向向量
   * - 根据转向角计算轮胎朝向
   */
  static Vec2d CreateUnitVec2d(const double angle);

  /**
   * @brief 获取 x 坐标分量
   * @return x 坐标
   */
  double x() const { return x_; }

  /**
   * @brief 获取 y 坐标分量
   * @return y 坐标
   */
  double y() const { return y_; }

  /**
   * @brief 设置 x 坐标分量
   * @param x 新的 x 坐标值
   */
  void set_x(const double x) { x_ = x; }

  /**
   * @brief 设置 y 坐标分量
   * @param y 新的 y 坐标值
   */
  void set_y(const double y) { y_ = y; }

  /**
   * @brief 计算向量长度（模）
   * @return 向量长度 sqrt(x² + y²)
   *
   * 使用 std::hypot 实现，避免溢出问题。
   */
  double Length() const;

  /**
   * @brief 计算向量长度的平方
   * @return 向量长度的平方 x² + y²
   *
   * 当只需要比较长度大小时，使用 LengthSquare 可以避免开方运算。
   */
  double LengthSquare() const;

  /**
   * @brief 计算向量与 x 轴正方向的夹角
   * @return 角度值（弧度），范围 [-π, π]
   *
   * 使用 atan2(y, x) 计算，处理了所有象限的情况。
   */
  double Angle() const;

  /**
   * @brief 将向量归一化为单位向量
   *
   * 将向量缩放为长度为 1 的单位向量，保持方向不变。
   * 如果向量长度小于 kMathEpsilon，则不进行操作（避免除零）。
   */
  void Normalize();

  /**
   * @brief 计算到另一个向量（点）的欧氏距离
   * @param other 另一个向量
   * @return 两点之间的距离
   */
  double DistanceTo(const Vec2d &other) const;

  /**
   * @brief 计算到另一个向量（点）的欧氏距离的平方
   * @param other 另一个向量
   * @return 两点之间距离的平方
   *
   * 用于距离比较时避免开方运算。
   */
  double DistanceSquareTo(const Vec2d &other) const;

  /**
   * @brief 计算与另一个向量的叉积（二维伪叉积）
   * @param other 另一个向量
   * @return 叉积结果 x * other.y - y * other.x
   *
   * 几何意义：
   * - 结果的绝对值等于两向量构成的平行四边形面积
   * - 结果的符号表示 other 在当前向量的哪一侧：
   *   - 正数：other 在当前向量的左侧（逆时针方向）
   *   - 负数：other 在当前向量的右侧（顺时针方向）
   *   - 零：两向量共线
   */
  double CrossProd(const Vec2d &other) const;

  /**
   * @brief 计算与另一个向量的点积（内积）
   * @param other 另一个向量
   * @return 点积结果 x * other.x + y * other.y
   *
   * 几何意义：
   * - a·b = |a||b|cos(θ)，其中 θ 是两向量夹角
   * - 用于计算投影：proj_b(a) = (a·b / |b|²) * b
   * - 判断垂直：a·b = 0 表示两向量垂直
   */
  double InnerProd(const Vec2d &other) const;

  /**
   * @brief 旋转向量（返回新向量）
   * @param angle 旋转角度（弧度），正值为逆时针旋转
   * @return 旋转后的新向量
   *
   * 旋转公式（二维旋转矩阵）：
   * x' = x*cos(θ) - y*sin(θ)
   * y' = x*sin(θ) + y*cos(θ)
   */
  Vec2d rotate(const double angle) const;

  /**
   * @brief 原地旋转向量
   * @param angle 旋转角度（弧度），正值为逆时针旋转
   *
   * 与 rotate() 不同，此方法直接修改当前向量。
   */
  void SelfRotate(const double angle);

  /**
   * @brief 向量加法运算符
   * @param other 另一个向量
   * @return 两向量之和
   */
  Vec2d operator+(const Vec2d &other) const;

  /**
   * @brief 向量减法运算符
   * @param other 另一个向量
   * @return 两向量之差
   */
  Vec2d operator-(const Vec2d &other) const;

  /**
   * @brief 向量标量乘法运算符
   * @param ratio 标量乘数
   * @return 缩放后的向量
   */
  Vec2d operator*(const double ratio) const;

  /**
   * @brief 向量标量除法运算符
   * @param ratio 标量除数
   * @return 缩放后的向量
   * @note ratio 不应为零或接近零
   */
  Vec2d operator/(const double ratio) const;

  /**
   * @brief 向量加法赋值运算符
   * @param other 另一个向量
   * @return 当前向量的引用
   */
  Vec2d &operator+=(const Vec2d &other);

  /**
   * @brief 向量减法赋值运算符
   * @param other 另一个向量
   * @return 当前向量的引用
   */
  Vec2d &operator-=(const Vec2d &other);

  /**
   * @brief 向量标量乘法赋值运算符
   * @param ratio 标量乘数
   * @return 当前向量的引用
   */
  Vec2d &operator*=(const double ratio);

  /**
   * @brief 向量标量除法赋值运算符
   * @param ratio 标量除数
   * @return 当前向量的引用
   */
  Vec2d &operator/=(const double ratio);

  /**
   * @brief 向量相等比较运算符
   * @param other 另一个向量
   * @return 如果两向量在容差范围内相等则返回 true
   *
   * 使用 kMathEpsilon 作为比较容差。
   */
  bool operator==(const Vec2d &other) const;

  /**
   * @brief 生成调试字符串
   * @return 格式化的向量信息
   */
  std::string DebugString() const;

protected:
  double x_ = 0.0;  ///< x 坐标分量
  double y_ = 0.0;  ///< y 坐标分量
};

/**
 * @brief 标量与向量的乘法（支持 ratio * vec 的写法）
 * @param ratio 标量乘数
 * @param vec 向量
 * @return 缩放后的向量
 */
Vec2d operator*(const double ratio, const Vec2d &vec);

} // namespace math
} // namespace common
