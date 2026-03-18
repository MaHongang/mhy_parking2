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
 * @file vec2d.cc
 * @brief 二维向量类的实现
 *
 * 本文件实现了 Vec2d 类的所有成员函数。
 */

#include "common/math/vec2d.h"
#include <cmath>
#include "absl/strings/str_cat.h"

namespace common
{
  namespace math
  {

    /**
     * @brief 创建指定角度的单位向量
     * @param angle 角度值（弧度）
     * @return 单位向量 (cos(angle), sin(angle))
     */
    Vec2d Vec2d::CreateUnitVec2d(const double angle)
    {
      return Vec2d(std::cos(angle), std::sin(angle));
    }

    /**
     * @brief 计算向量长度
     * @return 向量模长
     *
     * 使用 std::hypot 计算 sqrt(x² + y²)，
     * hypot 函数内部处理了溢出问题，比直接计算更安全。
     */
    double Vec2d::Length() const { return std::hypot(x_, y_); }

    /**
     * @brief 计算向量长度的平方
     * @return x² + y²
     */
    double Vec2d::LengthSquare() const { return x_ * x_ + y_ * y_; }

    /**
     * @brief 计算向量与 x 轴正方向的夹角
     * @return 角度值（弧度），范围 [-π, π]
     *
     * atan2(y, x) 能正确处理所有象限，
     * 并且当 x=0 时也能正确返回 ±π/2。
     */
    double Vec2d::Angle() const { return std::atan2(y_, x_); }

    /**
     * @brief 将向量归一化为单位向量
     *
     * 归一化后向量长度为 1，方向保持不变。
     * 当向量长度过小时，跳过归一化以避免数值不稳定。
     */
    void Vec2d::Normalize()
    {
      const double l = Length();
      if (l > kMathEpsilon)
      {
        x_ /= l;
        y_ /= l;
      }
    }

    /**
     * @brief 计算到另一点的欧氏距离
     * @param other 另一个点
     * @return 两点间距离
     */
    double Vec2d::DistanceTo(const Vec2d &other) const
    {
      return std::hypot(x_ - other.x_, y_ - other.y_);
    }

    /**
     * @brief 计算到另一点的欧氏距离的平方
     * @param other 另一个点
     * @return 两点间距离的平方
     *
     * 避免开方运算，用于距离比较。
     */
    double Vec2d::DistanceSquareTo(const Vec2d &other) const
    {
      const double dx = x_ - other.x_;
      const double dy = y_ - other.y_;
      return dx * dx + dy * dy;
    }

    /**
     * @brief 计算二维叉积
     * @param other 另一个向量
     * @return 叉积结果 x * other.y - y * other.x
     *
     * 叉积的几何意义：
     * - 结果等于两向量张成的平行四边形的有向面积
     * - 正值表示 other 在 this 的逆时针方向
     */
    double Vec2d::CrossProd(const Vec2d &other) const
    {
      return x_ * other.y() - y_ * other.x();
    }

    /**
     * @brief 计算点积（内积）
     * @param other 另一个向量
     * @return 点积结果 x * other.x + y * other.y
     *
     * 点积的几何意义：
     * - a·b = |a||b|cos(θ)
     * - 用于计算投影和判断夹角
     */
    double Vec2d::InnerProd(const Vec2d &other) const
    {
      return x_ * other.x() + y_ * other.y();
    }

    /**
     * @brief 旋转向量（返回新向量）
     * @param angle 旋转角度（弧度），正值为逆时针
     * @return 旋转后的新向量
     *
     * 二维旋转变换公式：
     * | x' |   | cos(θ)  -sin(θ) | | x |
     * | y' | = | sin(θ)   cos(θ) | | y |
     */
    Vec2d Vec2d::rotate(const double angle) const
    {
      return Vec2d(x_ * cos(angle) - y_ * sin(angle),
                   x_ * sin(angle) + y_ * cos(angle));
    }

    /**
     * @brief 原地旋转向量
     * @param angle 旋转角度（弧度），正值为逆时针
     *
     * 注意：需要临时保存 x_ 的值，因为计算 y_ 时需要原始的 x_。
     */
    void Vec2d::SelfRotate(const double angle)
    {
      double tmp_x = x_;
      x_ = x_ * cos(angle) - y_ * sin(angle);
      y_ = tmp_x * sin(angle) + y_ * cos(angle);
    }

    /**
     * @brief 向量加法
     */
    Vec2d Vec2d::operator+(const Vec2d &other) const
    {
      return Vec2d(x_ + other.x(), y_ + other.y());
    }

    /**
     * @brief 向量减法
     */
    Vec2d Vec2d::operator-(const Vec2d &other) const
    {
      return Vec2d(x_ - other.x(), y_ - other.y());
    }

    /**
     * @brief 向量与标量相乘
     */
    Vec2d Vec2d::operator*(const double ratio) const
    {
      return Vec2d(x_ * ratio, y_ * ratio);
    }

    /**
     * @brief 向量与标量相除
     * @note ratio 不应为零
     */
    Vec2d Vec2d::operator/(const double ratio) const
    {
   //   CHECK_GT(std::abs(ratio), kMathEpsilon);
      return Vec2d(x_ / ratio, y_ / ratio);
    }

    /**
     * @brief 向量加法赋值
     */
    Vec2d &Vec2d::operator+=(const Vec2d &other)
    {
      x_ += other.x();
      y_ += other.y();
      return *this;
    }

    /**
     * @brief 向量减法赋值
     */
    Vec2d &Vec2d::operator-=(const Vec2d &other)
    {
      x_ -= other.x();
      y_ -= other.y();
      return *this;
    }

    /**
     * @brief 向量标量乘法赋值
     */
    Vec2d &Vec2d::operator*=(const double ratio)
    {
      x_ *= ratio;
      y_ *= ratio;
      return *this;
    }

    /**
     * @brief 向量标量除法赋值
     */
    Vec2d &Vec2d::operator/=(const double ratio)
    {
  //    CHECK_GT(std::abs(ratio), kMathEpsilon);
      x_ /= ratio;
      y_ /= ratio;
      return *this;
    }

    /**
     * @brief 向量相等比较
     * @param other 另一个向量
     * @return 在 kMathEpsilon 容差内相等则返回 true
     */
    bool Vec2d::operator==(const Vec2d &other) const
    {
      return (std::abs(x_ - other.x()) < kMathEpsilon &&
              std::abs(y_ - other.y()) < kMathEpsilon);
    }

    /**
     * @brief 标量与向量相乘（支持 ratio * vec 写法）
     */
    Vec2d operator*(const double ratio, const Vec2d &vec) { return vec * ratio; }

    /**
     * @brief 生成调试字符串
     * @return 格式化的向量信息
     */
    std::string Vec2d::DebugString() const
    {
      return absl::StrCat("vec2d ( x = ", x_, "  y = ", y_, " )");
    }

  } // namespace math
} // namespace common
