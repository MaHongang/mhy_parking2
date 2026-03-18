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
 * @file speed_data.cc
 * @brief 速度数据类的实现
 *
 * 本文件实现了 SpeedData 类的所有成员函数，包括：
 * - 速度点的添加和管理
 * - 基于时间/弧长的插值查询
 * - 速度曲线统计信息计算
 **/

#include "planning_data/speed/speed_data.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "common/pnc_point.h"
#include <algorithm>
#include <mutex>
#include <utility>
//#include "common/util/util.h"
#include "common/math/linear_interpolation.h"
//#include "common/util/string_util.h"
#include "common/util/point_factory.h"

using common::SpeedPoint;

/**
 * @brief 从速度点向量构造 SpeedData
 * @param speed_points 速度点序列
 *
 * 构造时会按时间 t 进行升序排序，确保后续查询的正确性。
 */
SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  // 按时间排序，保证二分查找的正确性
  std::sort(begin(), end(), [](const SpeedPoint &p1, const SpeedPoint &p2) {
    return p1.t() < p2.t();
  });
}

/**
 * @brief 追加速度点到序列末尾
 * @param s 弧长位置 [m]
 * @param time 时间戳 [s]
 * @param v 速度 [m/s]
 * @param a 加速度 [m/s²]
 * @param da 加加速度 [m/s³]
 *
 * 使用互斥锁保证线程安全（虽然当前被注释掉）。
 * 新点的时间应大于当前最后一个点的时间。
 */
void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  static std::mutex mutex_speedpoint;
  // UNIQUE_LOCK_MULTITHREAD(mutex_speedpoint);

  if (!empty()) {
    //   ACHECK(back().t() < time);  // 确保时间递增
  }
  // 使用工厂方法创建 SpeedPoint 并追加
  push_back(common::util::PointFactory::ToSpeedPoint(s, time, v, a, da));
}

/**
 * @brief 基于时间进行插值查询
 * @param t 查询时间 [s]
 * @param[out] speed_point 输出的插值结果
 * @return 查询成功返回 true
 *
 * 实现逻辑：
 * 1. 检查数据有效性（至少 2 个点）
 * 2. 检查时间范围（带 1e-6 容差）
 * 3. 使用二分查找定位相邻点
 * 4. 对 s, v, a, da 分别进行线性插值
 */
bool SpeedData::EvaluateByTime(const double t,
                               common::SpeedPoint *const speed_point) const {
  // 至少需要 2 个点才能插值
  if (size() < 2) {
    return false;
  }
  // 检查时间是否在有效范围内（带微小容差）
  if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
    return false;
  }

  // 二分查找：找到第一个时间 >= t 的点
  auto comp = [](const common::SpeedPoint &sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    // t 超过末点时间，返回末点
    *speed_point = back();
  } else if (it_lower == begin()) {
    // t 小于首点时间，返回首点
    *speed_point = front();
  } else {
    // 在相邻两点之间进行线性插值
    const auto &p0 = *(it_lower - 1);  // 前一个点
    const auto &p1 = *it_lower;         // 当前点
    double t0 = p0.t();
    double t1 = p1.t();

    speed_point->Clear();
    // 对各个属性分别进行线性插值
    speed_point->set_s(common::math::lerp(p0.s(), t0, p1.s(), t1, t));
    speed_point->set_t(t);
    //   if (p0.has_v() && p1.has_v()) {
    speed_point->set_v(common::math::lerp(p0.v(), t0, p1.v(), t1, t));
    //   }
    //    if (p0.has_a() && p1.has_a()) {
    speed_point->set_a(common::math::lerp(p0.a(), t0, p1.a(), t1, t));
    //   }
    //    if (p0.has_da() && p1.has_da()) {
    speed_point->set_da(common::math::lerp(p0.da(), t0, p1.da(), t1, t));
    //    }
  }
  return true;
}

/**
 * @brief 基于弧长进行插值查询
 * @param s 查询弧长 [m]
 * @param[out] speed_point 输出的插值结果
 * @return 查询成功返回 true
 *
 * 与 EvaluateByTime 类似，但基于弧长 s 进行查询。
 * 假设弧长是单调递增的（适用于正常行驶场景）。
 *
 * 注意：对于倒车场景，弧长可能递减，需要特殊处理。
 */
bool SpeedData::EvaluateByS(const double s,
                            common::SpeedPoint *const speed_point) const {
  if (size() < 2) {
    return false;
  }
  // 检查弧长是否在有效范围内
  if (!(front().s() < s + 1.0e-6 && s - 1.0e-6 < back().s())) {
    return false;
  }

  // 二分查找：找到第一个弧长 >= s 的点
  auto comp = [](const common::SpeedPoint &sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    // 在相邻两点之间进行线性插值
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double s0 = p0.s();
    double s1 = p1.s();

    speed_point->Clear();
    speed_point->set_s(s);
    // 基于弧长插值时间和其他属性
    speed_point->set_t(common::math::lerp(p0.t(), s0, p1.t(), s1, s));
    if (/*p0.has_v() && p1.has_v()*/ 1) {
      speed_point->set_v(common::math::lerp(p0.v(), s0, p1.v(), s1, s));
    }
    if (/*p0.has_a() && p1.has_a()*/ 1) {
      speed_point->set_a(common::math::lerp(p0.a(), s0, p1.a(), s1, s));
    }
    if (/*p0.has_da() && p1.has_da()*/ 1) {
      speed_point->set_da(common::math::lerp(p0.da(), s0, p1.da(), s1, s));
    }
  }
  return true;
}

/**
 * @brief 计算速度曲线的总时间
 * @return 总时间 [s]
 */
double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

/**
 * @brief 计算速度曲线的总长度
 * @return 总长度 [m]
 */
double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

/**
 * @brief 生成调试字符串
 * @return 格式化的速度点信息（当前实现为空）
 *
 * TODO: 实现详细的调试输出
 */
std::string SpeedData::DebugString() const {
  // const auto limit = std::min(
  //     size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  // return absl::StrCat("[\n",
  //                     absl::StrJoin(begin(), begin() + limit, ",\n",
  //                                   common::util::DebugStringFormatter()),
  //                     "]\n");
}