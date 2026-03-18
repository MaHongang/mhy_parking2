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
 * @file speed_data.h
 * @brief 速度数据类的定义
 *
 * 本文件定义了 SpeedData 类，用于存储和管理速度规划结果。
 * SpeedData 是速度曲线的离散化表示，由一系列 SpeedPoint 组成：
 * - 每个 SpeedPoint 包含 (s, t, v, a, da) 五元组
 * - s: 弧长位置
 * - t: 时间戳
 * - v: 速度
 * - a: 加速度
 * - da: 加加速度 (jerk)
 *
 * 主要功能：
 * - 基于时间 t 的速度点插值查询
 * - 基于弧长 s 的速度点插值查询
 * - 速度曲线的总时间和总长度计算
 **/
#pragma once

#include <string>
#include <vector>

#include "common/pnc_point.h"

using common::SpeedPoint;

/**
 * @class SpeedData
 * @brief 速度数据类，存储速度规划的离散化结果
 *
 * 该类继承自 std::vector<SpeedPoint>，提供了对速度曲线的高级操作：
 * - 支持基于时间或弧长的插值查询
 * - 内部按时间 t 升序存储速度点
 * - 使用线性插值在相邻点之间估计任意时刻的速度状态
 *
 * 使用场景：
 * - 存储速度优化器的输出
 * - 轨迹跟踪时的速度查询
 * - 速度曲线的可视化和调试
 *
 * 典型的速度规划流程：
 * 1. 根据路径和约束生成初始速度点
 * 2. 使用优化器求解最优速度曲线
 * 3. 将结果存入 SpeedData
 * 4. 轨迹跟踪时通过 EvaluateByTime 查询目标速度
 */
class SpeedData : public std::vector<SpeedPoint> {
public:
  /**
   * @brief 默认构造函数，创建空的速度数据
   */
  SpeedData() = default;

  /**
   * @brief 虚析构函数
   */
  virtual ~SpeedData() = default;

  /**
   * @brief 从速度点向量构造 SpeedData
   * @param speed_points 速度点序列
   * @note 构造时会按时间 t 自动排序
   */
  explicit SpeedData(std::vector<SpeedPoint> speed_points);

  /**
   * @brief 追加一个速度点到末尾
   * @param s 弧长位置 [m]
   * @param time 时间戳 [s]
   * @param v 速度 [m/s]
   * @param a 加速度 [m/s²]
   * @param da 加加速度 [m/s³]
   * @note 新点的时间必须大于当前最后一个点的时间
   */
  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  /**
   * @brief 基于时间查询插值速度点
   * @param time 查询时间 [s]
   * @param[out] speed_point 输出的插值速度点
   * @return 查询成功返回 true，失败返回 false
   *
   * 失败条件：
   * - 速度点数量少于 2
   * - 查询时间超出有效范围（带 1.0e-6 容差）
   *
   * 插值方法：对 s, v, a, da 分别进行线性插值
   */
  bool EvaluateByTime(const double time, SpeedPoint *const speed_point) const;

  /**
   * @brief 基于弧长查询插值速度点
   * @param s 查询弧长 [m]
   * @param[out] speed_point 输出的插值速度点
   * @return 查询成功返回 true，失败返回 false
   *
   * @note 假设弧长 s 是单调递增的（适用于城市驾驶场景）
   *
   * 失败条件：
   * - 速度点数量少于 2
   * - 查询弧长超出有效范围（带 1.0e-6 容差）
   */
  bool EvaluateByS(const double s, SpeedPoint *const speed_point) const;

  /**
   * @brief 获取速度曲线的总时间
   * @return 总时间 = 末点时间 - 首点时间 [s]
   */
  double TotalTime() const;

  /**
   * @brief 获取速度曲线的总长度
   * @return 总长度 = 末点弧长 - 首点弧长 [m]
   * @note 假设弧长单调递增
   */
  double TotalLength() const;

  /**
   * @brief 获取调试信息字符串
   * @return 格式化的速度点信息
   */
  virtual std::string DebugString() const;
};
