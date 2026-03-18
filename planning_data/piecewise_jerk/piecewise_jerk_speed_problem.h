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
 * @file piecewise_jerk_speed_problem.h
 * @brief 分段 Jerk 速度优化问题 - 派生类
 * 
 * 继承自 PiecewiseJerkProblem，专门用于速度曲线的二次规划优化。
 * 
 * ============================================================================
 * 问题描述：路径-时间 (s-t) 速度规划
 * ============================================================================
 * 
 * s (弧长/位置)
 * |
 * |                       P(t₁, s₁)  P(t₂, s₂)
 * |            P(t₀, s₀)                       ... P(t_{n-1}, s_{n-1})
 * |P(start)
 * |
 * |________________________________________________________ t (时间)
 * 
 * 假设时间步长均匀: Δt = t_{k+1} - t_k = 常数
 * 
 * 给定起点状态 (s₀, ṡ₀, s̈₀)，目标是找到最优的速度曲线，使轨迹平滑。
 * 
 * ============================================================================
 * 优化变量 (3n 维向量)
 * ============================================================================
 * 
 * x = [s₀, s₁, ..., s_{n-1},      // 位置 (弧长)
 *      ṡ₀, ṡ₁, ..., ṡ_{n-1},      // 速度 (一阶导数)
 *      s̈₀, s̈₁, ..., s̈_{n-1}]      // 加速度 (二阶导数)
 * 
 * ============================================================================
 * 目标函数
 * ============================================================================
 * 
 * min J = Σᵢ [ w_s·(sᵢ - s_ref)² +           // 位置跟踪
 *              (w_v + penalty_dx[i])·(ṡᵢ - ṡ_ref)² +  // 速度跟踪 + 惩罚
 *              w_a·s̈ᵢ² +                      // 加速度正则化
 *              w_j·(s⃛ᵢ)² ]                    // Jerk 平滑性
 *       + w_end·‖x_{n-1} - x_end‖²            // 终点状态约束
 * 
 * 其中 Jerk: s⃛ᵢ = (s̈ᵢ₊₁ - s̈ᵢ) / Δt
 * 
 * ============================================================================
 * 与基类的区别
 * ============================================================================
 * 
 * 1. 新增速度参考项: w_v·(ṡ - ṡ_ref)²  (基类无此项)
 * 2. 新增速度惩罚: penalty_dx[i]      (可对不同时刻施加不同惩罚)
 * 3. 覆盖 CalculateKernel/CalculateOffset 实现速度优化专用的 Hessian 矩阵
 */

#pragma once

#include <utility>
#include <vector>

#include "planning_data/piecewise_jerk/piecewise_jerk_problem.h"

/**
 * @class PiecewiseJerkSpeedProblem
 * @brief 分段 Jerk 速度优化问题
 * 
 * 继承自 PiecewiseJerkProblem，实现速度曲线专用的 Hessian 矩阵和线性项计算。
 * 
 * 使用方法:
 * @code
 *   // 创建问题: 100个时间点，时间步长0.1s，初始状态[s=0, v=1, a=0]
 *   PiecewiseJerkSpeedProblem problem(100, 0.1, {0.0, 1.0, 0.0});
 *   
 *   // 设置边界
 *   problem.set_x_bounds(0.0, 100.0);    // 位置范围
 *   problem.set_dx_bounds(0.0, 10.0);    // 速度范围 [0, 10] m/s
 *   problem.set_ddx_bounds(-3.0, 3.0);   // 加速度范围 [-3, 3] m/s²
 *   
 *   // 设置速度参考 (权重=10, 目标速度=2 m/s)
 *   problem.set_dx_ref(10.0, 2.0);
 *   
 *   // 求解
 *   problem.Optimize();
 *   
 *   // 获取结果
 *   auto& s = problem.opt_x();    // 位置序列
 *   auto& v = problem.opt_dx();   // 速度序列
 *   auto& a = problem.opt_ddx();  // 加速度序列
 * @endcode
 */
class PiecewiseJerkSpeedProblem : public PiecewiseJerkProblem {
public:
  /**
   * @brief 构造函数
   * @param num_of_knots 时间离散点数量 n
   * @param delta_s 时间步长 Δt (注意: 虽然参数名是 delta_s，实际代表时间步长)
   * @param x_init 初始状态 [s₀, ṡ₀, s̈₀] = [位置, 速度, 加速度]
   */
  PiecewiseJerkSpeedProblem(const size_t num_of_knots, const double delta_s,
                            const std::array<double, 3> &x_init);

  virtual ~PiecewiseJerkSpeedProblem() = default;

  /**
   * @brief 设置速度参考值
   * 
   * 目标函数增加项: w_v · Σᵢ(ṡᵢ - ṡ_ref)²
   * 
   * @param weight_dx_ref 速度参考权重 w_v
   * @param dx_ref 目标速度值 ṡ_ref (所有时刻使用相同参考值)
   */
  void set_dx_ref(const double weight_dx_ref, const double dx_ref);

  /**
   * @brief 设置速度惩罚项
   * 
   * 允许对不同时刻施加不同的速度惩罚，常用于:
   * - 低速区域增加惩罚，避免车辆静止
   * - 障碍物附近增加惩罚，强制减速
   * 
   * Hessian 对角线变为: P[n+i, n+i] = w_v + penalty_dx[i]
   * 
   * @param penalty_dx 惩罚权重序列，长度必须等于 num_of_knots
   */
  void set_penalty_dx(std::vector<double> penalty_dx);

protected:
  /**
   * @brief 计算 Hessian 矩阵 P (二次项系数)
   * 
   * 构建 CSC 格式的稀疏对称矩阵 P，只存储上三角部分。
   * 
   * 矩阵结构 (3n × 3n):
   * ┌─────────┬─────────┬───────────┐
   * │  W_s    │    0    │     0     │  位置项: w_s
   * ├─────────┼─────────┼───────────┤
   * │   0     │  W_v    │     0     │  速度项: w_v + penalty[i]
   * ├─────────┼─────────┼───────────┤
   * │   0     │    0    │  W_a(三对角)│  加速度项: w_a + jerk耦合
   * └─────────┴─────────┴───────────┘
   * 
   * Jerk 项展开:
   * j² = ((s̈ᵢ₊₁ - s̈ᵢ)/Δt)² = (s̈ᵢ² - 2·s̈ᵢ·s̈ᵢ₊₁ + s̈ᵢ₊₁²) / Δt²
   * 产生三对角结构。
   * 
   * @param[out] P_data CSC 格式的非零值数组
   * @param[out] P_indices CSC 格式的行索引数组
   * @param[out] P_indptr CSC 格式的列指针数组
   */
  void CalculateKernel(std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices,
                       std::vector<c_int> *P_indptr) override;

  /**
   * @brief 计算线性项 q (一次项系数)
   * 
   * 来源于目标函数中的参考项展开:
   * (x - x_ref)² = x² - 2·x·x_ref + x_ref²
   *                 ↑       ↑
   *            Hessian项  线性项q
   * 
   * q 向量结构 (3n 维):
   * q = [ -2·w_s·s_ref[0], ..., -2·w_s·s_ref[n-1],     // 位置参考
   *       -2·w_v·ṡ_ref,    ..., -2·w_v·ṡ_ref,          // 速度参考
   *       0,               ..., 0                ]     // 加速度无参考
   * 
   * @param[out] q 线性项向量
   */
  void CalculateOffset(std::vector<c_float> *q) override;

  /**
   * @brief OSQP 求解器参数设置
   * 
   * 速度优化专用的精度和求解参数:
   * - eps_abs/eps_rel: 1e-4 (相对较高精度)
   * - polish: true (启用解的抛光精修)
   * - verbose: true (打印求解信息)
   * 
   * @return OSQP 设置结构体指针 (调用者负责释放)
   */
  OSQPSettings *SolverDefaultSettings() override;

  // ==================== 成员变量 ====================

  /// 是否设置了速度参考
  bool has_dx_ref_ = false;
  
  /// 速度参考权重 w_v，对应目标函数 w_v·(ṡ - ṡ_ref)²
  double weight_dx_ref_ = 0.0;
  
  /// 目标速度值 ṡ_ref (标量，所有时刻使用相同值)
  double dx_ref_ = 0.0;

  /// 速度惩罚权重序列 penalty_dx[i]
  /// Hessian 对角线: P[n+i, n+i] = w_v + penalty_dx[i]
  /// 用途: 在特定区域增加惩罚 (如低速区、障碍物附近)
  std::vector<double> penalty_dx_;
};
