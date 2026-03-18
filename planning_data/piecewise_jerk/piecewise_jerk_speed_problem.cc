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
 * @file piecewise_jerk_speed_problem.cc
 * @brief 分段 Jerk 速度优化问题实现
 * 
 * 实现速度优化专用的 Hessian 矩阵和线性项计算。
 */

#include <algorithm>
#include "planning_data/piecewise_jerk/piecewise_jerk_speed_problem.h"

// ============================================================================
// 构造函数
// ============================================================================

PiecewiseJerkSpeedProblem::PiecewiseJerkSpeedProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3> &x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {
  // 初始化速度惩罚为零向量
  // penalty_dx_[i] 表示第 i 个时刻的额外速度惩罚权重
  penalty_dx_.resize(num_of_knots_, 0.0);
}

// ============================================================================
// 参数设置方法
// ============================================================================

void PiecewiseJerkSpeedProblem::set_dx_ref(const double weight_dx_ref,
                                           const double dx_ref) {
  // 设置速度参考:
  // 目标函数增加项: weight_dx_ref · Σᵢ(ṡᵢ - dx_ref)²
  weight_dx_ref_ = weight_dx_ref;
  dx_ref_ = dx_ref;
  has_dx_ref_ = true;
}

void PiecewiseJerkSpeedProblem::set_penalty_dx(std::vector<double> penalty_dx) {
  // 设置逐点速度惩罚
  // penalty_dx[i] 会加到 Hessian 矩阵的速度对角线元素上
  // 用途: 在特定区域 (如低速区、障碍物附近) 增加惩罚
  penalty_dx_ = std::move(penalty_dx);
}

// ============================================================================
// Hessian 矩阵计算 (核心方法)
// ============================================================================

void PiecewiseJerkSpeedProblem::CalculateKernel(std::vector<c_float> *P_data,
                                                std::vector<c_int> *P_indices,
                                                std::vector<c_int> *P_indptr) {
  /**
   * 构建 Hessian 矩阵 P (3n × 3n 稀疏对称矩阵)
   * 
   * 目标函数的二次型: J = ½·xᵀPx + qᵀx
   * 
   * 矩阵结构:
   * ┌───────────┬───────────┬─────────────────┐
   * │    P_s    │     0     │        0        │  索引 [0, n-1]
   * ├───────────┼───────────┼─────────────────┤
   * │     0     │    P_v    │        0        │  索引 [n, 2n-1]
   * ├───────────┼───────────┼─────────────────┤
   * │     0     │     0     │  P_a (三对角)    │  索引 [2n, 3n-1]
   * └───────────┴───────────┴─────────────────┘
   * 
   * 非零元素数量: n + n + n + (n-1) = 4n - 1
   *              位置  速度  加速度对角线  jerk交叉项
   */
  
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;      // 优化变量总数
  const int kNumValue = 4 * n - 1;  // P 矩阵非零元素数 (仅上三角)
  
  // 临时存储每列的非零元素 (行索引, 值)
  // 使用列优先存储，便于后续转换为 CSC 格式
  //columns[i] 存储第 i 列的非零元素
  //columns是一个二维向量，用于用于临时存储 Hessian 矩阵的非零元素，以便最后转换为 OSQP 要求的 CSC 格式。
 //它的结构是：vector<列向量>，其中每个列向量包含多个 (行索引, 值) 对。
  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);
  int value_index = 0;

  // ========================================================================
  // 第一部分: 位置项 P_s (对角阵)
  // ========================================================================
  // 目标函数: w_s · Σᵢ(sᵢ - s_ref)² = w_s · Σᵢ sᵢ²  - 2·w_s·Σᵢ sᵢ·s_ref + const
  //                                    ↑ Hessian          ↑ 线性项q
  // P[i,i] = w_s / scale²
  //
  // 【数学推导】为什么除以 scale²?
  // 设 x_phys 为物理量(如米)，x_opt 为求解器变量，s 为缩放因子(scale_factor)
  // 关系定义: x_opt = x_phys * s  =>  x_phys = x_opt / s
  // 目标函数项: J = w * (x_phys)^2
  // 代入变量:   J = w * (x_opt / s)^2 = (w / s²) * x_opt²
  // 因此，针对优化变量 x_opt 的二次项系数(Hessian)应为 w / s²
  
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, weight_x_ref_ /
                                   (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  
  // 最后一个位置点额外加上终点权重
  // P[n-1, n-1] = (w_s + w_end_s) / scale²
  columns[n - 1].emplace_back(n - 1, (weight_x_ref_ + weight_end_state_[0]) /
                                         (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // ========================================================================
  // 第二部分: 速度项 P_v (对角阵)
  // ========================================================================
  // 目标函数: (w_v + penalty[i]) · (ṡᵢ - ṡ_ref)²
  // P[n+i, n+i] = (w_v + penalty[i]) / scale²
  
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(n + i,
                                (weight_dx_ref_ + penalty_dx_[i]) /
                                    (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  
  // 最后一个速度点额外加上终点权重
  // P[2n-1, 2n-1] = (w_v + penalty[n-1] + w_end_v) / scale²
  columns[2 * n - 1].emplace_back(
      2 * n - 1, (weight_dx_ref_ + penalty_dx_[n - 1] + weight_end_state_[1]) /
                     (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  // ========================================================================
  // 第三部分: 加速度项 P_a (三对角阵，包含 Jerk 耦合)
  // ========================================================================
  // Jerk 项: w_j · Σᵢ jᵢ² = w_j · Σᵢ ((s̈ᵢ₊₁ - s̈ᵢ)/Δt)²
  //        = w_j/Δt² · Σᵢ (s̈ᵢ² - 2·s̈ᵢ·s̈ᵢ₊₁ + s̈ᵢ₊₁²)
  //          ↑ 对角线贡献    ↑ 非对角线贡献
  //delta_s_ 是时间步长 Δt
  auto delta_s_square = delta_s_ * delta_s_;  // Δt²
  
  // 第一个加速度点: 只被 j₀ 影响 (单侧)
  // P[2n, 2n] = (w_a + w_j/Δt²) / scale²
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // 中间的加速度点: 被 j_{i-1} 和 j_i 同时影响 (双侧)
  // P[2n+i, 2n+i] = (w_a + 2·w_j/Δt²) / scale²
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  // 最后一个加速度点: 只被 j_{n-2} 影响 + 终点权重
  // P[3n-1, 3n-1] = (w_a + w_j/Δt² + w_end_a) / scale²
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // ========================================================================
  // 第四部分: Jerk 交叉项 (上三角非对角线)
  // ========================================================================
  // Jerk 展开的交叉项: -2·w_j/Δt² · s̈ᵢ · s̈ᵢ₊₁
  // 对于上三角矩阵，我们需要添加 P[2n+i, 2n+i+1]（行索引 <= 列索引）
  // 所以应该添加到 columns[2n+i+1]（第 2n+i+1 列），行索引为 2n+i
  //
  // 【数学推导】为什么是 -2.0?
  // Jerk 代价项: w_j * Σ (jerk_i)^2
  // 离散化: jerk_i ≈ (a_{i+1} - a_i) / Δs
  // 展开平方项: (a_{i+1} - a_i)^2 = a_{i+1}^2 - 2*a_{i+1}*a_i + a_i^2
  //                                           ↑ 交叉项系数为 -2
  //
  // 注意: 这里计算的是原始系数，后续构建 P 矩阵时还会统一乘 2 (抵消 OSQP 的 1/2)
  //为什么要乘-2.0? 因为在目标函数中，Jerk 项展开后包含交叉项 -2·w_j/Δt² · s̈ᵢ · s̈ᵢ₊₁，
  //而在构建 Hessian 矩阵时，这个交叉项的系数就是 -2·w_j/Δt² / scale²。
  //
  // 【关键修复】之前错误地添加到 columns[2n+i]，行索引为 2n+i+1，这会产生下三角元素
  // 正确的做法是：添加到 columns[2n+i+1]（第 2n+i+1 列），行索引为 2n+i
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i + 1].emplace_back(2 * n + i,
                                        -2.0 * weight_dddx_ / delta_s_square /
                                            (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  // ========================================================================
  // 转换为 CSC (Compressed Sparse Column) 格式
  // ========================================================================
  // CSC 格式三元组:
  // - P_data: 非零值数组，按列优先顺序
  // - P_indices: 对应的行索引
  // - P_indptr: 每列第一个元素在 P_data 中的位置
  //
  // 注意: OSQP 要求 P 矩阵乘以 2，因为目标函数是 ½xᵀPx
  // 重要: OSQP 要求每列的行索引必须按升序排序！
  
  int ind_p = 0;
  for (int i = 0; i < kNumParam; ++i) {
    P_indptr->push_back(ind_p);  // 第 i 列的起始位置
    
    // 对当前列的行索引进行排序（OSQP 要求）
    // 使用 lambda 函数按行索引排序
    std::sort(columns[i].begin(), columns[i].end(),
              [](const std::pair<c_int, c_float> &a,
                 const std::pair<c_int, c_float> &b) {
                return a.first < b.first;  // 按行索引升序排序
              });
    
    // 验证：确保所有行索引 >= 列索引（上三角要求）
    // 同时检查是否有重复的行索引（这会导致 OSQP 验证失败）
    c_int last_row = -1;
    for (const auto &row_data_pair : columns[i]) {
      c_int row_idx = row_data_pair.first;
      
      // OSQP 的 validate_data 检查: if (P->i[ptr] > j) 报错
      // 这意味着 OSQP 要求行索引 <= 列索引（上三角，包括对角线）
      // 如果 row_idx > i，那就是下三角元素，这是不允许的
      if (row_idx > i) {
        std::cout << "严重错误: P矩阵包含下三角元素 (" << row_idx 
                  << ", " << i << ")，值=" << row_data_pair.second 
                  << "，这违反了OSQP的上三角要求！(行索引必须 <= 列索引)" << std::endl;
        // 不要添加下三角元素，直接跳过
        continue;
      }
      
      // 检查重复的行索引（OSQP 不允许同一列有重复的行索引）
      if (row_idx == last_row) {
        std::cout << "警告: P矩阵第" << i << "列有重复的行索引 " << row_idx 
                  << "，这可能导致OSQP验证失败！" << std::endl;
        // 合并重复项：累加值
        if (!P_data->empty() && !P_indices->empty() && 
            P_indices->back() == row_idx) {
          P_data->back() += row_data_pair.second * 2.0;
          continue;  // 跳过当前项，因为已经累加到前一项
        }
      }
      
      last_row = row_idx;
      
      // OSQP 的目标函数是 ½xᵀPx，而我们的目标是 xᵀPx，所以需要乘 2
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_idx);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);  // 最后一个元素表示总非零个数
}

// ============================================================================
// 线性项计算
// ============================================================================

void PiecewiseJerkSpeedProblem::CalculateOffset(std::vector<c_float> *q) {
  /**
   * 计算目标函数的线性项 q
   * 
   * 来源: 参考跟踪项的展开
   * w · (x - x_ref)² = w·x² - 2·w·x·x_ref + w·x_ref²
   *                     ↑ Hessian    ↑ 线性项q      ↑ 常数项(忽略)
   * 
   * 线性项: q = -2·w·x_ref
   * 
   * q 向量结构 (3n 维):
   * ┌─────────────────────────────────────────┐
   * │ q[0..n-1]:   -2·w_s·s_ref[i]           │  位置参考
   * ├─────────────────────────────────────────┤
   * │ q[n..2n-1]:  -2·w_v·ṡ_ref              │  速度参考
   * ├─────────────────────────────────────────┤
   * │ q[2n..3n-1]: 0 (加速度无参考)           │
   * └─────────────────────────────────────────┘
   * 
   * 终点状态参考会额外叠加到对应位置。
   */
  
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  
  // resize 会将新增的元素初始化为 0.0 (Value Initialization)
  // 如果 q 初始为空，resize 后即为全零向量，这对后续的 += 累加操作是必须的
  q->resize(kNumParam);
  
  for (int i = 0; i < n; ++i) {
    // 位置参考项: q[i] = -2·w_s·s_ref[i] / scale
    if (has_x_ref_) {
      // .at(i) vs [i]:
      // .at(i) 提供边界检查，如果索引越界会抛出 std::out_of_range 异常
      // [i] 不检查边界，效率略高但越界会导致未定义行为
      // 这里使用 at(i) 是为了更安全，且返回的是引用，支持 += 修改原值
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
    
    // 速度参考项: q[n+i] = -2·w_v·ṡ_ref / scale
    // 注意: 所有时刻使用相同的参考速度 dx_ref_
    if (has_dx_ref_) {
      q->at(n + i) += -2.0 * weight_dx_ref_ * dx_ref_ / scale_factor_[1];
    }
  }

  // 终点状态参考 (叠加到最后一个时刻)
  if (has_end_state_ref_) {
    // 终点位置: q[n-1] += -2·w_end_s·s_end / scale
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    
    // 终点速度: q[2n-1] += -2·w_end_v·ṡ_end / scale
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    
    // 终点加速度: q[3n-1] += -2·w_end_a·s̈_end / scale
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

// ============================================================================
// OSQP 求解器设置
// ============================================================================

OSQPSettings *PiecewiseJerkSpeedProblem::SolverDefaultSettings() {
  /**
   * 配置 OSQP 求解器参数
   * 
   * OSQP 使用 ADMM (Alternating Direction Method of Multipliers) 算法
   * 求解 QP 问题: min ½xᵀPx + qᵀx  s.t. l ≤ Ax ≤ u
   * 
   * 关键参数说明:
   * - eps_abs/eps_rel: 收敛精度 (原始残差和对偶残差)
   * - polish: 在 ADMM 收敛后进行精修，提高解的精度
   * - verbose: 打印迭代信息
   * - scaled_termination: 使用缩放后的终止条件
   */
  
  // 分配内存 (使用 OSQP 的内存管理函数)
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  
  // 首先加载默认设置
  osqp_set_default_settings(settings);
  
  // ======================== 精度设置 ========================
  // 绝对精度: |原始残差| < eps_abs 且 |对偶残差| < eps_abs
  settings->eps_abs = 1e-4;
  
  // 相对精度: 与问题规模相关的收敛判据
  settings->eps_rel = 1e-4;
  
  // 原问题不可行判定阈值 (primal infeasibility)
  settings->eps_prim_inf = 1e-5;
  
  // 对偶问题不可行判定阈值 (dual infeasibility)
  settings->eps_dual_inf = 1e-5;
  
  // ======================== 其他设置 ========================
  // 启用解的抛光 (polishing): ADMM 收敛后进行额外的求解步骤
  // 可以提高解的精度，但会增加计算时间
  settings->polish = true;
  
  // 打印求解过程信息 (设为 false 可关闭输出)
  settings->verbose = true;
  
  // 使用缩放后的终止条件 (推荐开启，提高数值稳定性)
  settings->scaled_termination = true;

  return settings;
}
