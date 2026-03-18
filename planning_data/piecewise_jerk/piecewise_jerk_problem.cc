/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "planning_data/piecewise_jerk/piecewise_jerk_problem.h"
#include <iostream>

namespace {
// 变量默认范围上限：±10^10（实际上是无界）
constexpr double kMaxVariableRange = 1.0e10;
} // namespace

/**
 * @brief 构造函数 - 初始化 Piecewise Jerk 优化问题
 * 
 * 该问题将时间/空间轴离散化为 n 个节点，优化变量包括：
 * - x[0..n-1]:  位置（或弧长 s）
 * - dx[0..n-1]: 速度（一阶导数）
 * - ddx[0..n-1]: 加速度（二阶导数）
 * 
 * 目标：最小化位置偏差、速度、加速度和 Jerk（加加速度）的加权组合
 * 
 * @param num_of_knots 离散节点数量 n（至少 2 个）
 * @param delta_s 时间步长 Δt 或空间步长 Δs
 * @param x_init 初始状态 {x₀, ẋ₀, ẍ₀}
 */
PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3> &x_init) {
  // CHECK_GE(num_of_knots, 2U);
  num_of_knots_ = num_of_knots;

  x_init_ = x_init;  // 初始状态：{位置, 速度, 加速度}

  delta_s_ = delta_s;  // 时间或空间步长

  // 初始化边界为默认值（几乎无界），后续通过 set_xxx_bounds 设置实际边界
  x_bounds_.resize(num_of_knots_,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  dx_bounds_.resize(num_of_knots_,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  ddx_bounds_.resize(num_of_knots_,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));

  // 初始化位置参考权重为 0（默认不跟踪参考）
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}

/**
 * @brief 构建 OSQP 求解器所需的 QP 问题数据
 * 
 * 标准 QP 问题形式：
 *   min  ½·xᵀPx + qᵀx
 *   s.t. l ≤ Ax ≤ u
 * 
 * 其中：
 * - P: Hessian 矩阵（3n×3n 对称正定），编码目标函数的二次项
 * - q: 线性项向量（3n×1），编码目标函数的一次项
 * - A: 约束矩阵（m×3n），编码等式和不等式约束
 * - l, u: 约束上下界（m×1）
 * 
 * @return OSQP 数据结构指针
 */
OSQPData *PiecewiseJerkProblem::FormulateProblem() {
  // ========================================================================
  // 步骤 1: 计算 Hessian 矩阵 P（目标函数的二次项系数）
  // ========================================================================
  std::vector<c_float> P_data;     // CSC 格式：非零元素值
  std::vector<c_int> P_indices;    // CSC 格式：行索引
  std::vector<c_int> P_indptr;     // CSC 格式：列指针
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // ========================================================================
  // 步骤 2: 计算约束矩阵 A 和边界 l, u
  // ========================================================================
  std::vector<c_float> A_data;     // CSC 格式：非零元素值
  std::vector<c_int> A_indices;    // CSC 格式：行索引
  std::vector<c_int> A_indptr;     // CSC 格式：列指针
  std::vector<c_float> lower_bounds;  // 约束下界 l
  std::vector<c_float> upper_bounds;  // 约束上界 u
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // ========================================================================
  // 步骤 3: 计算线性项 q（目标函数的一次项系数）
  // ========================================================================
  std::vector<c_float> q;
  CalculateOffset(&q);

  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
  // CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  size_t kernel_dim = 3 * num_of_knots_;
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  
  // 创建 P 矩阵（上三角 CSC 格式）
  // OSQP 要求：P 矩阵必须是上三角的，且每列的行索引必须按升序排序
  
  // 在创建 csc_matrix 之前验证数据
  std::cout << "创建 P 矩阵前验证: 维度=" << kernel_dim 
            << ", 非零元素数=" << P_data.size() << std::endl;
  if (P_indptr.size() != kernel_dim + 1) {
    std::cout << "错误: P_indptr 大小不正确: " << P_indptr.size() 
              << " != " << (kernel_dim + 1) << std::endl;
  }
  
  // 验证每列的行索引是否按升序排列，以及是否包含下三角元素
  int ind_check = 0;
  for (size_t j = 0; j < kernel_dim; ++j) {
    if (j < P_indptr.size() - 1) {
      size_t start = P_indptr[j];
      size_t end = P_indptr[j + 1];
      for (size_t k = start; k < end; ++k) {
        if (k < P_indices.size() && k < P_data.size()) {
          c_int row_idx = P_indices[k];
          // OSQP 要求: 行索引 <= 列索引（上三角，包括对角线）
          // 如果 row_idx > j，那就是下三角元素，这是不允许的
          if (row_idx > static_cast<c_int>(j)) {
            std::cout << "错误: 创建前发现下三角元素 (" << row_idx 
                      << ", " << j << ")，值=" << P_data[k] << std::endl;
          }
          if (k < end - 1 && row_idx >= P_indices[k + 1]) {
            std::cout << "警告: 创建前发现未排序的行索引: 第" << j 
                      << "列, 索引" << k << ": " << row_idx 
                      << " >= " << P_indices[k + 1] << std::endl;
          }
        }
        ++ind_check;
      }
    }
  }
  
  c_float* P_data_ptr = CopyData(P_data);
  c_int* P_indices_ptr = CopyData(P_indices);
  c_int* P_indptr_ptr = CopyData(P_indptr);
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), P_data_ptr,
                       P_indices_ptr, P_indptr_ptr);
  
  // 验证 P 矩阵格式（调试用）
  if (data->P != nullptr) {
    bool has_error = false;
    // 检查每列的行索引是否按升序排列，以及是否包含下三角元素
    for (c_int j = 0; j < kernel_dim; ++j) {
      c_int start = data->P->p[j];
      c_int end = data->P->p[j + 1];
      
      // 检查所有元素（包括最后一个）
      for (c_int k = start; k < end; ++k) {
        c_int row_idx = data->P->i[k];
        
        // OSQP 的 validate_data 检查: if (P->i[ptr] > j) 报错
        // 这意味着 OSQP 要求行索引 <= 列索引（上三角，包括对角线）
        // 如果 row_idx > j，那就是下三角元素，这是不允许的
        if (row_idx > j) {
          std::cout << "错误: P矩阵包含下三角元素 (" << row_idx 
                    << ", " << j << ")，值=" << data->P->x[k] 
                    << " (OSQP要求行索引 <= 列索引)" << std::endl;
          has_error = true;
        }
        
        // 检查行索引是否按升序排列（相邻元素比较）
        if (k < end - 1 && row_idx > data->P->i[k + 1]) {
          std::cout << "警告: P矩阵第" << j << "列的行索引未排序: " 
                    << row_idx << " > " << data->P->i[k + 1] << std::endl;
          has_error = true;
        }
      }
    }
    
    if (has_error) {
      std::cout << "P矩阵验证失败！矩阵维度: " << kernel_dim << "x" << kernel_dim 
                << ", 非零元素数: " << P_data.size() << std::endl;
    } else {
      std::cout << "P矩阵格式验证通过: " << kernel_dim << "x" << kernel_dim 
                << ", 非零元素数: " << P_data.size() << std::endl;
    }
  }
  
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

bool PiecewiseJerkProblem::Optimize(const int max_iter) {
  OSQPData *data = FormulateProblem();

  OSQPSettings *settings = SolverDefaultSettings();
  settings->max_iter = max_iter;

  OSQPWorkspace *osqp_work = nullptr;
  // OSQP API: osqp_setup(OSQPWorkspace**, const OSQPData*, const OSQPSettings*)
  c_int exitflag = osqp_setup(&osqp_work, data, settings);
  if (exitflag != 0) {
    std::cout << "OSQP setup failed with exitflag: " << exitflag << std::endl;
    FreeData(data);
    c_free(settings);
    return false;
  }

  osqp_solve(osqp_work);

  auto status = osqp_work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    std::cout << "failed optimization status:\t" << osqp_work->info->status;
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  } else if (osqp_work->solution == nullptr) {
    std::cout << "The solution from OSQP is nullptr";
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return false;
  }

  // extract primal results
  x_.resize(num_of_knots_);
  dx_.resize(num_of_knots_);
  ddx_.resize(num_of_knots_);
  for (size_t i = 0; i < num_of_knots_; ++i) {
    x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
    dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
    ddx_.at(i) =
        osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
  }

  // Cleanup
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return true;
}

/**
 * @brief 计算仿射约束矩阵 A 和边界 l, u
 * 
 * ============================================================================
 * 约束类型总览（共 3n + 3(n-1) + 3 = 6n 个约束）
 * ============================================================================
 * 
 * 1. **变量边界约束** (3n 个):
 *    l ≤ x[i] ≤ u    (i = 0..n-1)  位置边界
 *    l ≤ dx[i] ≤ u   (i = 0..n-1)  速度边界
 *    l ≤ ddx[i] ≤ u  (i = 0..n-1)  加速度边界
 * 
 * 2. **Jerk 边界约束** (n-1 个):
 * //实际上Jerk 边界约束已经隐式的包含在下面的等式约束中
 *    约束加加速度（三阶导数）的范围
 *    l ≤ (ddx[i+1] - ddx[i]) / Δt ≤ u
 * 
 * 3. **速度连续性约束** (n-1 个):
 *    通过梯形积分保证速度的连续性
 *    dx[i+1] - dx[i] - 0.5*Δt*(ddx[i] + ddx[i+1]) = 0
 * 
 * 4. **位置连续性约束** (n-1 个):
 *    通过 Simpson 积分保证位置的连续性
 *    x[i+1] - x[i] - Δt*dx[i] - (Δt²/3)*ddx[i] - (Δt²/6)*ddx[i+1] = 0
 * 
 * 5. **初始状态约束** (3 个):
 *    固定起点的位置、速度和加速度
 *    x[0] = x_init[0]
 *    dx[0] = x_init[1]
 *    ddx[0] = x_init[2]
 * 
 * ============================================================================
 * 优化变量排列
 * ============================================================================
 * 
 * 变量向量 x ∈ ℝ^(3n):
 * x = [x[0], x[1], ..., x[n-1],           // 位置：索引 0 ~ n-1
 *      dx[0], dx[1], ..., dx[n-1],        // 速度：索引 n ~ 2n-1
 *      ddx[0], ddx[1], ..., ddx[n-1]]     // 加速度：索引 2n ~ 3n-1
 * 
 * ============================================================================
 * 数学推导：连续性约束
 * ============================================================================
 * 
 * 假设在区间 [t_i, t_{i+1}] 上，加速度 a(t) 线性插值：
 *   a(t) = a_i + (a_{i+1} - a_i) * (t - t_i) / Δt
 * 
 * 【速度积分】
 * v_{i+1} = v_i + ∫[t_i, t_{i+1}] a(t) dt
 *         = v_i + (a_i + a_{i+1}) * Δt / 2       （梯形积分）
 * 
 * 【位置积分】
 * s_{i+1} = s_i + ∫[t_i, t_{i+1}] v(t) dt
 * 其中 v(t) = v_i + ∫[t_i, t] a(τ) dτ
 *           = v_i + a_i*(t-t_i) + (a_{i+1}-a_i)*(t-t_i)²/(2Δt)
 * 
 * 积分得：
 * s_{i+1} = s_i + v_i*Δt + a_i*Δt²/2 + (a_{i+1}-a_i)*Δt²/6
 *         = s_i + v_i*Δt + a_i*Δt²/3 + a_{i+1}*Δt²/6     （Simpson 积分）
 * 
 * @param[out] A_data CSC 格式的非零元素值
 * @param[out] A_indices CSC 格式的行索引
 * @param[out] A_indptr CSC 格式的列指针
 * @param[out] lower_bounds 约束下界 l
 * @param[out] upper_bounds 约束上界 u
 */
void PiecewiseJerkProblem::CalculateAffineConstraint(
    std::vector<c_float> *A_data, std::vector<c_int> *A_indices,
    std::vector<c_int> *A_indptr, std::vector<c_float> *lower_bounds,
    std::vector<c_float> *upper_bounds) {
  // ========================================================================
  // 约束数量计算
  // ========================================================================
  // 3N params bounds on x, x', x''      → 3n 个变量边界
  // 3(N-1) constraints on x, x', x''    → 3(n-1) 个连续性约束
  // 3 constraints on x_init_            → 3 个初始状态约束
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  // ========================================================================
  // 数据结构：临时存储每个变量对应的约束（列优先）
  // ========================================================================
  // variables[i] 存储第 i 个变量参与的所有约束
  // 每个元素是 pair<约束索引, 系数>
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  // ========================================================================
  // 约束索引计数器
  // ========================================================================
  // constraint_index：当前约束的行索引，范围 [0, num_of_constraints-1]
  // 作用：
  // 1. 为每个约束分配唯一的行号
  // 2. 在构建稀疏矩阵时，记录每个非零元素所在的约束（行）
  // 3. 对应约束矩阵 A 的行索引，以及边界向量 l, u 的索引
  // 
  // 约束添加顺序：
  // [0, 3n-1]:           变量边界约束
  // [3n, 4n-2]:          Jerk 边界约束
  // [4n-1, 5n-3]:        速度连续性约束
  // [5n-2, 6n-4]:        位置连续性约束
  // [6n-3, 6n-1]:        初始状态约束
  int constraint_index = 0;
  
  // ========================================================================
  // 第一类约束：变量边界约束 (3n 个)
  // ========================================================================
  // 形式：l_i ≤ var_i ≤ u_i
  // 矩阵形式：A[constraint_index, var_index] = 1.0
  
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      // -------------------- 位置边界 --------------------
      // 约束：x_bounds[i].first ≤ x[i] ≤ x_bounds[i].second
      // 矩阵行：A[constraint_index, i] = 1.0
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      // -------------------- 速度边界 --------------------
      // 约束：dx_bounds[i-n].first ≤ dx[i-n] ≤ dx_bounds[i-n].second
      // 矩阵行：A[constraint_index, i] = 1.0
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      // -------------------- 加速度边界 --------------------
      // 约束：ddx_bounds[i-2n].first ≤ ddx[i-2n] ≤ ddx_bounds[i-2n].second
      // 矩阵行：A[constraint_index, i] = 1.0
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  // CHECK_EQ(constraint_index, num_of_variables);

  // ========================================================================
  // 第二类约束：Jerk（加加速度）边界约束 (n-1 个)
  // ========================================================================
  // 数学形式：jerk_i = (ddx[i+1] - ddx[i]) / Δt
  // 约束形式：dddx_bound.first ≤ jerk_i ≤ dddx_bound.second
  // 
  // 整理后：dddx_bound.first * Δt ≤ ddx[i+1] - ddx[i] ≤ dddx_bound.second * Δt
  // 
  // 矩阵形式：A[constraint_index, 2n+i] = -1.0     （ddx[i] 的系数）
  //          A[constraint_index, 2n+i+1] = +1.0   （ddx[i+1] 的系数）
  // 
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);       // -ddx[i]
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);    // +ddx[i+1]
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // ========================================================================
  // 第三类约束：速度连续性约束（梯形积分）(n-1 个)
  // ========================================================================
  // 数学推导：
  // 假设加速度 a(t) 在 [t_i, t_{i+1}] 区间线性插值
  // 速度积分：v_{i+1} = v_i + ∫[t_i, t_{i+1}] a(t) dt
  //                   = v_i + (a_i + a_{i+1}) * Δt / 2    （梯形公式）
  // 
  // 整理为约束：v_{i+1} - v_i - 0.5*Δt*a_i - 0.5*Δt*a_{i+1} = 0
  // 
  // 注意：这里引入了 scale_factor 来处理数值缩放
  // 原约束乘以 scale_factor[2]（加速度的缩放因子）
  // 
  // 矩阵形式（已缩放）：
  //   scale[2] * dx[i+1] - scale[2] * dx[i] 
  //   - 0.5*Δt*scale[1] * ddx[i] - 0.5*Δt*scale[1] * ddx[i+1] = 0
  // 
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);      // -dx[i]
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);   // +dx[i+1]
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);          // -0.5*Δt*ddx[i]
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);      // -0.5*Δt*ddx[i+1]
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // ========================================================================
  // 第四类约束：位置连续性约束（Simpson 积分）(n-1 个)
  // ========================================================================
  // 数学推导：
  // 速度：v(t) = v_i + ∫[t_i, t] a(τ) dτ
  //           = v_i + a_i*(t-t_i) + (a_{i+1}-a_i)*(t-t_i)²/(2Δt)
  // 
  // 位置积分：s_{i+1} = s_i + ∫[t_i, t_{i+1}] v(t) dt
  //                   = s_i + v_i*Δt + ∫[0, Δt] [a_i*τ + (a_{i+1}-a_i)*τ²/(2Δt)] dτ
  //                   = s_i + v_i*Δt + a_i*Δt²/2 + (a_{i+1}-a_i)*Δt²/6
  //                   = s_i + v_i*Δt + (2a_i + a_{i+1})*Δt²/6
  //                   = s_i + v_i*Δt + a_i*Δt²/3 + a_{i+1}*Δt²/6
  // 
  // 整理为约束：
  // s_{i+1} - s_i - Δt*v_i - (Δt²/3)*a_i - (Δt²/6)*a_{i+1} = 0
  // 
  // 缩放处理（乘以 scale[1]*scale[2]）：
  // scale[1]*scale[2] * (x[i+1] - x[i]) 
  //   - Δt*scale[0]*scale[2] * dx[i]
  //   - (Δt²/3)*scale[0]*scale[1] * ddx[i]
  //   - (Δt²/6)*scale[0]*scale[1] * ddx[i+1] = 0
  // 
  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);         // -x[i]
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);      // +x[i+1]
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);       // -Δt*dx[i]
    variables[2 * n + i].emplace_back(constraint_index, -delta_s_sq_ / 3.0 *
                                                            scale_factor_[0] *
                                                            scale_factor_[1]);     // -(Δt²/3)*ddx[i]
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);                // -(Δt²/6)*ddx[i+1]

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // ========================================================================
  // 第五类约束：初始状态约束 (3 个)
  // ========================================================================
  // 固定起点的位置、速度和加速度为给定值
  // 
  // constrain on x_init
  variables[0].emplace_back(constraint_index, 1.0);  // x[0] = x_init[0]
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);  // dx[0] = x_init[1]
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);  // ddx[0] = x_init[2]
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  // CHECK_EQ(constraint_index, num_of_constraints);

  // ========================================================================
  // 转换为 CSC (Compressed Sparse Column) 格式
  // ========================================================================
  // CSC 是稀疏矩阵的列压缩存储格式，OSQP 要求使用此格式
  // 
  // 三个数组的含义：
  // - A_data[k]: 第 k 个非零元素的值
  // - A_indices[k]: 第 k 个非零元素的行索引
  // - A_indptr[j]: 第 j 列的第一个非零元素在 A_data 中的位置
  //   A_indptr[j+1] - A_indptr[j] 即为第 j 列的非零元素个数
  // 
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);  // 记录第 i 列的起始位置
    for (const auto &variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);       // 非零元素值（系数）

      // constraint index
      A_indices->push_back(variable_nz.first);     // 行索引（约束索引）
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);  // 最后一个元素：总非零元素个数
}

OSQPSettings *PiecewiseJerkProblem::SolverDefaultSettings() {
  // Define Solver default settings
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  bool FLAGS_enable_osqp_debug = false;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;
  return settings;
}

void PiecewiseJerkProblem::set_x_bounds(
    std::vector<std::pair<double, double>> x_bounds) {
  // CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);
}

void PiecewiseJerkProblem::set_dx_bounds(
    std::vector<std::pair<double, double>> dx_bounds) {
  // CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}

void PiecewiseJerkProblem::set_ddx_bounds(
    std::vector<std::pair<double, double>> ddx_bounds) {
  // CHECK_EQ(ddx_bounds.size(), num_of_knots_);
  ddx_bounds_ = std::move(ddx_bounds);
}

void PiecewiseJerkProblem::set_x_bounds(const double x_lower_bound,
                                        const double x_upper_bound) {
  for (auto &x : x_bounds_) {
    x.first = x_lower_bound;
    x.second = x_upper_bound;
  }
}

void PiecewiseJerkProblem::set_dx_bounds(const double dx_lower_bound,
                                         const double dx_upper_bound) {
  for (auto &x : dx_bounds_) {
    x.first = dx_lower_bound;
    x.second = dx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_ddx_bounds(const double ddx_lower_bound,
                                          const double ddx_upper_bound) {
  for (auto &x : ddx_bounds_) {
    x.first = ddx_lower_bound;
    x.second = ddx_upper_bound;
  }
}

void PiecewiseJerkProblem::set_x_ref(const double weight_x_ref,
                                     std::vector<double> x_ref) {
  // CHECK_EQ(x_ref.size(), num_of_knots_);
  weight_x_ref_ = weight_x_ref;
  // set uniform weighting
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::set_x_ref(std::vector<double> weight_x_ref_vec,
                                     std::vector<double> x_ref) {
  // CHECK_EQ(x_ref.size(), num_of_knots_);
  //  CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);
  //  set piecewise weighting
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  x_ref_ = std::move(x_ref);
  has_x_ref_ = true;
}

void PiecewiseJerkProblem::set_end_state_ref(
    const std::array<double, 3> &weight_end_state,
    const std::array<double, 3> &end_state_ref) {
  weight_end_state_ = weight_end_state;
  end_state_ref_ = end_state_ref;
  has_end_state_ref_ = true;
}

void PiecewiseJerkProblem::FreeData(OSQPData *data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}
