# OSQP 使用指南 —— 结合 Piecewise Jerk 问题详解

## 目录

1. [OSQP 简介](#1-osqp-简介)
2. [QP 问题标准形式](#2-qp-问题标准形式)
3. [CSC 稀疏矩阵格式](#3-csc-稀疏矩阵格式)
4. [OSQP 数据结构](#4-osqp-数据结构)
5. [Piecewise Jerk 问题建模](#5-piecewise-jerk-问题建模)
6. [代码实现详解](#6-代码实现详解)
7. [OSQP 求解流程](#7-osqp-求解流程)
8. [参数调优指南](#8-参数调优指南)
9. [常见问题与调试](#9-常见问题与调试)

---

## 1. OSQP 简介

### 1.1 什么是 OSQP

**OSQP** (Operator Splitting Quadratic Program) 是一个开源的凸二次规划 (Quadratic Programming, QP) 求解器。它基于 **ADMM** (Alternating Direction Method of Multipliers) 算法，专门设计用于求解大规模稀疏 QP 问题。

### 1.2 OSQP 的特点

| 特性 | 说明 |
|------|------|
| **高效性** | 针对稀疏问题优化，适合大规模 QP |
| **鲁棒性** | 能处理不精确数据和病态问题 |
| **无需矩阵分解** | 避免数值不稳定性 |
| **热启动** | 支持使用上次解作为初始值加速收敛 |
| **跨平台** | C 语言实现，提供多种语言接口 |

### 1.3 安装 OSQP

```bash
# Ubuntu/Debian
sudo apt-get install libosqp-dev

# 或从源码安装
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make
sudo make install
```

### 1.4 CMake 配置

```cmake
find_package(osqp REQUIRED)
target_link_libraries(your_target osqp::osqp)
```

---

## 2. QP 问题标准形式

### 2.1 数学表达

OSQP 求解以下标准形式的 QP 问题：

$$
\begin{aligned}
\min_{x} \quad & \frac{1}{2} x^T P x + q^T x \\
\text{s.t.} \quad & l \leq Ax \leq u
\end{aligned}
$$

其中：
- $x \in \mathbb{R}^n$ —— 优化变量向量
- $P \in \mathbb{R}^{n \times n}$ —— Hessian 矩阵（半正定对称）
- $q \in \mathbb{R}^n$ —— 线性项向量
- $A \in \mathbb{R}^{m \times n}$ —— 约束矩阵
- $l, u \in \mathbb{R}^m$ —— 约束的下界和上界

### 2.2 约束类型表示

OSQP 使用统一的 **双边约束** 形式，可以表达多种约束：

| 约束类型 | 表示方式 |
|----------|----------|
| 等式约束 $Ax = b$ | $l = u = b$ |
| 不等式约束 $Ax \leq b$ | $l = -\infty, u = b$ |
| 不等式约束 $Ax \geq b$ | $l = b, u = +\infty$ |
| 范围约束 $a \leq x \leq b$ | $A = I, l = a, u = b$ |

### 2.3 为什么目标函数要乘 2

在代码中你会看到：

```cpp
P_data->push_back(row_data_pair.second * 2.0);  // OSQP 需要乘 2
```

这是因为 OSQP 的目标函数定义为 $\frac{1}{2} x^T P x$，而我们的原始目标函数通常是 $x^T P x$。为了匹配标准形式，需要将 P 矩阵元素乘以 2。

---

## 3. CSC 稀疏矩阵格式

### 3.1 什么是 CSC 格式

**CSC** (Compressed Sparse Column) 是一种列压缩稀疏矩阵存储格式。OSQP 使用此格式存储 P 和 A 矩阵，大大减少内存占用和计算量。

### 3.2 CSC 三元组表示

CSC 格式使用三个数组：

| 数组 | 说明 | 长度 |
|------|------|------|
| `x` (data) | 非零元素的值 | nnz |
| `i` (indices) | 非零元素的行索引 | nnz |
| `p` (indptr) | 每列第一个非零元素在 x 中的位置 | n+1 |

其中 `nnz` 是非零元素个数，`n` 是列数。

### 3.3 CSC 格式示例

考虑矩阵：

$$
A = \begin{bmatrix}
1 & 0 & 2 \\
0 & 0 & 3 \\
4 & 5 & 6
\end{bmatrix}
$$

CSC 表示（按列存储）：

```
data    = [1, 4, 5, 2, 3, 6]    // 按列顺序的非零值
indices = [0, 2, 2, 0, 1, 2]    // 对应的行索引
indptr  = [0, 2, 3, 6]          // 第0列从0开始，第1列从2开始，第2列从3开始
```

解读：
- **第0列**: indptr[0]=0 到 indptr[1]=2，即 data[0:2]=[1,4]，行索引 indices[0:2]=[0,2]
- **第1列**: indptr[1]=2 到 indptr[2]=3，即 data[2:3]=[5]，行索引 indices[2:3]=[2]
- **第2列**: indptr[2]=3 到 indptr[3]=6，即 data[3:6]=[2,3,6]，行索引 indices[3:6]=[0,1,2]

### 3.4 CSC 格式深度解析

为了彻底理解 CSC，我们来看一个更复杂的例子，包含**全零列**的情况。

假设矩阵 $M$ ($4 \times 4$)：

$$
M = \begin{bmatrix}
\mathbf{10} & 0 & 0 & 0 \\
0 & \mathbf{20} & 0 & 0 \\
\mathbf{30} & 0 & 0 & 0 \\
0 & \mathbf{40} & 0 & \mathbf{60}
\end{bmatrix}
$$

注意：**第 2 列是全零列**。

#### 3.4.1 数组拆解

1.  **`data` (非零值)**: 按列顺序提取所有非零元素。
    *   第 0 列: 10, 30
    *   第 1 列: 20, 40
    *   第 2 列: (无)
    *   第 3 列: 60
    *   **结果**: `[10, 30, 20, 40, 60]`

2.  **`indices` (行索引)**: 对应每个非零值的行号。
    *   10 -> 行 0, 30 -> 行 2
    *   20 -> 行 1, 40 -> 行 3
    *   60 -> 行 3
    *   **结果**: `[0, 2, 1, 3, 3]`

3.  **`indptr` (列指针)**: 每一列在 `data` 中的起始位置。
    *   `indptr` 的长度永远是 **列数 + 1**。
    *   `indptr[i]` 表示第 `i` 列的起始下标。
    *   第 `i` 列的元素范围是 `[indptr[i], indptr[i+1])` (左闭右开)。

    推导过程：
    *   **第 0 列**: 起始下标 **0**。(`indptr[0] = 0`)
    *   **第 1 列**: 第 0 列有 2 个元素，所以从下标 **2** 开始。(`indptr[1] = 2`)
    *   **第 2 列**: 第 1 列有 2 个元素，2+2=4，从下标 **4** 开始。(`indptr[2] = 4`)
    *   **第 3 列**: 第 2 列有 **0** 个元素，4+0=4，从下标 **4** 开始。(`indptr[3] = 4`) —— **注意：这里值没变！**
    *   **结束**: 第 3 列有 1 个元素，4+1=5。(`indptr[4] = 5`)

    *   **结果**: `[0, 2, 4, 4, 5]`

#### 3.4.2 关键问题解答

**Q1: 为什么 `indptr` 的最后一个值是 `data.size()` 而不是 `data.size() + 1`？**

A: 因为区间是 **左闭右开** 的。
对于最后一列（第 3 列），范围是 `[indptr[3], indptr[4])`，即 `[4, 5)`。
循环代码通常如下：
```cpp
for (int k = indptr[col]; k < indptr[col+1]; ++k) {
    process(data[k]);
}
```
如果 `indptr[4]` 是 6 (即 size+1)，循环会尝试访问 `data[5]`，导致数组越界（因为 `data` 下标最大是 4）。

**Q2: 如果某一列全为零，`indptr` 如何取值？**

A: 该列的起始位置等于下一列的起始位置，即 `indptr[i] == indptr[i+1]`。
在上面的例子中，第 2 列全为零，所以 `indptr[2] = 4` 且 `indptr[3] = 4`。
当程序遍历第 2 列时：
```cpp
for (int k = 4; k < 4; ++k) { ... }
```
循环条件 `k < 4` 不成立，循环体一次都不执行，正确跳过了全零列。

### 3.5 代码中的 CSC 构建

```cpp
// 在 piecewise_jerk_speed_problem.cc 中
int ind_p = 0;
for (int i = 0; i < kNumParam; ++i) {
    P_indptr->push_back(ind_p);           // 记录当前列的起始位置
    for (const auto& row_data_pair : columns[i]) {
        P_data->push_back(row_data_pair.second * 2.0);  // 非零值
        P_indices->push_back(row_data_pair.first);       // 行索引
        ++ind_p;
    }
}
P_indptr->push_back(ind_p);  // 最后一个元素表示总非零个数
```

---

## 4. OSQP 数据结构

### 4.1 OSQPData 结构

```c
typedef struct {
    c_int n;     // 变量个数（P 矩阵的维度）
    c_int m;     // 约束个数（A 矩阵的行数）
    csc *P;      // Hessian 矩阵（上三角，CSC 格式）
    csc *A;      // 约束矩阵（CSC 格式）
    c_float *q;  // 线性项向量
    c_float *l;  // 约束下界
    c_float *u;  // 约束上界
} OSQPData;
```

### 4.2 OSQPSettings 结构（常用参数）

```c
typedef struct {
    c_float eps_abs;       // 绝对精度，默认 1e-3
    c_float eps_rel;       // 相对精度，默认 1e-3
    c_float eps_prim_inf;  // 原问题不可行判定阈值
    c_float eps_dual_inf;  // 对偶问题不可行判定阈值
    c_int   max_iter;      // 最大迭代次数，默认 4000
    c_int   polish;        // 是否进行解的抛光（提高精度）
    c_int   verbose;       // 是否打印求解信息
    c_int   scaled_termination;  // 是否使用缩放的终止条件
} OSQPSettings;
```

### 4.3 OSQPSolution 结构

```c
typedef struct {
    c_float *x;  // 原变量的最优解
    c_float *y;  // 对偶变量的最优解
} OSQPSolution;
```

### 4.4 求解状态码

```c
#define OSQP_SOLVED                 (1)   // 求解成功
#define OSQP_SOLVED_INACCURATE      (2)   // 求解成功但精度不足
#define OSQP_PRIMAL_INFEASIBLE      (-3)  // 原问题不可行
#define OSQP_DUAL_INFEASIBLE        (-4)  // 对偶问题不可行
#define OSQP_MAX_ITER_REACHED       (-2)  // 达到最大迭代次数
```

---

## 5. Piecewise Jerk 问题建模

### 5.1 问题描述

Piecewise Jerk 问题是一个轨迹优化问题，目标是生成平滑的运动曲线，同时满足动力学约束和边界限制。

**状态变量**（以速度优化为例）：

| 变量 | 符号 | 物理意义 |
|------|------|----------|
| 位置 | $s_i$ | 沿轨迹的弧长位置 |
| 速度 | $v_i = \dot{s}_i$ | 沿轨迹的速度 |
| 加速度 | $a_i = \ddot{s}_i$ | 沿轨迹的加速度 |
| 加加速度 (Jerk) | $j_i = \dddot{s}_i$ | 加速度的变化率 |

### 5.2 优化目标

$$
\min \sum_{i=0}^{n-1} \left[ w_s(s_i - s_{ref})^2 + w_v \cdot v_i^2 + w_a \cdot a_i^2 + w_j \cdot j_i^2 \right]
$$

其中：
- $w_s$ —— 位置跟踪权重
- $w_v$ —— 速度权重（通常希望匀速）
- $w_a$ —— 加速度权重（平滑性）
- $w_j$ —— Jerk 权重（舒适性）

### 5.3 动力学约束（三阶积分）

```
┌────────────────────────────────────────────────────────────────────────┐
│                    三阶积分离散化模型                                    │
├────────────────────────────────────────────────────────────────────────┤
│ 加速度-速度关系（梯形积分）:                                             │
│   v[i+1] = v[i] + 0.5 * (a[i] + a[i+1]) * Δt                          │
│                                                                        │
│ 速度-位置关系（考虑 jerk 的积分）:                                       │
│   s[i+1] = s[i] + v[i]*Δt + (1/3)*a[i]*Δt² + (1/6)*a[i+1]*Δt²        │
│                                                                        │
│ Jerk 定义:                                                             │
│   j[i] = (a[i+1] - a[i]) / Δt                                         │
└────────────────────────────────────────────────────────────────────────┘
```

### 5.4 变量布局

对于 $n$ 个时间节点，优化变量向量 $x$ 的布局如下：

```
x = [s_0, s_1, ..., s_{n-1},     <-- 位置变量 (索引 0 到 n-1)
     v_0, v_1, ..., v_{n-1},     <-- 速度变量 (索引 n 到 2n-1)
     a_0, a_1, ..., a_{n-1}]     <-- 加速度变量 (索引 2n 到 3n-1)
```

总变量数: $3n$

### 5.5 Hessian 矩阵 P 的结构

Hessian 矩阵是 $3n \times 3n$ 的稀疏对称矩阵：

```
        s    |    v    |    a
    ─────────┼─────────┼─────────
s   │  W_s   │    0    │    0    │
    ─────────┼─────────┼─────────┤
v   │   0    │   W_v   │    0    │
    ─────────┼─────────┼─────────┤
a   │   0    │    0    │   W_a   │
    ─────────┴─────────┴─────────┘
```

其中 $W_a$ 块包含 Jerk 项产生的耦合：

$$
j_i^2 = \left(\frac{a_{i+1} - a_i}{\Delta t}\right)^2 = \frac{a_i^2 - 2a_i a_{i+1} + a_{i+1}^2}{\Delta t^2}
$$

展开后会在 $W_a$ 块的对角线和副对角线产生非零元素。

### 5.6 约束矩阵 A 的结构

约束包含多个部分：

| 约束类型 | 数量 | 说明 |
|----------|------|------|
| 变量边界 | $3n$ | $s_{min} \leq s_i \leq s_{max}$ 等 |
| Jerk 约束 | $n-1$ | $j_{min} \leq (a_{i+1}-a_i)/\Delta t \leq j_{max}$ |
| 速度积分 | $n-1$ | $v_{i+1} = v_i + 0.5(a_i+a_{i+1})\Delta t$ |
| 位置积分 | $n-1$ | $s_{i+1} = s_i + v_i\Delta t + ...$ |
| 初始状态 | $3$ | $s_0, v_0, a_0$ 固定 |

总约束数: $3n + 3(n-1) + 3 = 6n$

---

## 6. 代码实现详解

### 6.1 构造函数

```cpp
PiecewiseJerkProblem::PiecewiseJerkProblem(
    const size_t num_of_knots,        // 时间节点数 n
    const double delta_s,              // 时间步长 Δt
    const std::array<double, 3>& x_init  // 初始状态 [s₀, v₀, a₀]
) {
    num_of_knots_ = num_of_knots;
    x_init_ = x_init;
    delta_s_ = delta_s;
    
    // 初始化边界为无穷大
    x_bounds_.resize(num_of_knots_, {-1e10, 1e10});
    dx_bounds_.resize(num_of_knots_, {-1e10, 1e10});
    ddx_bounds_.resize(num_of_knots_, {-1e10, 1e10});
}
```

### 6.2 Hessian 矩阵计算 (CalculateKernel)

```cpp
void PiecewiseJerkSpeedProblem::CalculateKernel(
    std::vector<c_float>* P_data,      // 非零值
    std::vector<c_int>* P_indices,     // 行索引
    std::vector<c_int>* P_indptr       // 列指针
) {
    const int n = num_of_knots_;
    
    // 使用二维向量临时存储每列的非零元素
    std::vector<std::vector<std::pair<c_int, c_float>>> columns(3 * n);
    
    // ========== 位置项权重 ==========
    // 目标: w_s * (s_i - s_ref)^2 = w_s * s_i^2 - 2*w_s*s_ref*s_i + const
    // Hessian 对角线: P[i,i] = w_s
    for (int i = 0; i < n; ++i) {
        columns[i].emplace_back(i, weight_x_ref_);
    }
    
    // ========== 速度项权重 ==========
    // P[n+i, n+i] = w_v
    for (int i = 0; i < n; ++i) {
        columns[n + i].emplace_back(n + i, weight_dx_);
    }
    
    // ========== 加速度项权重（含 Jerk 贡献）==========
    // j_i^2 = (a[i+1] - a[i])^2 / Δt^2
    // 对角线: P[2n+i, 2n+i] = w_a + 2*w_j/Δt^2 (中间点)
    // 副对角线: P[2n+i, 2n+i+1] = -2*w_j/Δt^2
    double delta_s_sq = delta_s_ * delta_s_;
    
    // 第一个点: 只被 j_0 影响
    columns[2*n].emplace_back(2*n, weight_ddx_ + weight_dddx_/delta_s_sq);
    
    // 中间点: 被 j_{i-1} 和 j_i 影响
    for (int i = 1; i < n-1; ++i) {
        columns[2*n + i].emplace_back(2*n + i, 
            weight_ddx_ + 2.0*weight_dddx_/delta_s_sq);
    }
    
    // 最后一个点
    columns[3*n - 1].emplace_back(3*n - 1, 
        weight_ddx_ + weight_dddx_/delta_s_sq);
    
    // Jerk 交叉项 (上三角)
    for (int i = 0; i < n-1; ++i) {
        columns[2*n + i].emplace_back(2*n + i + 1, 
            -2.0 * weight_dddx_ / delta_s_sq);
    }
    
    // 转换为 CSC 格式...
}
```

### 6.3 约束矩阵计算 (CalculateAffineConstraint)

```cpp
void PiecewiseJerkProblem::CalculateAffineConstraint(...) {
    const int n = num_of_knots_;
    
    // 约束 1: 变量边界 (3n 个)
    // A 是单位矩阵的对应行
    for (int i = 0; i < 3*n; ++i) {
        variables[i].emplace_back(constraint_index, 1.0);
        lower_bounds[constraint_index] = bound_lower;
        upper_bounds[constraint_index] = bound_upper;
        ++constraint_index;
    }
    
    // 约束 2: Jerk 边界 (n-1 个)
    // j_i = (a[i+1] - a[i]) / Δt
    // => j_min*Δt <= a[i+1] - a[i] <= j_max*Δt
    for (int i = 0; i < n-1; ++i) {
        variables[2*n + i].emplace_back(constraint_index, -1.0);     // -a[i]
        variables[2*n + i + 1].emplace_back(constraint_index, 1.0);  // +a[i+1]
        lower_bounds[constraint_index] = dddx_bound_.first * delta_s_;
        upper_bounds[constraint_index] = dddx_bound_.second * delta_s_;
        ++constraint_index;
    }
    
    // 约束 3: 速度积分等式 (n-1 个)
    // v[i+1] - v[i] - 0.5*Δt*a[i] - 0.5*Δt*a[i+1] = 0
    for (int i = 0; i < n-1; ++i) {
        variables[n + i].emplace_back(constraint_index, -1.0);        // -v[i]
        variables[n + i + 1].emplace_back(constraint_index, 1.0);     // +v[i+1]
        variables[2*n + i].emplace_back(constraint_index, -0.5*delta_s_);
        variables[2*n + i + 1].emplace_back(constraint_index, -0.5*delta_s_);
        lower_bounds[constraint_index] = 0.0;
        upper_bounds[constraint_index] = 0.0;
        ++constraint_index;
    }
    
    // 约束 4: 位置积分等式 (n-1 个)
    // s[i+1] - s[i] - Δt*v[i] - Δt²/3*a[i] - Δt²/6*a[i+1] = 0
    ...
    
    // 约束 5: 初始状态 (3 个)
    // s[0] = s_init, v[0] = v_init, a[0] = a_init
    ...
}
```

### 6.4 问题构建 (FormulateProblem)

```cpp
OSQPData* PiecewiseJerkProblem::FormulateProblem() {
    // 1. 计算 Hessian 矩阵
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices, P_indptr;
    CalculateKernel(&P_data, &P_indices, &P_indptr);
    
    // 2. 计算约束矩阵
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices, A_indptr;
    std::vector<c_float> lower_bounds, upper_bounds;
    CalculateAffineConstraint(&A_data, &A_indices, &A_indptr,
                              &lower_bounds, &upper_bounds);
    
    // 3. 计算线性项
    std::vector<c_float> q;
    CalculateOffset(&q);
    
    // 4. 构建 OSQPData
    OSQPData* data = c_malloc(sizeof(OSQPData));
    data->n = 3 * num_of_knots_;           // 变量数
    data->m = lower_bounds.size();          // 约束数
    
    // 创建 CSC 矩阵 (OSQP 会接管内存)
    data->P = csc_matrix(data->n, data->n, P_data.size(),
                         CopyData(P_data), CopyData(P_indices), 
                         CopyData(P_indptr));
    data->A = csc_matrix(data->m, data->n, A_data.size(),
                         CopyData(A_data), CopyData(A_indices),
                         CopyData(A_indptr));
    data->q = CopyData(q);
    data->l = CopyData(lower_bounds);
    data->u = CopyData(upper_bounds);
    
    return data;
}
```

### 6.5 优化求解 (Optimize)

```cpp
bool PiecewiseJerkProblem::Optimize(const int max_iter) {
    // 1. 构建问题
    OSQPData* data = FormulateProblem();
    
    // 2. 设置求解器参数
    OSQPSettings* settings = SolverDefaultSettings();
    settings->max_iter = max_iter;
    
    // 3. 创建工作空间
    OSQPWorkspace* work = osqp_setup(data, settings);
    if (work == nullptr) {
        // 处理错误...
        return false;
    }
    
    // 4. 求解
    osqp_solve(work);
    
    // 5. 检查状态
    if (work->info->status_val != OSQP_SOLVED && 
        work->info->status_val != OSQP_SOLVED_INACCURATE) {
        // 求解失败...
        return false;
    }
    
    // 6. 提取解
    for (size_t i = 0; i < num_of_knots_; ++i) {
        x_[i] = work->solution->x[i];                      // 位置
        dx_[i] = work->solution->x[i + num_of_knots_];     // 速度
        ddx_[i] = work->solution->x[i + 2*num_of_knots_];  // 加速度
    }
    
    // 7. 清理资源
    osqp_cleanup(work);
    FreeData(data);
    c_free(settings);
    
    return true;
}
```

---

## 7. OSQP 求解流程

### 7.1 完整流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        OSQP 求解流程                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐                                                   │
│  │ 问题建模      │  定义目标函数和约束                               │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 构建 P 矩阵   │  CalculateKernel() → CSC 格式                    │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 构建 q 向量   │  CalculateOffset()                               │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 构建 A 矩阵   │  CalculateAffineConstraint() → CSC 格式          │
│  │ 及 l, u 边界  │                                                   │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 填充 OSQPData │  FormulateProblem()                              │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 配置 Settings │  SolverDefaultSettings()                         │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ osqp_setup() │  初始化求解器工作空间                              │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ osqp_solve() │  执行 ADMM 迭代求解                               │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 检查状态      │  work->info->status_val                          │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 提取解        │  work->solution->x                               │
│  └──────┬───────┘                                                   │
│         ↓                                                           │
│  ┌──────────────┐                                                   │
│  │ 清理资源      │  osqp_cleanup(), FreeData()                      │
│  └──────────────┘                                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2 OSQP 核心 API

| 函数 | 说明 |
|------|------|
| `osqp_setup(data, settings)` | 初始化工作空间 |
| `osqp_solve(work)` | 执行求解 |
| `osqp_warm_start(work, x, y)` | 热启动 |
| `osqp_update_P(work, Px_new, ...)` | 更新 P 矩阵 |
| `osqp_update_A(work, Ax_new, ...)` | 更新 A 矩阵 |
| `osqp_update_bounds(work, l, u)` | 更新边界 |
| `osqp_cleanup(work)` | 释放工作空间 |

---

## 8. 参数调优指南

### 8.1 精度参数

```cpp
settings->eps_abs = 1e-4;  // 绝对精度
settings->eps_rel = 1e-4;  // 相对精度
```

**建议**:
- 实时应用: `1e-3` ~ `1e-4`（速度优先）
- 高精度应用: `1e-5` ~ `1e-6`（精度优先）

### 8.2 迭代控制

```cpp
settings->max_iter = 4000;  // 最大迭代次数
```

**建议**:
- 简单问题: 1000 ~ 2000
- 复杂问题: 4000 ~ 10000

### 8.3 抛光 (Polishing)

```cpp
settings->polish = true;  // 启用解的精修
```

抛光会在 ADMM 收敛后进行额外的求解步骤，提高解的精度，但会增加计算时间。

### 8.4 数值缩放

```cpp
// 在 PiecewiseJerkProblem 中
std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};
```

当变量量纲差异大时，使用缩放因子改善数值条件：

```cpp
// 例如: 位置 (米), 速度 (米/秒), 加速度 (米/秒²)
// 如果位置范围是 [0, 100]，加速度范围是 [-2, 2]
// 可以设置缩放因子使变量范围接近
scale_factor_ = {{0.01, 0.5, 1.0}};  // 使变量范围都接近 1
```

### 8.5 权重调优

| 权重 | 效果 | 典型值 |
|------|------|--------|
| `weight_x_ref_` | 位置跟踪精度 | 1.0 ~ 10.0 |
| `weight_dx_` | 速度平滑度 | 0.1 ~ 1.0 |
| `weight_ddx_` | 加速度大小 | 1.0 ~ 10.0 |
| `weight_dddx_` | 舒适性 (jerk) | 10.0 ~ 100.0 |

---

## 9. 常见问题与调试

### 9.1 求解失败常见原因

| 状态码 | 原因 | 解决方案 |
|--------|------|----------|
| PRIMAL_INFEASIBLE | 约束冲突 | 检查边界设置是否矛盾 |
| DUAL_INFEASIBLE | 目标无下界 | 检查 P 矩阵半正定性 |
| MAX_ITER_REACHED | 迭代不收敛 | 增加 max_iter 或调整精度 |
| 返回 nullptr | 数据错误 | 检查矩阵维度和 CSC 格式 |

### 9.2 CSC 格式调试技巧

```cpp
// 验证 CSC 格式正确性
void ValidateCSC(const std::vector<c_float>& data,
                 const std::vector<c_int>& indices,
                 const std::vector<c_int>& indptr,
                 int rows, int cols) {
    // 1. indptr 长度应为 cols + 1
    assert(indptr.size() == cols + 1);
    
    // 2. data 和 indices 长度相等
    assert(data.size() == indices.size());
    
    // 3. indptr 应单调递增
    for (int i = 1; i <= cols; ++i) {
        assert(indptr[i] >= indptr[i-1]);
    }
    
    // 4. 最后一个 indptr 应等于非零元素个数
    assert(indptr[cols] == data.size());
    
    // 5. 所有行索引应在有效范围内
    for (auto idx : indices) {
        assert(idx >= 0 && idx < rows);
    }
}
```

### 9.3 打印矩阵调试

```cpp
void PrintSparseMatrix(const csc* matrix, const std::string& name) {
    std::cout << name << " (" << matrix->m << "x" << matrix->n << "):" << std::endl;
    for (int j = 0; j < matrix->n; ++j) {
        for (int p = matrix->p[j]; p < matrix->p[j+1]; ++p) {
            std::cout << "  (" << matrix->i[p] << ", " << j << ") = " 
                      << matrix->x[p] << std::endl;
        }
    }
}
```

### 9.4 性能优化建议

1. **热启动**: 在连续求解时，使用上一次的解作为初始值
   ```cpp
   osqp_warm_start(work, x_prev, y_prev);
   ```

2. **矩阵更新**: 对于结构不变的问题，只更新数值
   ```cpp
   osqp_update_P(work, Px_new, OSQP_NULL, 0);
   osqp_update_A(work, Ax_new, OSQP_NULL, 0);
   ```

3. **预分配内存**: 避免频繁的内存分配
   ```cpp
   // 预分配 vector 容量
   P_data.reserve(expected_nnz);
   ```

---

## 附录 A: 完整示例代码

```cpp
#include "planning_data/piecewise_jerk/piecewise_jerk_speed_problem.h"

int main() {
    // 问题参数
    const size_t n = 100;           // 100 个时间节点
    const double delta_t = 0.1;     // 时间步长 0.1s
    std::array<double, 3> init = {0.0, 1.0, 0.0};  // 初始 [s, v, a]
    
    // 创建问题
    PiecewiseJerkSpeedProblem problem(n, delta_t, init);
    
    // 设置边界
    problem.set_x_bounds(0.0, 100.0);      // 位置 [0, 100] m
    problem.set_dx_bounds(0.0, 10.0);      // 速度 [0, 10] m/s
    problem.set_ddx_bounds(-3.0, 3.0);     // 加速度 [-3, 3] m/s²
    problem.set_dddx_bound(2.0);           // jerk [-2, 2] m/s³
    
    // 设置权重
    problem.set_weight_x(1.0);
    problem.set_weight_dx(0.1);
    problem.set_weight_ddx(1.0);
    problem.set_weight_dddx(10.0);
    
    // 设置参考位置（例如匀速运动）
    std::vector<double> s_ref(n);
    for (size_t i = 0; i < n; ++i) {
        s_ref[i] = 1.0 * i * delta_t;  // 1 m/s 匀速
    }
    problem.set_x_ref(1.0, s_ref);
    
    // 求解
    if (problem.Optimize()) {
        const auto& s = problem.opt_x();
        const auto& v = problem.opt_dx();
        const auto& a = problem.opt_ddx();
        
        // 使用结果...
        for (size_t i = 0; i < n; i += 10) {
            std::cout << "t=" << i*delta_t << ": s=" << s[i] 
                      << ", v=" << v[i] << ", a=" << a[i] << std::endl;
        }
    }
    
    return 0;
}
```

---

## 附录 B: 参考资料

1. [OSQP 官方文档](https://osqp.org/docs/)
2. [OSQP GitHub 仓库](https://github.com/osqp/osqp)
3. [Apollo Planning 模块](https://github.com/ApolloAuto/apollo)
4. [Sparse Matrix 稀疏矩阵格式](https://en.wikipedia.org/wiki/Sparse_matrix)

---
zhihu 小虎日记 Apollo Piecewise Jerk Speed/Path Planner: 从速度/路径规划到MPC
*文档版本: 1.0*  
*最后更新: 2025-12-11*
todo:单独验证pjso的效果
todo:前进速度优化和后退有什么区别
