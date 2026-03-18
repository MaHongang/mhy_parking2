# PiecewiseJerkSpeedProblem 调用流程详解

本文档详细介绍了 `PiecewiseJerkSpeedProblem` 类的调用流程，该类主要用于自动驾驶中的**速度规划（Speed Planning）**，通过二次规划（QP）生成平滑且满足约束的速度曲线。

## 1. 整体流程概览

调用过程主要分为五个步骤：
1.  **初始化 (Initialization)**: 构建问题实例，设定时间步长和初始状态。
2.  **设置约束 (Constraints)**: 设定位置、速度、加速度和加加速度的物理极限。
3.  **设置权重与参考 (Weights & References)**: 设定优化目标（如跟车距离、目标速度）及各项代价权重。
4.  **执行求解 (Optimization)**: 调用 OSQP 求解器进行优化。
5.  **获取结果 (Result Retrieval)**: 提取优化后的 s-t 轨迹（位置、速度、加速度）。

---

## 2. 详细步骤说明

### 步骤 1: 初始化 (Initialization)

构造函数需要三个关键参数：
*   `num_of_knots`: 离散点数量（例如 100 个点）。
*   `delta_s`: 时间步长 $\Delta t$（注意：虽然变量名叫 `delta_s`，在速度规划中通常代表时间间隔，如 0.1s）。
*   `x_init`: 初始状态数组 `[s_0, v_0, a_0]`。

```cpp
// 示例：规划未来 8 秒的轨迹，步长 0.1s (共 80 个点)
size_t num_of_knots = 80;
double delta_t = 0.1;
std::array<double, 3> init_state = {0.0, 5.0, 0.0}; // s=0, v=5m/s, a=0

PiecewiseJerkSpeedProblem speed_problem(num_of_knots, delta_t, init_state);
```

### 步骤 2: 设置边界约束 (Set Bounds)

必须为每个离散点设置物理限制。如果不设置，默认范围可能过大或过小。

*   **位置边界 ($s$)**: 通常由路径规划确定的可行驶区域决定（如 s_min 为当前位置，s_max 为路径终点）。
*   **速度边界 ($v$)**: $[0, v_{limit}]$。
*   **加速度边界 ($a$)**: $[-a_{max}, a_{max}]$，考虑舒适性。
*   **加加速度边界 ($jerk$)**: $[-j_{max}, j_{max}]$，控制顿挫感。

```cpp
// 1. 设置位置边界 (s)
std::vector<std::pair<double, double>> s_bounds(num_of_knots);
for (int i = 0; i < num_of_knots; ++i) {
    s_bounds[i] = {0.0, 100.0}; // 示例：0到100米
}
speed_problem.set_x_bounds(s_bounds);

// 2. 设置速度边界 (v)
// 也可以使用 set_dx_bounds(low, high) 统一设置
speed_problem.set_dx_bounds(0.0, 10.0); // 0 ~ 10 m/s

// 3. 设置加速度边界 (a)
speed_problem.set_ddx_bounds(-4.0, 2.0); // 减速最大 -4, 加速最大 2

// 4. 设置 Jerk 边界
speed_problem.set_dddx_bound(-2.0, 2.0); // Jerk 范围 [-2, 2]
```

### 步骤 3: 设置权重与参考 (Weights & References)

这是调整规划行为的核心。通过调整权重，可以在“严格跟踪参考”和“保持平滑”之间取得平衡。

#### 3.1 基础权重 (Weights)
*   `weight_x`: 位置代价权重（通常较小，除非有定点停车需求）。
*   `weight_dx`: 速度代价权重（在基类中设置，但在 SpeedProblem 中通常使用 `set_dx_ref` 覆盖）。
*   `weight_ddx`: 加加速度权重（平滑性）。
*   `weight_dddx`: Jerk 权重（舒适性，**非常重要**）。

```cpp
speed_problem.set_weight_x(0.0);       // 通常不强求位置
speed_problem.set_weight_dx(0.0);      // 由 set_dx_ref 单独设置
speed_problem.set_weight_ddx(10.0);    // 惩罚大加速度
speed_problem.set_weight_dddx(1000.0); // 强烈惩罚 Jerk，保证舒适
```

#### 3.2 目标参考 (References)
*   **速度参考**: 希望车辆保持的目标巡航速度。
*   **位置参考**: 如果有特定的 s-t 曲线需要跟随（如动态障碍物避让生成的粗略解）。

```cpp
// 设置目标巡航速度 8 m/s，权重为 10.0
// 目标函数项: 10.0 * Σ(v_i - 8.0)^2
speed_problem.set_dx_ref(10.0, 8.0);

// (可选) 设置位置参考
// std::vector<double> s_ref = ...;
// speed_problem.set_x_ref(1.0, s_ref);
```

### 步骤 4: 求解 (Optimization)

调用 `Optimize()` 方法。该方法内部会：
1.  构建 Hessian 矩阵 $P$ 和线性向量 $q$。
2.  构建约束矩阵 $A$ 和边界向量 $l, u$。
3.  调用 OSQP 求解器。

```cpp
// max_iter: 最大迭代次数
bool success = speed_problem.Optimize(4000);

if (!success) {
    // 处理求解失败的情况（如降级策略）
    return;
}
```

### 步骤 5: 获取结果 (Result Retrieval)

求解成功后，从实例中提取优化后的序列。

```cpp
// 获取优化后的 s, v, a 序列
const std::vector<double>& opt_s = speed_problem.opt_x();
const std::vector<double>& opt_v = speed_problem.opt_dx();
const std::vector<double>& opt_a = speed_problem.opt_ddx();

// 使用结果生成轨迹点...
for (int i = 0; i < num_of_knots; ++i) {
    double t = i * delta_t;
    // TrajectoryPoint point;
    // point.set_relative_time(t);
    // point.set_s(opt_s[i]);
    // point.set_v(opt_v[i]);
    // point.set_a(opt_a[i]);
}
```

---

## 3. 完整代码示例

```cpp
#include "planning_data/piecewise_jerk/piecewise_jerk_speed_problem.h"

void PlanSpeedProfile() {
    // 1. 初始化: 5秒规划，0.1s步长
    double delta_t = 0.1;
    int n = 50;
    std::array<double, 3> init_state = {0.0, 2.0, 0.0}; // 初始速度 2m/s
    
    PiecewiseJerkSpeedProblem prob(n, delta_t, init_state);

    // 2. 设置约束
    // 假设总路程限制在 100m 内
    std::vector<std::pair<double, double>> s_bounds(n);
    for(int i=0; i<n; ++i) s_bounds[i] = {0.0, 100.0};
    prob.set_x_bounds(s_bounds);
    
    prob.set_dx_bounds(0.0, 10.0);    // 速度 0-10 m/s
    prob.set_ddx_bounds(-4.0, 2.0);   // 加速度 -4 ~ 2 m/s^2
    prob.set_dddx_bound(-2.0, 2.0);   // Jerk -2 ~ 2 m/s^3

    // 3. 设置权重和目标
    prob.set_weight_ddx(10.0);        // 加速度惩罚
    prob.set_weight_dddx(2000.0);     // Jerk 惩罚 (平滑关键)
    
    // 目标速度 8 m/s，权重 20
    prob.set_dx_ref(20.0, 8.0);

    // 4. 求解
    if (prob.Optimize()) {
        // 5. 获取结果
        auto s = prob.opt_x();
        auto v = prob.opt_dx();
        auto a = prob.opt_ddx();
        // ... 后续处理
    } else {
        // 错误处理
    }
}
```

## 4. 内部数学模型简述

该问题被建模为标准的二次规划 (QP) 问题：

$$
\min \frac{1}{2} x^T P x + q^T x \\
s.t. \quad l \le Ax \le u
$$

*   **优化变量 $x$**: 包含所有时刻的 $s, \dot{s}, \ddot{s}$。
*   **目标函数**: 最小化 速度误差 + 加速度 + Jerk。
*   **约束**: 
    *   动力学约束 (积分关系): $s_{i+1} = s_i + v_i \Delta t + \frac{1}{2} a_i \Delta t^2$ 等。
    *   边界约束: $s, v, a, jerk$ 的上下限。
