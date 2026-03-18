# Coarse Trajectory Generator - 粗轨迹生成器

本模块实现了基于 **Hybrid A*** 算法的自动泊车路径规划，包含栅格搜索（Grid Search）、Hybrid A* 搜索和 Reeds-Shepp 曲线生成三个核心组件。

---

## 目录结构

```
coarse_trajectory_generator/
├── grid_search.h/cc          # 2D 栅格 A* 搜索（用于启发式代价计算）
├── hybrid_a_star.h/cc        # Hybrid A* 主算法
├── node3d.h/cc               # 3D 状态节点定义
├── reeds_shepp_path.h/cc     # Reeds-Shepp 曲线生成
└── readme.md                 # 本文档
```

---

## 一、算法概述

### Hybrid A* vs 传统 A*

| 特性 | 传统 A* | Hybrid A* |
|------|---------|-----------|
| 状态空间 | (x, y) 二维 | (x, y, θ) 三维 |
| 移动方式 | 4/8 方向离散移动 | 基于车辆运动学模型 |
| 路径特点 | 折线路径 | 平滑可执行轨迹 |
| 适用场景 | 全向移动机器人 | 非完整约束车辆（汽车） |

---

## 二、Grid Search - 栅格搜索

### 2.1 Node2d 数据结构

```cpp
class Node2d {
    int grid_x_, grid_y_;      // 栅格索引
    double path_cost_;         // g: 从起点到当前的代价
    double heuristic_;         // h: 到终点的启发式代价
    double cost_;              // f = g + h
    std::string index_;        // 唯一标识 "x_y"
    std::shared_ptr<Node2d> pre_node_;  // 父节点
};
```

### 2.2 GenerateDpMap - DP代价地图生成

**功能**：从终点反向扩展，计算地图上每个可达栅格到终点的最短路径代价。

**算法**：反向 Dijkstra

```
┌─────────────────────────────────────┐
│         初始化                       │
│  • 终点 end_node 加入 open_pq       │
│  • 终点 cost = 0                    │
└─────────────────┬───────────────────┘
                  │
                  ▼
┌─────────────────────────────────────┐
│     while (open_pq 非空)            │
│  1. 取出 cost 最小的节点            │
│  2. 加入 dp_map_ (已确定最短代价)   │
│  3. 扩展 8 邻域节点                 │
│  4. 若邻居未访问且可达，加入open_pq │
└─────────────────────────────────────┘
```

**代价计算**：
- 直线移动（上下左右）：代价 = 1.0
- 对角移动：代价 = √2 ≈ 1.414

```
    ┌───┬───┬───┐
    │√2 │ 1 │√2 │   邻居移动代价示意
    ├───┼───┼───┤
    │ 1 │ C │ 1 │   C = 当前节点
    ├───┼───┼───┤
    │√2 │ 1 │√2 │
    └───┴───┴───┘
```

### 2.3 CheckDpMap - 代价查询

```cpp
double CheckDpMap(const double sx, const double sy) {
    // 1. 坐标转换为栅格索引
    std::string index = Node2d::CalcIndex(sx, sy, resolution, bounds);
    
    // 2. 哈希表 O(1) 查询
    if (dp_map_.find(index) != dp_map_.end()) {
        return dp_map_[index]->GetCost() * xy_grid_resolution_;
    }
    return infinity;  // 不可达
}
```

**优势**：
- 预计算一次，查询 O(1)
- 考虑障碍物，比欧氏距离更准确
- 保证启发式可采纳（admissible）

---

## 三、Hybrid A* 主算法

### 3.1 Node3d 数据结构

```cpp
class Node3d {
    // 状态
    double x_, y_, phi_;                    // 世界坐标 + 航向角
    int x_grid_, y_grid_, phi_grid_;        // 离散栅格索引
    
    // 轨迹信息（一个节点包含多个中间点）
    std::vector<double> traversed_x_, traversed_y_, traversed_phi_;
    
    // 代价
    double traj_cost_;       // g: 轨迹代价
    double heuristic_cost_;  // h: 启发式代价
    
    // 控制信息
    double steering_;        // 前轮转角
    bool direction_;         // true=前进, false=倒车
    
    // 父节点
    std::shared_ptr<Node3d> pre_node_;
};
```

### 3.2 算法主流程

```
                          ┌─────────────┐
                          │    开始     │
                          └──────┬──────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │  1. 初始化             │
                    │  • 创建起点/终点节点   │
                    │  • 障碍物转换为线段    │
                    │  • 清空 open/close set │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │  2. 预计算 DP Map      │
                    │  GenerateDpMap(终点)   │
                    │  建立全图启发式代价表  │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │  3. 起点加入 open_set  │
                    └────────────┬───────────┘
                                 │
              ┌──────────────────┴──────────────────┐
              │                                      │
              ▼                                      │
    ┌──────────────────┐                            │
    │  open_set 空？   │────── 是 ────▶ 规划失败    │
    └────────┬─────────┘                            │
             │ 否                                    │
             ▼                                      │
    ┌──────────────────┐                            │
    │ 4. 取出最小代价  │                            │
    │    节点          │                            │
    └────────┬─────────┘                            │
             │                                      │
             ▼                                      │
    ┌──────────────────┐                            │
    │ 5. 尝试 RS 直连  │                            │
    │ AnalyticExpansion│────── 成功 ────▶ 规划成功  │
    └────────┬─────────┘                    │       │
             │ 失败                         │       │
             ▼                              │       │
    ┌──────────────────┐                    │       │
    │ 6. 扩展邻居节点  │                    │       │
    │ Next_node_generator                   │       │
    │ (碰撞检测+代价)  │                    │       │
    └────────┬─────────┘                    │       │
             │                              │       │
             └──────────────────────────────┼───────┘
                                            │
                                            ▼
                               ┌────────────────────┐
                               │ 7. 路径回溯        │
                               │    GetResult()     │
                               └─────────┬──────────┘
                                         │
                                         ▼
                               ┌────────────────────┐
                               │ 8. 速度规划        │
                               │ GetTemporalProfile │
                               └────────────────────┘
```

### 3.3 Next_node_generator - 邻居节点生成

基于自行车运动学模型扩展候选节点。

**转向角分配**：

```
假设 next_node_num_ = 10:

前进方向 (index 0-4):
    δ = -max_steer + (2*max_steer/4) * index
    
倒车方向 (index 5-9):
    δ = -max_steer + (2*max_steer/4) * (index - 5)

     左转最大    直行    右转最大
        ↖       ↑       ↗        前进 5 个方向
        ↙       ↓       ↘        倒车 5 个方向
```

**运动学模型**（自行车模型）：

```cpp
for (i = 0; i < arc / step_size_; ++i) {
    next_x   = last_x + step_size_ * cos(last_phi) * direction;
    next_y   = last_y + step_size_ * sin(last_phi) * direction;
    next_phi = last_phi + step_size_ / wheelbase * tan(steering) * direction;
}
```

```
                    ┌─────────────┐
                    │    后轴     │  wheelbase (轴距)
                    └──────┬──────┘
                           │
                ┌──────────┴──────────┐
                │      车身           │
                └──────────┬──────────┘
                           │
                    ┌──────┴──────┐
                    │   前轴      │
                    │  转角 δ     │
                    └─────────────┘
                    
    转弯半径 R = wheelbase / tan(δ)
    角速度 ω = v * tan(δ) / wheelbase
```

### 3.4 代价函数

**总代价**：`f = g + h`

#### TrajCost (g) - 轨迹代价

```cpp
double TrajCost(current_node, next_node) {
    double cost = 0.0;
    
    // 1. 行驶方向惩罚
    if (前进) cost += step_size_ * traj_forward_penalty_;   // 通常 1.0
    else      cost += step_size_ * traj_back_penalty_;      // 通常 > 1.0
    
    // 2. 换挡惩罚
    if (方向改变) cost += traj_gear_switch_penalty_;        // 例如 10.0
    
    // 3. 转向角惩罚
    cost += traj_steer_penalty_ * |steering|;
    
    // 4. 转向角变化惩罚（平滑性）
    cost += traj_steer_change_penalty_ * |Δsteering|;
    
    return cost;
}
```

#### HoloObstacleHeuristic (h) - 启发式代价

```cpp
double HoloObstacleHeuristic(next_node) {
    // O(1) 查询预计算的 DP Map
    return grid_a_star_heuristic_generator_->CheckDpMap(x, y);
}
```

### 3.5 AnalyticExpansion - Reeds-Shepp 解析扩展

尝试用 Reeds-Shepp 曲线直接连接当前节点到终点。

**Reeds-Shepp 曲线**：由直线段和圆弧组成的最短路径，支持前进和倒车。

```
RS 路径类型示例:

LSL: 左转 → 直行 → 左转
┌───╮         ╭───┐
│   │─────────│   │
└───╯         ╰───┘

RSR: 右转 → 直行 → 右转

CSC: 曲线 → 直线 → 曲线
```

**流程**：
```
当前节点 ──▶ ShortestRSP() ──▶ RSPCheck() ──▶ 成功则完成
              计算最短RS曲线    碰撞检测
```

### 3.6 ValidityCheck - 碰撞检测

```cpp
bool ValidityCheck(node) {
    for (每个轨迹点) {
        // 1. 构建车辆包围盒
        Box2d car_box(center, heading, length, width);
        
        // 2. 检测与障碍物碰撞
        for (每个障碍物线段) {
            if (car_box.HasOverlap(linesegment)) {
                return false;  // 碰撞
            }
        }
    }
    return true;  // 无碰撞
}
```

---

## 四、速度规划

### 4.1 简单差分法（GenerateSpeedAcceleration）

#### 4.1.1 速度计算公式详解

```cpp
result->v.push_back(0.0);  // 起点速度为0

for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) / 2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) / 2.0;
    result->v.push_back(discrete_v);
}

result->v.push_back(0.0);  // 终点速度为0
```

**公式解析**：

使用**中心差分法**计算第 i 个点的瞬时速度：

$$v_i = \frac{v_{前向} + v_{后向}}{2}$$

具体展开为：

$$v_i = \frac{1}{2}\left(\frac{\Delta x_{前}}{dt} + \frac{\Delta x_{后}}{dt}\right) \cos\phi_i + \frac{1}{2}\left(\frac{\Delta y_{前}}{dt} + \frac{\Delta y_{后}}{dt}\right) \sin\phi_i$$

**为什么要投影到车辆坐标系？**

这实际上是将**世界坐标系下的速度向量**投影到**车辆朝向方向**：

$$v = \vec{v}_{世界} \cdot \vec{e}_{车头} = v_x \cos\phi + v_y \sin\phi$$

作用：
- 得到沿车辆行驶方向的标量速度
- **正值表示前进，负值表示倒车**

**关于 delta_t_ 参数**：
- 在构造函数中从配置文件读取，默认值 0.2s
- 物理含义：相邻路径点之间的时间间隔
- 通常根据车辆最大速度和加速度预估

**首尾速度设为 0 的原因**：
- 物理意义：车辆从静止起步，到达终点后静止
- 这在泊车场景下是合理的约束

#### 4.1.2 加速度计算

```cpp
for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
}
```

**数组大小关系**：

| 数组 | 大小 | 物理含义 |
|------|------|----------|
| x, y, phi | n | 路径点（状态量） |
| v | n | 每个点的瞬时速度（状态量） |
| a | n - 1 | 点 i 到点 i+1 的平均加速度（控制量） |
| steer | n - 1 | 点 i 到点 i+1 的转向角（控制量） |

**为什么 a 比 x 少一个？**

这符合离散控制系统的建模方式：

$$x_{k+1} = f(x_k, u_k)$$

- **n 个状态点** → **n-1 个控制量**
- 加速度和转向角是**控制输入**，作用于两个状态点之间

```
时刻:     t0    t1    t2    t3   ...   tn-1
位置:     x0    x1    x2    x3   ...   xn-1     (n个状态)
速度:     v0    v1    v2    v3   ...   vn-1     (n个状态)
加速度:     a0    a1    a2   ...   an-2         (n-1个控制)
转向角:     s0    s1    s2   ...   sn-2         (n-1个控制)
```

代码中的验证：
```cpp
if (result->x.size() - result->a.size() != 1) {
    // x 比 a 多 1 个
    return false;
}
```

#### 4.1.3 转向角计算

```cpp
for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base / step_size_;
    if (result->v[i] > 0.0) {
        discrete_steer = std::atan(discrete_steer);
    } else {
        discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
}
```

**数学原理推导**：

基于**自行车模型（Bicycle Model）**，车辆的运动学方程为：

$$\dot{\phi} = \frac{v}{L} \tan(\delta)$$

其中：
- $\phi$ - 航向角（heading angle）
- $v$ - 车辆速度
- $L$ - 轴距（wheelbase）
- $\delta$ - 前轮转向角（steering angle）

离散化后：

$$\frac{\phi_{i+1} - \phi_i}{\Delta t} = \frac{v}{L} \tan(\delta)$$

由于 $v \cdot \Delta t = \Delta s$（位移），整理得：

$$\phi_{i+1} - \phi_i = \frac{\Delta s}{L} \tan(\delta)$$

反解转向角：

$$\delta = \arctan\left(\frac{(\phi_{i+1} - \phi_i) \cdot L}{\Delta s}\right)$$

**为什么使用 step_size_ 而不是 delta_t_？**

| 参数 | 含义 | 单位 | 来源 |
|------|------|------|------|
| `step_size_` | 相邻路径点间的**位移** | 米 (m) | 路径规划时固定 |
| `delta_t_` | 相邻路径点间的**时间** | 秒 (s) | 速度规划时使用 |

关键原因：

1. **物理模型一致性**：Hybrid A* 在扩展节点时使用的运动学方程是：
   ```cpp
   next_phi = last_phi + traveled_distance / wheel_base * tan(steering);
   ```
   这里用的是**位移**（`traveled_distance`），反推转向角时也应该用位移。

2. **路径是几何量**：Hybrid A* 生成的是**几何路径**（x, y, φ），点与点之间的间距是固定的 `step_size_`，与时间无关。

3. **避免除零问题**：如果用 `delta_t_`，需要通过 $\Delta s = v \cdot \Delta t$ 计算位移，但首尾点速度为 0 会导致问题。

**前进/倒车的符号处理**：

```cpp
if (result->v[i] > 0.0) {
    discrete_steer = std::atan(discrete_steer);   // 前进
} else {
    discrete_steer = std::atan(-discrete_steer);  // 倒车
}
```

倒车时取负号的原因：
- 倒车时速度 $v < 0$，但运动学方程中速度应取绝对值
- 航向角变化方向与前进时相反
- 为保持转向角物理意义一致（左转为正、右转为负），需要取反

### 4.2 S曲线优化（GenerateSCurveSpeedAcceleration）

S 曲线优化使用**分段多项式加加速度（Piecewise Jerk）优化**来生成平滑的速度曲线。与简单差分法不同，它通过二次规划（QP）求解最优速度轨迹。

#### 4.2.1 计算累积弧长

```cpp
for (size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
}
```

将路径从 $(x, y)$ 坐标转换为弧长参数 $s$，便于后续速度规划。

#### 4.2.2 估计总行驶时间

```cpp
const double total_t = std::max(
    1.5 * (max_v * max_v + path_length * max_acc) / (max_acc * max_v),
    10.0);
```

**公式推导**：

考虑从静止加速到最大速度，再匀速行驶，最后减速到静止的理想梯形速度曲线：

```
速度 v
    ^
    |      ___________
v_max     /           \
    |    /             \
    |   /               \
    |  /                 \
    | /                   \
    +-------------------------> 时间 t
     t1    匀速段    t1
      ←s1→ ←─s2──→ ←s1→
```

- **加速段**：$s_1 = \frac{v_{max}^2}{2a_{max}}$，时间 $t_1 = \frac{v_{max}}{a_{max}}$
- **匀速段**：$s_2 = L - 2s_1$，时间 $t_2 = \frac{s_2}{v_{max}}$
- **减速段**：同加速段

总时间：
$$T = 2t_1 + t_2 = \frac{2v_{max}}{a_{max}} + \frac{L - \frac{v_{max}^2}{a_{max}}}{v_{max}} = \frac{v_{max}^2 + L \cdot a_{max}}{a_{max} \cdot v_{max}}$$

代码中乘以 1.5 作为安全裕量，确保有足够的时间完成轨迹。

#### 4.2.3 构建二次规划问题

**决策变量**：在 $n$ 个时间节点上的位置 $s_k$

**状态向量**：$\mathbf{x}_k = [s_k, \dot{s}_k, \ddot{s}_k]^T = [s, v, a]^T$

**目标函数**：

$$\min \sum_{k=0}^{n-1} \dddot{s}_k^2 \cdot \Delta t + w_{ref} \sum_{k}(s_k - s_{ref})^2 + w_{a} \sum_{k} a_k^2$$

即最小化**加加速度（jerk）的平方和**，使加速度变化平滑，产生 S 形速度曲线。

代码中的权重设置：
```cpp
piecewise_jerk_problem.set_x_ref(10000.0, x_ref);  // 位置参考权重（尽快到达终点）
piecewise_jerk_problem.set_weight_ddx(10.0);       // 加速度权重
piecewise_jerk_problem.set_weight_dddx(10.0);      // jerk 权重
piecewise_jerk_problem.set_dddx_bound(0.5);        // jerk 边界
```

**约束条件**：

| 约束类型 | 数学表达式 | 代码实现 |
|---------|-----------|---------|
| 位置边界 | $0 \leq s \leq L$ | `x_bounds = {0.0, path_length}` |
| 速度边界 | $0 \leq v \leq v_{max}$ | `dx_bounds = {0.0, max_v}` |
| 加速度边界 | $-a_{max} \leq a \leq a_{max}$ | `ddx_bounds = {-max_acc, max_acc}` |
| jerk 边界 | $\|\dddot{s}\| \leq j_{max}$ | `set_dddx_bound(0.5)` |
| 终点约束 | $s_n = L, v_n = 0, a_n = 0$ | 硬约束设置 |

#### 4.2.4 离散化动力学约束

使用**三阶积分关系**作为等式约束：

$$s_{k+1} = s_k + v_k \Delta t + \frac{1}{2} a_k \Delta t^2 + \frac{1}{6} j_k \Delta t^3$$

$$v_{k+1} = v_k + a_k \Delta t + \frac{1}{2} j_k \Delta t^2$$

$$a_{k+1} = a_k + j_k \Delta t$$

这些约束保证了位置、速度、加速度之间的物理一致性。

#### 4.2.5 路径与速度融合

```cpp
for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
     cur_rel_time += kDenseTimeResoltuion) {
    // 1. 根据时间查询速度曲线上的弧长 s
    speed_data.EvaluateByTime(cur_rel_time, &speed_point);
    
    // 2. 根据弧长 s 插值路径上的位置 (x, y, θ)
    path_point = path_data.Evaluate(speed_point.s());
    
    // 3. 组合结果
    combined_result.x.push_back(path_point.x());
    combined_result.v.push_back(speed_point.v());
    combined_result.a.push_back(speed_point.a());
}
```

**关键思想**：时间 $t$ → 弧长 $s$ → 位置 $(x, y, \theta)$

#### 4.2.6 S 曲线速度剖面特点

```
速度 v
    ^
    |        ____
    |      /     \
    |     /       \
    |    /         \
    |   /           \
    |  /             \
    +------------------> 时间 t
      ↑      ↑      ↑
    加速   匀速   减速
    j>0    j=0    j<0
```

- **加速段**：jerk > 0，加速度从 0 平滑增加到 $a_{max}$，再减小到 0
- **匀速段**：jerk = 0，加速度保持为 0
- **减速段**：jerk < 0，加速度从 0 平滑减小到 $-a_{max}$，再增加到 0

#### 4.2.7 两种方法对比

| 特性 | 简单差分法 | S 曲线优化 |
|------|-----------|-----------|
| 计算方法 | 后处理差分 | 二次规划优化 |
| 速度连续性 | 可能不连续 | 保证连续平滑 |
| 加速度连续性 | 不保证 | 保证连续 |
| Jerk (加加速度) | 可能很大 | 有界约束 |
| 计算复杂度 | O(n) | O(n³) QP求解 |
| 乘坐舒适性 | 一般 | 优秀 |
| 适用场景 | 快速原型验证 | 实际车辆控制 |

---

## 五、配置参数

```cpp
struct HybridAstar {
    double xy_grid_resolution = 0.4;   // xy 栅格分辨率 (m)
    double phi_grid_resolution = 0.1;  // φ 栅格分辨率 (rad)
    double next_node_num = 10;         // 扩展节点数 (前5后5)
    double step_size = 0.2;            // 步长 (m)
    
    // 代价权重
    double traj_forward_penalty = 1.0;      // 前进惩罚
    double traj_back_penalty = 1.0;         // 倒车惩罚
    double traj_gear_switch_penalty = 10.0; // 换挡惩罚
    double traj_steer_penalty = 0;          // 转向角惩罚
    double traj_steer_change_penalty = 0.1; // 转向角变化惩罚
    
    double grid_a_star_xy_resolution = 0.4; // A* 分辨率
    double node_radius = 0.25;              // 碰撞检测半径
};
```

---

## 六、复杂度分析

| 组件 | 时间复杂度 | 空间复杂度 |
|------|-----------|-----------|
| DP Map 预计算 | O(N log N) | O(N) |
| 启发式查询 | **O(1)** | - |
| 单次节点扩展 | O(K × M) | O(K) |
| Hybrid A* 总体 | O(N × K × M) | O(N) |

其中：N = 栅格数，K = 候选方向数，M = 障碍物线段数

---

## 七、使用示例

```cpp
#include "hybrid_a_star.h"

// 1. 创建配置
PlannerOpenSpaceConfig config;

// 2. 创建 Hybrid A* 实例
HybridAStar hybrid_astar(config);

// 3. 设置起点、终点、边界、障碍物
double sx = 10.0, sy = 10.0, sphi = 0.0;      // 起点
double ex = 50.0, ey = 50.0, ephi = M_PI/2;   // 终点
std::vector<double> XYbounds = {0, 100, 0, 100};  // 边界
std::vector<std::vector<Vec2d>> obstacles = {...}; // 障碍物

// 4. 执行规划
HybridAStartResult result;
bool success = hybrid_astar.Plan(sx, sy, sphi, ex, ey, ephi,
                                  XYbounds, obstacles, &result);

// 5. 获取结果
if (success) {
    // result.x, result.y, result.phi - 位置和航向
    // result.v, result.a - 速度和加速度
    // result.steer - 转向角
}
```

---

## 八、Piecewise Jerk 优化器

### 8.1 模块概述

`planning_data/piecewise_jerk/` 目录下包含了基于 OSQP 的分段 Jerk 优化器，用于生成平滑的速度曲线。

#### 文件结构

```
planning_data/piecewise_jerk/
├── piecewise_jerk_problem.h/cc       # QP 优化基类
├── piecewise_jerk_speed_problem.h/cc # 速度优化派生类
└── discretized_path.h                # 离散化路径（辅助）
```

### 8.2 QP 问题形式

$$\min_{s,v,a} \quad \frac{1}{2} \mathbf{x}^T P \mathbf{x} + q^T \mathbf{x}$$

$$\text{s.t.} \quad l \leq A \mathbf{x} \leq u$$

其中决策变量 $\mathbf{x} = [s_0, ..., s_{n-1}, v_0, ..., v_{n-1}, a_0, ..., a_{n-1}]^T$。

### 8.3 目标函数

```cpp
// Hessian 矩阵 P 对角线元素
P[i,i]     = w_x_ref                // 位置参考权重
P[n+i,n+i] = w_dx_ref + penalty[i]  // 速度参考权重
P[2n+i,2n+i] = w_ddx + 2*w_dddx/Δt² // 加速度 + jerk 贡献

// Jerk 交叉项
P[2n+i, 2n+i+1] = -2*w_dddx/Δt²
```

### 8.4 约束矩阵 A

| 约束类型 | 表达式 | 数量 |
|---------|--------|------|
| 变量边界 | $s_{min} \leq s_i \leq s_{max}$ | 3n |
| Jerk 约束 | $jerk_{min} \leq \frac{a_{i+1}-a_i}{\Delta t} \leq jerk_{max}$ | n-1 |
| 速度积分 | $v_{i+1} = v_i + \frac{1}{2}(a_i + a_{i+1})\Delta t$ | n-1 |
| 位置积分 | $s_{i+1} = s_i + v_i \Delta t + \frac{1}{3}a_i \Delta t^2 + \frac{1}{6}a_{i+1}\Delta t^2$ | n-1 |
| 初始状态 | $s_0 = s_{init}, v_0 = v_{init}, a_0 = a_{init}$ | 3 |

### 8.5 编译配置

项目支持可选的 OSQP 依赖：

```cmake
# CMakeLists.txt (planning_data)
find_package(osqp QUIET)

if(osqp_FOUND)
    target_link_libraries(planning_data PUBLIC osqp::osqp)
    target_compile_definitions(planning_data PUBLIC USE_OSQP)
else()
    message(STATUS "osqp not found, using simplified speed optimization")
endif()
```

- **有 OSQP**：使用完整的 QP 优化求解
- **无 OSQP**：使用五次多项式（smootherstep）回退实现

### 8.6 代码示例

```cpp
// 创建速度优化问题
const size_t num_knots = 50;
const double delta_t = 0.2;
std::array<double, 3> init_state = {0.0, 0.0, 0.0}; // s, v, a

PiecewiseJerkSpeedProblem problem(num_knots, delta_t, init_state);

// 设置约束
problem.set_x_bounds(0.0, path_length);        // 位置边界
problem.set_dx_bounds(0.0, max_speed);         // 速度边界  
problem.set_ddx_bounds(-max_acc, max_acc);     // 加速度边界
problem.set_dddx_bound(max_jerk);              // Jerk 边界

// 设置权重
problem.set_weight_ddx(10.0);   // 加速度平滑
problem.set_weight_dddx(10.0);  // Jerk 平滑

// 设置参考轨迹（鼓励尽快到达终点）
std::vector<double> s_ref(num_knots, path_length);
problem.set_x_ref(10000.0, std::move(s_ref));

// 求解
if (problem.Optimize()) {
    const auto& s = problem.opt_x();   // 位置序列
    const auto& v = problem.opt_dx();  // 速度序列
    const auto& a = problem.opt_ddx(); // 加速度序列
}
```

---

## 九、参考文献

1. Dolgov, D., et al. "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments." IJRR, 2010.
2. Reeds, J. A., and Shepp, L. A. "Optimal paths for a car that goes both forwards and backwards." Pacific Journal of Mathematics, 1990.
3. Stellato, B., et al. "OSQP: An Operator Splitting Solver for Quadratic Programs." Mathematical Programming Computation, 2020.

---

## 十、维护信息

- **作者**: Apollo Authors (原始代码)
- **修改**: MaHongang
- **日期**: 2025年12月
- **更新**: 同步 parking2 项目的 OSQP 优化器实现
