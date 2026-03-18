/**
 * @file planner_open_space_config.h
 * @brief 开放空间规划器配置参数定义
 *
 * 本文件定义了开放空间（泊车）规划器的各种配置参数，包括：
 * - Hybrid A* 算法参数
 * - 距离逼近优化参数
 * - 双变量热启动参数
 * - 迭代锚点平滑参数
 *
 * 开放空间规划的典型流程：
 * 1. Hybrid A* 生成粗轨迹（warm start）
 * 2. 双变量热启动初始化对偶变量
 * 3. 距离逼近优化求解精细轨迹
 * 4. 迭代锚点平滑处理
 *
 * @note 参数单位说明：
 * - 长度单位：米 [m]
 * - 角度单位：弧度 [rad]
 * - 时间单位：秒 [s]
 * - 速度单位：米/秒 [m/s]
 * - 加速度单位：米/秒² [m/s²]
 */

#pragma once

#include <string>

/**
 * @struct sCurveConfig
 * @brief S 曲线（速度优化）参数配置
 *
 * 用于分段 jerk 速度优化问题的权重配置。
 * 优化目标函数：min w_acc*a² + w_jerk*j² + w_kappa*κ + w_s*(s-s_ref)² + w_v*(v-v_ref)²
 */
struct sCurveConfig {
  double acc_weight = 1.0;             ///< 加速度惩罚权重
  double jerk_weight = 0.0;            ///< 加加速度（jerk）惩罚权重
  double kappa_penalty_weight = 100.0; ///< 曲率惩罚权重（限制高曲率处的速度）
  double ref_s_weight = 0.1;           ///< 参考弧长跟踪权重
  double ref_v_weight = 0.0;           ///< 参考速度跟踪权重
};

/**
 * @struct HybridAstar
 * @brief Hybrid A* 算法参数配置
 *
 * Hybrid A* 是一种结合了栅格 A* 和连续状态空间搜索的算法，
 * 用于在考虑车辆运动学约束的情况下搜索可行路径。
 *
 * 算法特点：
 * - 考虑非完整约束（车辆无法横向移动）
 * - 支持前进和后退
 * - 使用 Reed-Shepp 曲线进行解析扩展
 */
struct HybridAstar {
  // ==================== 栅格化参数 ====================
  double xy_grid_resolution = 0.4;  ///< XY 平面栅格分辨率 [m]
  double phi_grid_resolution = 0.1; ///< 航向角栅格分辨率 [rad]

  // ==================== 搜索参数 ====================
  double next_node_num = 10;        ///< 每次扩展的节点数量（前进 5 + 后退 5）
  double step_size = 0.2;           ///< 扩展步长 [m]

  // ==================== 代价函数权重 ====================
  double traj_forward_penalty = 1.0;      ///< 前进运动惩罚
  double traj_back_penalty = 1.0;         ///< 后退运动惩罚
  double traj_gear_switch_penalty = 10.0; ///< 档位（前进/后退）切换惩罚
  double traj_steer_penalty = 0;          ///< 方向盘转角惩罚
  double traj_steer_change_penalty = 0.1; ///< 方向盘转角变化惩罚

  // ==================== 启发函数参数 ====================
  double grid_a_star_xy_resolution = 0.4; ///< 2D A* 启发函数的栅格分辨率 [m]
  double node_radius = 0.25;              ///< 节点判重半径 [m]

  // ==================== 其他参数 ====================
  double delta_t = 1;                     ///< 时间步长 [s]
  sCurveConfig s_curve_config;            ///< 速度优化配置
};

/**
 * @struct IpoptConfig
 * @brief IPOPT 优化器参数配置
 *
 * IPOPT (Interior Point OPTimizer) 是一个用于大规模非线性优化的开源软件。
 * 这些参数控制优化器的收敛行为和数值稳定性。
 */
struct IpoptConfig {
  double ipopt_print_level = 0;                         ///< 打印级别（0-12）
  double mumps_mem_percent = 6000;                      ///< MUMPS 内存百分比
  double mumps_pivtol = 1e-06;                          ///< MUMPS 主元容差
  double ipopt_max_iter = 2000;                         ///< 最大迭代次数
  double ipopt_tol = 0.0001;                            ///< 收敛容差
  double ipopt_acceptable_constr_viol_tol = 0.1;        ///< 可接受的约束违反容差
  double ipopt_min_hessian_perturbation = 1e-12;        ///< 最小 Hessian 扰动
  double ipopt_jacobian_regularization_value = 1e-07;   ///< Jacobian 正则化值
  std::string ipopt_print_timing_statistics = "yes";    ///< 打印时间统计
  std::string ipopt_alpha_for_y = "min";                ///< y 的步长选择策略
  std::string ipopt_recalc_y = "yes";                   ///< 重新计算对偶变量
  double ipopt_mu_init = 0.1;                           ///< 初始障碍参数
};

/**
 * @struct DistanceApproachConfig
 * @brief 距离逼近优化器参数配置
 *
 * 距离逼近是一种基于非线性优化的轨迹优化方法，
 * 通过优化与障碍物的距离和轨迹平滑度来生成安全、舒适的轨迹。
 *
 * 优化变量：位置 (x, y)、航向角 φ、速度 v、方向盘转角 δ、加速度 a
 *
 * 可选模式：
 * - DISTANCE_APPROACH_IPOPT: 基本模式
 * - DISTANCE_APPROACH_IPOPT_RELAX_END: 终点约束放松
 * - DISTANCE_APPROACH_CORRIDOR_IPOPT: 安全走廊模式
 */
struct DistanceApproachConfig {
  // ==================== 控制输入权重 ====================
  double weight_steer = 0.3;            ///< 方向盘转角权重
  double weight_a = 1.0;                ///< 加速度权重
  double weight_steer_rate = 2;         ///< 方向盘转速权重
  double weight_a_rate = 2.5;           ///< 加速度变化率权重

  // ==================== 状态跟踪权重 ====================
  double weight_x = 1;                  ///< x 位置跟踪权重
  double weight_y = 1;                  ///< y 位置跟踪权重
  double weight_phi = 1;                ///< 航向角跟踪权重
  double weight_v = 0.0;                ///< 速度跟踪权重

  // ==================== 拼接约束权重 ====================
  double weight_steer_stitching = 1.75; ///< 方向盘转角拼接权重
  double weight_a_stitching = 3.25;     ///< 加速度拼接权重

  // ==================== 时间优化权重 ====================
  double weight_first_order_time = 1;   ///< 一阶时间权重（总时间）
  double weight_second_order_time = 2.0;///< 二阶时间权重（时间变化率）

  // ==================== 其他权重 ====================
  double weight_end_state = 1.0;        ///< 终点状态权重
  double weight_slack = 1.0;            ///< 松弛变量权重

  // ==================== 约束参数 ====================
  double min_safety_distance = 0.01;    ///< 最小安全距离 [m]
  double max_speed_forward = 2.0;       ///< 最大前进速度 [m/s]
  double max_speed_reverse = 1.0;       ///< 最大倒车速度 [m/s]
  double max_acceleration_forward = 2.0;///< 最大前进加速度 [m/s²]
  double max_acceleration_reverse = 1.0;///< 最大倒车加速度 [m/s²]
  double min_time_sample_scaling = 0.5; ///< 最小时间缩放因子
  double max_time_sample_scaling = 1.5; ///< 最大时间缩放因子
  double use_fix_time = false;          ///< 是否使用固定时间

  IpoptConfig ipopt_config;             ///< IPOPT 优化器配置

  // ==================== 调试选项 ====================
  bool enable_constraint_check = false;     ///< 启用约束检查
  bool enable_hand_derivative = false;      ///< 启用手动导数
  bool enable_derivative_check = false;     ///< 启用导数检查
  bool enable_initial_final_check = false;  ///< 启用初始/终止状态检查

  /**
   * 距离逼近模式选择：
   * - "DISTANCE_APPROACH_IPOPT": 基本优化模式
   * - "DISTANCE_APPROACH_IPOPT_RELAX_END": 终点约束放松模式
   * - "DISTANCE_APPROACH_CORRIDOR_IPOPT": 安全走廊模式（推荐）
   */
  std::string distance_approach_mode = "DISTANCE_APPROACH_CORRIDOR_IPOPT";

  bool enable_check_initial_state = false;  ///< 启用初始状态检查
  bool enable_jacobian_ad = true;           ///< 启用自动微分计算 Jacobian
};

/**
 * @struct OsqpConfig
 * @brief OSQP 二次规划求解器参数配置
 *
 * OSQP (Operator Splitting Quadratic Program) 是一个高效的
 * 凸二次规划求解器，用于双变量热启动问题。
 */
struct OsqpConfig {
  double alpha = 1.0;         ///< 过松弛参数
  double eps_abs = 1.0e-3;    ///< 绝对容差
  double eps_rel = 1.0e-3;    ///< 相对容差
  double max_iter = 10000;    ///< 最大迭代次数
  bool polish = true;         ///< 启用解的精修
  bool osqp_debug_log = false;///< 启用调试日志
};

/**
 * @struct DualVariableWarmStartConfig
 * @brief 双变量热启动参数配置
 *
 * 双变量热启动用于为非线性优化提供良好的初始对偶变量，
 * 可以加速收敛并提高求解成功率。
 */
struct DualVariableWarmStartConfig {
  double weight_d = 1.0;                ///< 距离权重
  std::string qp_format = "OSQP";       ///< QP 求解器类型
  OsqpConfig osqp_config;               ///< OSQP 配置
  double min_safety_distance = 0.1;     ///< 最小安全距离 [m]
  bool debug_osqp = false;              ///< 启用 OSQP 调试
  double beta = 1.0;                    ///< Beta 参数
};

/**
 * @struct IterativeAnchoringSmootherConfig
 * @brief 迭代锚点平滑器参数配置
 *
 * 迭代锚点平滑器用于对优化后的轨迹进行后处理，
 * 通过插值和重锚点来提高轨迹的平滑度和可执行性。
 */
struct IterativeAnchoringSmootherConfig {
  // ==================== 插值参数 ====================
  double interpolated_delta_s = 0.1;        ///< 插值弧长间隔 [m]

  // ==================== 重锚点参数 ====================
  double reanchoring_trails_num = 50;       ///< 重锚点尝试次数
  double reanchoring_pos_stddev = 0.25;     ///< 位置重锚点标准差 [m]
  double reanchoring_length_stddev = 1.0;   ///< 长度重锚点标准差 [m]

  // ==================== 边界估计参数 ====================
  bool estimate_bound = false;              ///< 是否估计边界
  double default_bound = 2.0;               ///< 默认边界 [m]
  double vehicle_shortest_dimension = 1.04; ///< 车辆最短尺寸 [m]

  // ==================== 碰撞检测参数 ====================
  double collision_decrease_ratio = 0.9;    ///< 碰撞检测衰减比例

  // ==================== 速度/加速度约束 ====================
  double max_forward_v = 2.0;               ///< 最大前进速度 [m/s]
  double max_reverse_v = 2.0;               ///< 最大倒车速度 [m/s]
  double max_forward_acc = 3.0;             ///< 最大前进加速度 [m/s²]
  double max_reverse_acc = 2.0;             ///< 最大倒车加速度 [m/s²]
  double max_acc_jerk = 4.0;                ///< 最大加加速度 [m/s³]

  // ==================== 时间参数 ====================
  double delta_t = 0.2;                     ///< 时间步长 [s]
  sCurveConfig s_curve_config;              ///< 速度优化配置
};

/**
 * @class PlannerOpenSpaceConfig
 * @brief 开放空间规划器总配置类
 *
 * 整合所有子配置，提供开放空间规划器的完整参数设置。
 *
 * 配置层次：
 * - warm_start_config: Hybrid A* 热启动配置
 * - distance_approach_config: 距离逼近优化配置
 * - dual_variable_warm_start_config: 对偶变量热启动配置
 * - iterative_anchoring_smoother_config: 迭代平滑器配置
 */
class PlannerOpenSpaceConfig {
private:
  /* data */
public:
  PlannerOpenSpaceConfig() = default;
  ~PlannerOpenSpaceConfig() = default;

  HybridAstar warm_start_config;                                    ///< Hybrid A* 配置
  DistanceApproachConfig distance_approach_config;                  ///< 距离逼近优化配置
  DualVariableWarmStartConfig dual_variable_warm_start_config;      ///< 对偶变量热启动配置
  IterativeAnchoringSmootherConfig iterative_anchoring_smoother_config; ///< 迭代平滑器配置

  double is_near_destination_threshold = 0.1;  ///< 判定接近目标的阈值 [m]
  double delta_t = 0.1;                         ///< 全局时间步长 [s]
};

// //类的成员变量是否能够在定义时设置初值

// PlannerOpenSpaceConfig::PlannerOpenSpaceConfig() {}
