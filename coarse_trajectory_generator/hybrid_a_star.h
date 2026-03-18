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

/**
 * @file hybrid_a_star.h
 * @brief Hybrid A* 路径规划器
 *
 * 本文件实现了 Hybrid A* 算法的核心逻辑，用于在开放空间（如停车场）中
 * 进行无人驾驶车辆的粗轨迹规划。该算法结合了传统 A* 的栅格搜索与
 * Reed-Shepp 曲线的解析扩展，能够生成满足车辆运动学约束的可行路径。
 *
 * 主要功能：
 * - Hybrid A* 路径搜索：考虑车辆运动学约束的三维（x, y, φ）状态空间搜索
 * - 启发式代价计算：使用 Grid A* 生成的 DP Map 作为启发函数
 * - Reed-Shepp 解析扩展：在搜索过程中尝试直接连接终点
 * - 速度规划：使用 Piecewise Jerk 优化生成平滑的速度曲线
 * - 轨迹分段：根据行驶方向（前进/倒车）对轨迹进行分段处理
 */

#pragma once

#include "common/math/math_utils.h"
#include "configs/vehicle_config.h"
#include "configs/vehicle_config_helper.h"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
//#include "common/obstacle.h"
#include "coarse_trajectory_generator/grid_search.h"
#include "coarse_trajectory_generator/node3d.h"
#include "coarse_trajectory_generator/reeds_shepp_path.h"
#include "configs/planner_open_space_config.h"
#include "planning_data/path/discretized_path.h"
#include "planning_data/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include "planning_data/speed/speed_data.h"

/**
 * @struct HybridAStartResult
 * @brief Hybrid A* 规划结果结构体
 *
 * 存储 Hybrid A* 算法规划出的完整轨迹信息，包括：
 * - 位置序列 (x, y)：车辆在世界坐标系中的位置（单位：米）
 * - 航向角序列 (phi)：车辆朝向角（单位：弧度）
 * - 速度序列 (v)：车辆速度（单位：m/s），正值前进，负值倒车
 * - 加速度序列 (a)：车辆加速度（单位：m/s²）
 * - 转向角序列 (steer)：前轮转向角（单位：弧度）
 * - 累计里程 (accumulated_s)：从起点到各点的累计行驶距离
 */
struct HybridAStartResult {
  std::vector<double> x;             ///< x 坐标序列（m）
  std::vector<double> y;             ///< y 坐标序列（m）
  std::vector<double> phi;           ///< 航向角序列（rad），即车辆朝向
  std::vector<double> v;             ///< 速度序列（m/s），正值前进，负值倒车
  std::vector<double> a;             ///< 加速度序列（m/s²）
  std::vector<double> steer;         ///< 前轮转向角序列（rad）
  std::vector<double> accumulated_s; ///< 累计里程（m），从起点到当前点的弧长
};

/**
 * @class HybridAStar
 * @brief Hybrid A* 路径规划器类
 *
 * Hybrid A* 算法是一种结合离散栅格搜索与连续状态空间的路径规划算法，
 * 专门用于满足车辆运动学约束的路径规划场景（如自动泊车）。
 *
 * 算法流程：
 * 1. 初始化：构建障碍物线段表示，生成起点和终点的 Node3d
 * 2. 生成 DP Map：使用 Grid A* (Dijkstra) 计算地图上每点到终点的启发代价
 * 3. Hybrid A* 主循环：
 *    - 从优先队列中取出代价最小的节点
 *    - 尝试 Reed-Shepp 解析扩展直接连接终点
 *    - 若无法直接连接，则按运动学模型扩展邻居节点
 *    - 计算各节点的 g（轨迹代价）+ h（启发代价）并加入 open set
 * 4. 路径回溯：从终点回溯至起点，拼接完整轨迹
 * 5. 速度规划：使用 Piecewise Jerk 优化生成平滑的速度、加速度曲线
 *
 * 主要特点：
 * - 考虑车辆运动学约束（最小转弯半径、前进/倒车）
 * - 使用 Reed-Shepp 曲线加速搜索收敛
 * - 支持轨迹分段（按行驶方向切分）以便后续控制
 */
class HybridAStar {
public:
  /**
   * @brief 构造函数
   * @param open_space_conf 开放空间规划器配置，包含网格分辨率、惩罚系数等参数
   */
  explicit HybridAStar(const PlannerOpenSpaceConfig &open_space_conf);
  virtual ~HybridAStar() = default;

  /**
   * @brief 执行 Hybrid A* 路径规划
   * @param sx 起点 x 坐标（m）
   * @param sy 起点 y 坐标（m）
   * @param sphi 起点航向角（rad）
   * @param ex 终点 x 坐标（m）
   * @param ey 终点 y 坐标（m）
   * @param ephi 终点航向角（rad）
   * @param XYbounds 地图边界 {xmin, xmax, ymin, ymax}（m）
   * @param obstacles_vertices_vec 障碍物顶点集合，每个障碍物由多边形顶点表示
   * @param result 输出参数，规划结果（包含轨迹、速度等信息）
   * @return true 规划成功，false 规划失败
   */
  bool Plan(double sx, double sy, double sphi, double ex, double ey,
            double ephi, const std::vector<double> &XYbounds,
            const std::vector<std::vector<common::math::Vec2d>>
                &obstacles_vertices_vec,
            HybridAStartResult *result);

  /**
   * @brief 按行驶方向对轨迹进行分段
   * @param result 完整的规划结果
   * @param partitioned_result 输出参数，分段后的轨迹集合
   * @return true 分段成功，false 分段失败
   *
   * 根据车辆行驶方向（前进/倒车）将完整轨迹切分为多个子轨迹，
   * 便于后续控制器按段执行。每段轨迹内行驶方向一致。
   */
  bool TrajectoryPartition(const HybridAStartResult &result,
                           std::vector<HybridAStartResult> *partitioned_result);

private:
  /**
   * @brief Reed-Shepp 解析扩展
   * @param current_node 当前节点
   * @return true 成功找到一条无碰撞的 RS 曲线连接到终点，false 否则
   *
   * 尝试从当前节点直接使用 Reed-Shepp 曲线连接到终点。
   * 若成功且路径无碰撞，则设置 final_node_ 并返回 true，搜索结束。
   */
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);

  /**
   * @brief 碰撞与边界有效性检查
   * @param node 待检查的节点
   * @return true 节点有效（无碰撞且在边界内），false 否则
   *
   * 遍历节点包含的离散轨迹点，检查：
   * - 是否超出地图边界 (XYbounds_)
   * - 车辆包围盒是否与障碍物线段相交
   */
  bool ValidityCheck(std::shared_ptr<Node3d> node);

  /**
   * @brief Reed-Shepp 路径碰撞检查
   * @param reeds_shepp_to_end RS 路径
   * @return true 路径有效，false 路径有碰撞
   */
  bool RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end);

  /**
   * @brief 将 Reed-Shepp 路径加载为节点并加入闭集
   * @param reeds_shepp_to_end RS 路径
   * @param current_node 当前父节点
   * @return 生成的终点节点指针
   */
  std::shared_ptr<Node3d>
  LoadRSPinCS(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
              std::shared_ptr<Node3d> current_node);

  /**
   * @brief 生成下一个邻居节点
   * @param current_node 当前节点
   * @param next_node_index 邻居索引 (0 ~ next_node_num_-1)
   * @return 生成的邻居节点指针，若超出边界则返回 nullptr
   *
   * 根据车辆运动学模型，以不同的转向角和行驶方向（前进/倒车）
   * 扩展当前节点，生成候选的下一节点。
   * - 索引 [0, next_node_num_/2) 对应前进方向
   * - 索引 [next_node_num_/2, next_node_num_) 对应倒车方向
   */
  std::shared_ptr<Node3d>
  Next_node_generator(std::shared_ptr<Node3d> current_node,
                      size_t next_node_index);

  /**
   * @brief 计算节点代价
   * @param current_node 当前节点（父节点）
   * @param next_node 下一节点
   *
   * 计算 next_node 的总代价 f = g + h：
   * - g（轨迹代价）：父节点的 g + 边代价（TrajCost）
   * - h（启发代价）：基于 DP Map 的启发式估计
   */
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);

  /**
   * @brief 计算从当前节点到下一节点的轨迹代价
   * @param current_node 当前节点
   * @param next_node 下一节点
   * @return 轨迹代价（包含行驶距离、换挡惩罚、转向惩罚等）
   */
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);

  /**
   * @brief 计算启发式代价
   * @param next_node 目标节点
   * @return 启发代价（基于 DP Map 查表）
   */
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);

  /**
   * @brief 回溯搜索结果，生成完整轨迹
   * @param result 输出参数，规划结果
   * @return true 成功，false 失败
   */
  bool GetResult(HybridAStartResult *result);

  /**
   * @brief 生成速度剖面（时间-速度曲线）
   * @param result 输入输出参数，填充 v, a, steer 等
   * @return true 成功，false 失败
   */
  bool GetTemporalProfile(HybridAStartResult *result);

  /**
   * @brief 使用差分法计算速度和加速度
   * @param result 输入输出参数
   * @return true 成功，false 失败
   */
  bool GenerateSpeedAcceleration(HybridAStartResult *result);

  /**
   * @brief 使用 S 曲线（Piecewise Jerk）优化生成速度和加速度
   * @param result 输入输出参数
   * @return true 成功，false 失败
   *
   * 相比简单差分法，S 曲线优化能生成更平滑的速度曲线，
   * 减少加加速度（jerk）突变，提升乘坐舒适性。
   */
  bool GenerateSCurveSpeedAcceleration(HybridAStartResult *result);

private:
  // ==================== 配置与车辆参数 ====================
  PlannerOpenSpaceConfig planner_open_space_config_; ///< 开放空间规划器配置
  VehicleParam vehicle_param_;                       ///< 车辆参数（尺寸、转向比等）

  // ==================== 搜索参数（从配置中读取） ====================
  size_t next_node_num_ = 0;          ///< 每个节点扩展的邻居数量（前进+倒车）
  double max_steer_angle_ = 0.0;      ///< 最大前轮转向角（rad）
  double step_size_ = 0.0;            ///< 每步行驶距离（m）
  double xy_grid_resolution_ = 0.0;   ///< 栅格分辨率（m）
  double delta_t_ = 0.0;              ///< 速度计算的时间步长（s）

  // ==================== 轨迹代价惩罚系数 ====================
  double traj_forward_penalty_ = 0.0;       ///< 前进行驶惩罚
  double traj_back_penalty_ = 0.0;          ///< 倒车行驶惩罚
  double traj_gear_switch_penalty_ = 0.0;   ///< 换挡惩罚
  double traj_steer_penalty_ = 0.0;         ///< 转向角大小惩罚
  double traj_steer_change_penalty_ = 0.0;  ///< 转向角变化惩罚
  // ==================== RS 启发式惩罚系数（暂未使用） ====================
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;

  // ==================== 搜索状态 ====================
  std::vector<double> XYbounds_;       ///< 地图边界 {xmin, xmax, ymin, ymax}
  std::shared_ptr<Node3d> start_node_; ///< 起点节点
  std::shared_ptr<Node3d> end_node_;   ///< 终点节点
  std::shared_ptr<Node3d> final_node_; ///< 最终到达的节点（搜索结束后用于回溯）
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;     ///< 障碍物边界线段集合

  /**
   * @struct cmp
   * @brief 优先队列比较器
   *
   * 用于 std::priority_queue，使得 top() 返回代价最小的元素（min-heap）。
   * 注意：使用 > 而非 >= 以满足严格弱序要求。
   */
  struct cmp {
    bool operator()(const std::pair<std::string, double> &left,
                    const std::pair<std::string, double> &right) const {
      return left.second > right.second;
    }
  };

  // ==================== A* 搜索数据结构 ====================
  /**
   * @brief open_pq_ - 优先队列，按节点代价（f = g + h）排序
   *
   * 存储 <节点索引, 代价> 的 pair，top() 返回当前代价最小的节点。
   * 每次从此队列中取出代价最小的节点进行扩展。
   */
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;

  /**
   * @brief open_set_ - 开放集，存储所有待扩展的节点
   *
   * 键为节点的字符串索引（x_y_phi），值为节点指针。
   * 用于快速查找节点是否已在开放集中，并获取其 Node3d 对象。
   */
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;

  /**
   * @brief close_set_ - 封闭集，存储所有已扩展过的节点
   *
   * 节点被扩展后从 open_set_ 移入 close_set_，避免重复扩展。
   */
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;

  // ==================== 子模块 ====================
  std::unique_ptr<ReedShepp> reed_shepp_generator_;  ///< Reed-Shepp 曲线生成器
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_; ///< Grid A* 启发函数生成器
};
