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
 * @file reeds_shepp_path.h
 * @brief Reed-Shepp 曲线生成器
 *
 * Reed-Shepp 曲线是一种用于计算满足车辆运动学约束的最短路径的数学方法。
 * 它由三种基本运动原语组成：
 * - L (Left): 以最大曲率左转
 * - R (Right): 以最大曲率右转
 * - S (Straight): 直线行驶
 *
 * 车辆可以前进或倒车，因此实际上有 6 种运动原语（L+, L-, R+, R-, S+, S-）。
 * Reed-Shepp 证明了任意两点之间的最短路径一定是这些原语的有限组合，
 * 且组合形式只有 48 种（可归纳为若干 word 类型：CSC, CCC, CCCC, CCSC, CCSCC 等）。
 *
 * 本类实现：
 * - 枚举所有可能的 RS 曲线组合
 * - 计算各组合的路径长度
 * - 选择最短的有效路径
 * - 对选中路径进行插值，生成离散的轨迹点
 */

#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <omp.h>

#include "coarse_trajectory_generator/node3d.h"
#include "common/math/math_utils.h"
#include "configs/planner_open_space_config.h"
#include "configs/vehicle_config.h"

/**
 * @struct ReedSheppPath
 * @brief Reed-Shepp 路径结构体
 *
 * 存储一条完整的 Reed-Shepp 路径信息，包括：
 * - 各段的长度与类型（L/R/S）
 * - 插值后的离散轨迹点 (x, y, φ)
 * - 各点的行驶方向（前进/倒车）
 */
struct ReedSheppPath {
  std::vector<double> segs_lengths; ///< 各段的弧长（归一化后，乘以 1/max_kappa 得实际长度）
  std::vector<char> segs_types;     ///< 各段类型：'L'=左转, 'R'=右转, 'S'=直行
  double total_length = 0.0;        ///< 总路径长度（归一化）
  std::vector<double> x;            ///< 插值后的 x 坐标序列（m）
  std::vector<double> y;            ///< 插值后的 y 坐标序列（m）
  std::vector<double> phi;          ///< 插值后的航向角序列（rad）
  std::vector<bool> gear;           ///< 行驶方向：true=前进, false=倒车
};

/**
 * @struct RSPParam
 * @brief Reed-Shepp 路径参数
 *
 * 用于存储单条 RS 曲线的计算结果：
 * - flag: 该组合是否有效
 * - t, u, v: 三段运动原语的长度参数（对于某些 word 类型可能只用两段）
 */
struct RSPParam {
  bool flag = false;  ///< 该组合是否有效（几何上可行）
  double t = 0.0;     ///< 第一段长度
  double u = 0.0;     ///< 第二段长度
  double v = 0.0;     ///< 第三段长度
};

/**
 * @class ReedShepp
 * @brief Reed-Shepp 曲线生成器
 *
 * 计算从起点到终点的最短 Reed-Shepp 路径。
 * 支持车辆前进和倒车，考虑最大转向角限制。
 *
 * 使用方法：
 * @code
 * ReedShepp rs_generator(vehicle_param, open_space_conf);
 * auto path = std::make_shared<ReedSheppPath>();
 * if (rs_generator.ShortestRSP(start_node, end_node, path)) {
 *     // path 中包含最短路径的离散轨迹点
 * }
 * @endcode
 */
class ReedShepp {
public:
  /**
   * @brief 构造函数
   * @param vehicle_param 车辆参数（轴距、最大转向角等）
   * @param open_space_conf 开放空间规划配置
   */
  ReedShepp(const VehicleParam &vehicle_param,
            const PlannerOpenSpaceConfig &open_space_conf);
  virtual ~ReedShepp() = default;

  /**
   * @brief 计算从起点到终点的最短 Reed-Shepp 路径
   * @param start_node 起点节点（包含 x, y, φ）
   * @param end_node 终点节点（包含 x, y, φ）
   * @param optimal_path 输出参数，最短路径
   * @return true 成功找到路径，false 失败
   *
   * 该方法会枚举所有可能的 RS 曲线组合，选择总长度最短的有效路径，
   * 并对其进行插值生成离散轨迹点。
   */
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::shared_ptr<ReedSheppPath> optimal_path);

protected:
  /**
   * @brief 生成所有可能的 RS 路径组合
   * @param start_node 起点
   * @param end_node 终点
   * @param all_possible_paths 输出参数，所有可能的路径
   * @return true 成功，false 失败
   */
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief 生成 RS 路径的通用配置（串行版本）
   * @details 枚举 6 种 word 类型：SCS, CSC, CCC, CCCC, CCSC, CCSCC
   */
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief 生成 RS 路径的通用配置（并行版本）
   */
  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief 对选定的 RS 路径进行插值，生成离散轨迹点
   * @param start_node 起点
   * @param end_node 终点
   * @param shortest_path 输入输出参数，将插值结果填充到 x/y/phi/gear
   * @return true 成功，false 失败
   */
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d> start_node,
                                   const std::shared_ptr<Node3d> end_node,
                                   ReedSheppPath *shortest_path);

  /**
   * @brief 对单段运动原语进行插值
   * @param index 当前段索引
   * @param pd 段长度
   * @param m 段类型 ('L', 'R', 'S')
   * @param ox, oy, ophi 段起点的世界坐标和航向
   * @param px, py, pphi, pgear 输出参数，插值后的轨迹点
   */
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double> *px, std::vector<double> *py,
                     std::vector<double> *pphi, std::vector<bool> *pgear);

  /**
   * @brief 设置单条 RS 路径
   * @param size 段数
   * @param lengths 各段长度数组
   * @param types 各段类型字符串
   * @param all_possible_paths 输出，添加到可能路径集合
   */
  bool SetRSP(const int size, const double *lengths, const char *types,
              std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief 设置单条 RS 路径（并行版本）
   */
  bool SetRSPPar(const int size, const double *lengths,
                 const std::string &types,
                 std::vector<ReedSheppPath> *all_possible_paths, const int idx);

  // ==================== 6 种 RS 曲线 Word 类型 ====================
  // 每种 word 类型对应多种具体的组合形式

  /**
   * @brief SCS 类型：直行-转弯-直行
   * @details 包含 SLS, SRS 两种组合
   */
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief CSC 类型：转弯-直行-转弯
   * @details 包含 LSL, LSR, RSL, RSR 等组合
   */
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief CCC 类型：转弯-转弯-转弯
   * @details 包含 LRL, RLR 等组合
   */
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief CCCC 类型：四段转弯
   * @details 包含 LRLR, RLRL 等组合
   */
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief CCSC 类型：三转一直
   * @details 包含 LRSL, LRSR, RLSL, RLSR 等组合
   */
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath> *all_possible_paths);

  /**
   * @brief CCSCC 类型：两转一直两转
   * @details 包含 LRSLR, RLSRL 等组合
   */
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath> *all_possible_paths);

  // ==================== 基本运动原语求解函数 ====================
  // 这些函数计算特定组合的几何参数 (t, u, v)

  void LSL(const double x, const double y, const double phi, RSPParam *param);
  void LSR(const double x, const double y, const double phi, RSPParam *param);
  void LRL(const double x, const double y, const double phi, RSPParam *param);
  void SLS(const double x, const double y, const double phi, RSPParam *param);
  void LRLRn(const double x, const double y, const double phi, RSPParam *param);
  void LRLRp(const double x, const double y, const double phi, RSPParam *param);
  void LRSR(const double x, const double y, const double phi, RSPParam *param);
  void LRSL(const double x, const double y, const double phi, RSPParam *param);
  void LRSLR(const double x, const double y, const double phi, RSPParam *param);

  /**
   * @brief 辅助函数：计算 tau 和 omega 参数
   * @details 用于某些 RS 曲线组合的几何计算
   */
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

protected:
  VehicleParam vehicle_param_;                       ///< 车辆参数
  PlannerOpenSpaceConfig planner_open_space_config_; ///< 规划配置
  double max_kappa_;  ///< 最大曲率（1/最小转弯半径），用于归一化计算
};
