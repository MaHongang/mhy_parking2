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

/*
 * @file
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/math/box2d.h"
#include "configs/planner_open_space_config.h"
#include "configs/vehicle_config.h"

// Node3d: Hybrid A* 中用于表示三维状态节点的类
// 每个 Node3d 表示车辆在平面上的一个状态，包括位置 (x,y) 和航向 phi，
// 同时保存从父节点到该节点所经过的离散轨迹（traversed_*），用于碰撞检测与路径回溯。
class Node3d {
public:
  // 构造函数：仅用单个点构造（x,y,phi）
  Node3d(const double x, const double y, const double phi);

  // 构造函数：使用坐标和地图边界、配置构造，计算栅格索引
  // XYbounds: {xmin, xmax, ymin, ymax}
  // open_space_conf 用于提供网格分辨率等参数
  Node3d(const double x, const double y, const double phi,
         const std::vector<double> &XYbounds,
         const PlannerOpenSpaceConfig &open_space_conf);

  // 构造函数：使用一段离散轨迹 (traversed_x/y/phi) 构造节点，节点状态为轨迹最后一个点
  // 常用于由运动学扩展生成的 next_node
  Node3d(const std::vector<double> &traversed_x, // 路径的离散点序列
         const std::vector<double> &traversed_y,
         const std::vector<double> &traversed_phi,
         const std::vector<double> &XYbounds,
         const PlannerOpenSpaceConfig &open_space_conf);

  virtual ~Node3d() = default;

  // 计算给定车辆参数与状态下的二维包围盒（用于碰撞检测）
  // 返回值 Box2d 表示车辆在 (x,y,phi) 处的有向包围矩形
  static common::math::Box2d GetBoundingBox(const VehicleParam &vehicle_param_,
                                            const double x, const double y,
                                            const double phi);

  // cost = traj_cost_ + heuristic_cost_，表示到达该节点的总代价（g + h）
  double GetCost() const { return traj_cost_ + heuristic_cost_; }

  // 返回轨迹代价 g
  double GetTrajCost() const { return traj_cost_; }
  // 返回启发代价 h
  double GetHeuCost() const { return heuristic_cost_; }

  // 网格化后的索引（在 Grid/DP map 中使用）
  int GetGridX() const { return x_grid_; }
  int GetGridY() const { return y_grid_; }
  int GetGridPhi() const { return phi_grid_; }

  // 连续空间的状态量
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }

  bool operator==(const Node3d &right) const;

  // 唯一字符串索引（格式: xGrid_yGrid_phiGrid）
  const std::string &GetIndex() const { return index_; }

  // 该节点所包含的离散轨迹点数量（traversed_x_.size()）
  size_t GetStepSize() const { return step_size_; }

  // 行驶方向：true 表示前进，false 表示倒车
  bool GetDirec() const { return direction_; }

  // 转向角（前轮或车辆状态下的 steering 值）
  double GetSteer() const { return steering_; }

  // 父节点指针（用于回溯整条路径）
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }

  // 访问该节点包含的离散轨迹（用于碰撞检测与轨迹拼接）
  const std::vector<double> &GetXs() const { return traversed_x_; }
  const std::vector<double> &GetYs() const { return traversed_y_; }
  const std::vector<double> &GetPhis() const { return traversed_phi_; }

  // 设置父节点
  void SetPre(std::shared_ptr<Node3d> pre_node) { pre_node_ = pre_node; }
  // 设置行驶方向（前进/倒车）
  void SetDirec(bool direction) { direction_ = direction; }
  // 设置轨迹代价 g
  void SetTrajCost(double cost) { traj_cost_ = cost; }
  // 设置启发代价 h
  void SetHeuCost(double cost) { heuristic_cost_ = cost; }
  // 设置转向角
  void SetSteer(double steering) { steering_ = steering; }

private:
  // 将栅格坐标转换为字符串索引（x_y_phi）
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid);

private:
  // 连续状态量：车辆在平面上的位置与朝向
  double x_ = 0.0;   // x (m)
  double y_ = 0.0;   // y (m)
  double phi_ = 0.0; // 航向角 φ (rad)

  // 该节点对应的离散轨迹点数量（traversed_* 的长度），至少为 1
  size_t step_size_ = 1;
  // traversed_*：从父节点到当前节点所经过的离散轨迹点集合（用于碰撞检测与回溯）
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;

  // 网格化后的索引（基于配置中的栅格分辨率计算）
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;

  // 唯一字符串索引
  std::string index_;

  // 代价量
  double traj_cost_ = 0.0;      // g: 从起点到该节点的轨迹代价
  double heuristic_cost_ = 0.0; // h: 启发式代价（例如基于 A* 的 DP map）
  double cost_ = 0.0;           // 保留字段（未必使用）

  // 父节点指针
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  // 转向角（用于轨迹生成时记录所用的转向）
  double steering_ = 0.0;
  // true for moving forward and false for moving backward
  bool direction_ = true;
};
