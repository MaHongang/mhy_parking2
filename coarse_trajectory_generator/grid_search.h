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

/*
 * @file
 */

#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h" // 安装：https://abseil.io/docs/cpp/quickstart-cmake.html
#include "common/math/line_segment2d.h"
#include "configs/planner_open_space_config.h"

// Node2d: 在 Grid (二维) 上的 A* 网格节点表示
// 仅包含网格坐标、当前路径代价(path_cost_, 即 g)、启发代价(heuristic_, 即 h)
// 以及用于回溯的父节点指针。该类在 GridSearch 中用于构建栅格图并运行 A*。
class Node2d {
public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double> &XYbounds) {
    // XYbounds with 0xmin, 1xmax, 2ymin, 3ymax
    grid_x_ =
        static_cast<int>((x - XYbounds[0]) / xy_resolution); // x方向的索引
    grid_y_ =
        static_cast<int>((y - XYbounds[2]) / xy_resolution); // y方向的索引
    index_ = ComputeStringIndex(grid_x_, grid_y_); //字符串类型的索引值x_y
  }
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double> &XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;//更新g的同时，更新f   f=g+h
  }
  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double cost) { cost_ = cost; }
  // 设置父节点（用于路径回溯）
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  // 访问器：返回网格索引（整数），注意原实现返回 double，这里保持原样但语义为 int
  double GetGridX() const { return grid_x_; };
  double GetGridY() const { return grid_y_; };
  // 返回当前节点的 g 值（路径代价）
  double GetPathCost() const { return path_cost_; }
  // 返回当前节点的启发式代价 h
  double GetHeuCost() const { return heuristic_; }
  // 返回总代价 f = g + h
  double GetCost() const { return cost_; }
  // 节点的唯一字符串索引（例如 "10_5"）用于哈希表 key
  const std::string &GetIndex() const { return index_; }
  // 返回父节点指针
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
  static std::string CalcIndex(const double x, const double y,
                               const double xy_resolution,
                               const std::vector<double> &XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d &right) const {
    return right.GetIndex() == index_;//重载== 定义两个节点相等/相同
  }

private:
  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return absl::StrCat(x_grid, "_", y_grid);
  }

private:
//可以拓展为 f=g+a*h
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;//g
  double heuristic_ = 0.0;//h
  double cost_ = 0.0;//f
  std::string index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

// Grid A* 的输出结果：一系列连续空间坐标（x,y）和估计的路径代价
struct GridAStartResult {
  std::vector<double> x; // 以连续坐标（m）表示的路径点 x
  std::vector<double> y; // 以连续坐标（m）表示的路径点 y
  double path_cost = 0.0; // 基于栅格计数的估计路径代价（稍后乘以分辨率可得到米为单位的长度）
};

// GridSearch: 基于栅格的 A* 搜索器，提供两类能力：
// 1) GenerateAStarPath: 在有障碍物的栅格地图中从起点到终点生成一条粗略路径（以栅格为单位）
// 2) GenerateDpMap: 以终点为根，计算地图上每个可达栅格到终点的最短代价（DP 表），用于作为启发式函数
class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig &open_space_conf);
  virtual ~GridSearch() = default;

  // 在 XYbounds 指定的区域上，以网格分辨率 xy_grid_resolution_ 为单位运行 A*。
  // obstacles_linesegments_vec 为由线段表示的障碍物集合（每个障碍物由若干线段组成）。
  // 结果写入 result（连续坐标，单位 m）。返回 true 表示找到路径。
  bool GenerateAStarPath(const double sx, const double sy, const double ex,
                         const double ey, const std::vector<double> &XYbounds,
                         const std::vector<std::vector<common::math::LineSegment2d>>
                             &obstacles_linesegments_vec,
                         GridAStartResult *result);

  // 生成一个从终点到地图上所有点的 DP 映射（最短代价表），用于启发式估价。
  bool GenerateDpMap(const double ex, const double ey,
                     const std::vector<double> &XYbounds,
                     const std::vector<std::vector<common::math::LineSegment2d>>
                         &obstacles_linesegments_vec);

  // 查询 DP 表中某点到终点的代价值（单位为栅格数 * xy_grid_resolution_），找不到则返回 +inf
  double CheckDpMap(const double sx, const double sy);

 private:
  // 欧氏距离，栅格间距离用于启发式
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);

  // 从当前栅格生成 8 邻域的候选下一个栅格节点（包含对角线），并设置其 path_cost
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(std::shared_ptr<Node2d> node);

  // 边界检测与障碍检测，返回 true 表示该网格可达（未碰撞且在边界内）
  bool CheckConstraints(std::shared_ptr<Node2d> node);

  // 将最终搜索树回溯为 GridAStartResult（连续空间路径）
  void LoadGridAStarResult(GridAStartResult *result);

 private:
  double xy_grid_resolution_ = 0.0; // 网格分辨率（m）
  double node_radius_ = 0.0;       // 节点碰撞判断半径（m）
  std::vector<double> XYbounds_;    // 地图边界 {xmin, xmax, ymin, ymax}
  double max_grid_x_ = 0.0;         // x 方向最大网格索引
  double max_grid_y_ = 0.0;         // y 方向最大网格索引

  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_; // 最终回溯得到的终点节点（如果找到）

  std::vector<std::vector<common::math::LineSegment2d>> obstacles_linesegments_vec_;

  // 优先队列比较器（用于存放 <index, cost> 配对，使得 top() 得到最小代价）
  struct cmp {
    // 比较器：用于优先队列，使得 top() 返回 cost 最小的元素（min-heap 行为）。
    // 注意：使用 "">" 而不是 ">="，以满足严格弱序 (strict weak ordering) 的要求。
    //operator()(left, right) 为 true 表示 “left 应该位于 right 之后（优先级更低）”。
    //priority_queue 会据此构建堆，使得 top() 是在比较意义上“最大”的元素
    bool operator()(const std::pair<std::string, double> &left,
                    const std::pair<std::string, double> &right) const noexcept {
      return left.second > right.second;
    }
  };

  // DP 映射：索引 -> 节点（用于 GenerateDpMap 的查表）
  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};
 