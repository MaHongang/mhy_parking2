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

#include "coarse_trajectory_generator/grid_search.h"

// 构造函数：从配置中读取网格分辨率和节点碰撞半径
GridSearch::GridSearch(const PlannerOpenSpaceConfig &open_space_conf) {
  xy_grid_resolution_ =
      open_space_conf.warm_start_config.grid_a_star_xy_resolution;
  node_radius_ = open_space_conf.warm_start_config.node_radius; // 节点碰撞判断半径（m）
}

// 计算两点之间的欧式距离（用于启发式估计）
double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// 边界与障碍检测：判断给定节点是否超出地图边界或与障碍线段过近
bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  // 边界检查：如果网格索引超出 [0, max_grid_x_/y] 范围，则不可达
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  // 若无障碍物定义，则直接认为可达
  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }
  
  // 将网格索引转换为世界坐标（障碍物是以世界坐标定义的）
  const double node_world_x = node_grid_x * xy_grid_resolution_ + XYbounds_[0];
  const double node_world_y = node_grid_y * xy_grid_resolution_ + XYbounds_[2];
  
  // 对所有障碍物线段做最近距离判断，若小于 node_radius_ 则视为碰撞
  for (const auto &obstacle_linesegments : obstacles_linesegments_vec_) {
    for (const common::math::LineSegment2d &linesegment :
         obstacle_linesegments) {
      if (linesegment.DistanceTo({node_world_x, node_world_y}) < node_radius_) {
        return false;
      }
    }
  }
  return true;
}

// 从当前栅格扩展 8 个相邻栅格（上下左右及四个对角线），并为每个新节点设置 path_cost。
// 注意：这里的坐标（current_node_x/y）是网格索引而非连续坐标，步长为 1 或 sqrt(2)
std::vector<std::shared_ptr<Node2d>>
GridSearch::GenerateNextNodes(std::shared_ptr<Node2d> current_node) {
  double current_node_x = current_node->GetGridX();
  double current_node_y = current_node->GetGridY();
  double current_node_path_cost = current_node->GetPathCost();
  double diagonal_distance = std::sqrt(2.0); // 对角线移动的代价（网格单位）
  std::vector<std::shared_ptr<Node2d>> next_nodes; // 顺时针扩展 8 个点
  std::shared_ptr<Node2d> up =
      std::make_shared<Node2d>(current_node_x, current_node_y + 1.0, XYbounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y + 1.0, XYbounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + 1.0, current_node_y, XYbounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + 1.0, current_node_y - 1.0, XYbounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down =
      std::make_shared<Node2d>(current_node_x, current_node_y - 1.0, XYbounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y - 1.0, XYbounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - 1.0, current_node_y, XYbounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - 1.0, current_node_y + 1.0, XYbounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridSearch::GenerateAStarPath(
    const double sx, const double sy, const double ex, const double ey,
    const std::vector<double> &XYbounds,
    const std::vector<std::vector<common::math::LineSegment2d>>
        &obstacles_linesegments_vec,
    GridAStartResult *result) {
  // open_pq 中存储 <node_index, cost>，cmp 比较器按 cost 从小到大（优先队列的 top 为最小代价）
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  // open_set / close_set 存放索引->Node 指针，方便快速查找与更新
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set; // 索引和节点指针
  std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;

  // 保存边界与障碍物信息到成员变量，便于在类内使用
  XYbounds_ = XYbounds;
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;

  // 计算网格尺寸（最大索引值）
  // XYbounds_: [xmin, xmax, ymin, ymax]
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);

  // 起点 / 终点：Node2d 构造函数会将连续坐标转换为网格索引
  std::shared_ptr<Node2d> start_node =
      std::make_shared<Node2d>(sx, sy, xy_grid_resolution_, XYbounds_); // 起点
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_); // 终点

  final_node_ = nullptr; // 重置成员变量，最终找到的到达终点的节点（可能与 end_node 不同）

  // 将起点加入 open_set 与 open_pq，cost 已在 Node2d 内计算（通常为 0）
  open_set.emplace(start_node->GetIndex(), start_node);
  open_pq.emplace(start_node->GetIndex(), start_node->GetCost());

  // A* 主循环：从 open_pq 中取出当前最小总代价节点，扩展其邻居
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];

    // 到达终点判断：Node2d 已自定义 operator==，用于近似判断坐标是否相同
    if (*(current_node) == *(end_node)) {
      final_node_ = current_node;
      break;
    }

    // 将当前节点放入闭集（表示已展开）
    close_set.emplace(current_node->GetIndex(), current_node);

    // 生成当前节点的 8 邻域节点
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));

    for (auto &next_node : next_nodes) {
      // 越界或碰撞检查
      if (!CheckConstraints(next_node)) {
        continue;
      }
      // 如果已在闭集则跳过
      if (close_set.find(next_node->GetIndex()) != close_set.end()) {
        continue;
      }

      // 若不在 open_set 中，则按常规处理并加入 open_set
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        // 计算启发式（与终点的欧式距离）并设置父节点
        next_node->SetHeuristic(
            EuclidDistance(next_node->GetGridX(), next_node->GetGridY(),
                           end_node->GetGridX(), end_node->GetGridY()));
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        // 若已存在 open_set，理论上应检查并更新更优路径（此处原实现注释掉该分支）
        // 保留当前位置策略：如果需要更严格的 A* 可在此比较并更新
      }
    }
  }

  if (final_node_ == nullptr) {
    std::cout << "[Error] Grid A* failed" << std::endl;
    return false;
  }

  // 将找到的路径回填到结果结构
  LoadGridAStarResult(result);
  return true;
}
//主要
bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::vector<double> &XYbounds,
    const std::vector<std::vector<common::math::LineSegment2d>>
        &obstacles_linesegments_vec) {
  // DP Map 反向搜索：从终点向外扩展，记录每个网格到终点的最短代价
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;

  dp_map_ = decltype(dp_map_)();
  XYbounds_ = XYbounds;

  // 计算网格尺寸（最大索引值）
  // XYbounds_: [xmin, xmax, ymin, ymax]
  max_grid_y_ = std::round((XYbounds_[3] - XYbounds_[2]) / xy_grid_resolution_);
  max_grid_x_ = std::round((XYbounds_[1] - XYbounds_[0]) / xy_grid_resolution_);

  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, XYbounds_);
  obstacles_linesegments_vec_ = obstacles_linesegments_vec;

  // 初始化：把终点加入 open_set 与 open_pq，开始反向搜索
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];

    // 将当前节点写入 dp_map_，表示该网格点到终点的最短/已知代价
    dp_map_.emplace(current_node->GetIndex(), current_node);

    // 生成邻居并尝试加入 open_set（或更新已有节点的 cost）
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        std::move(GenerateNextNodes(current_node));
    for (auto &next_node : next_nodes) {
      // 跳过越界或碰撞节点
      if (!CheckConstraints(next_node)) {
        continue;
      }
      // 若已存在于 dp_map_（已被闭合），无需处理
      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      // 若不在 open_set 中，则加入；否则检查并更新更优 cost
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          // 更新更短路径信息
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {//查表
  std::string index = Node2d::CalcIndex(sx, sy, xy_grid_resolution_, XYbounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void GridSearch::LoadGridAStarResult(GridAStartResult *result) {
  // 将 final_node_ 链表回溯生成从起点到终点的连续坐标（以米为单位）
  (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
  std::shared_ptr<Node2d> current_node = final_node_;
  std::vector<double> grid_a_x;
  std::vector<double> grid_a_y;

  // 从终点向前回溯直到起点（pre_node == nullptr）
  while (current_node->GetPreNode() != nullptr) {
    // Node2d 存储的是网格索引，这里转换为世界坐标：grid * resolution + min_bound
    grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ +
                       XYbounds_[0]);
    grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ +
                       XYbounds_[2]);
    current_node = current_node->GetPreNode();
  }

  // 目前路径按从终点到起点压入，故需要翻转为从起点到终点
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);
  (*result).y = std::move(grid_a_y);
}
