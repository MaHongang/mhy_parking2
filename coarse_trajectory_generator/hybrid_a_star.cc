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

#include "coarse_trajectory_generator/hybrid_a_star.h"
#include "planning_data/piecewise_jerk/piecewise_jerk_speed_problem.h"

using common::math::Box2d;
using common::math::Vec2d;

//在构造函数中初始化，初始化获取一些配置
HybridAStar::HybridAStar(const PlannerOpenSpaceConfig &open_space_conf)
    : planner_open_space_config_(open_space_conf) {
  reed_shepp_generator_ = std::make_unique<ReedShepp>(
      vehicle_param_, planner_open_space_config_); //构造RS类指针
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_); //构造A*指针
  //参数分配
  next_node_num_ = //扩展节点的数量
      planner_open_space_config_.warm_start_config.next_node_num;
  //前轮转角8.2/16=0.5rad
  max_steer_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;
  step_size_ =
      planner_open_space_config_.warm_start_config.step_size; //行驶步长???是一个节点的长度吗 md有两个step_size,一个是步长长度，一个是步长数量
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config.xy_grid_resolution;
  delta_t_ = planner_open_space_config_.warm_start_config.delta_t;//求速度的dt
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config.traj_forward_penalty;
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config.traj_back_penalty;
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config.traj_gear_switch_penalty;
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config.traj_steer_penalty;
  traj_steer_change_penalty_ =
      planner_open_space_config_.warm_start_config.traj_steer_change_penalty;
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
      

  if (!reed_shepp_generator_->ShortestRSP(
          current_node, end_node_,
          reeds_shepp_to_check)) { //搜寻最短的RS路径
    return false;
  }
  //检查ReedShep路径是否碰撞或者可行
  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  std::cout << "[Hybrid A*] RS path found" << std::endl;
  // load the whole RSP as nodes and add to the close
  // set//载入RSP作为节点并加入close集合
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}
//判断行驶点是车辆行驶位置是否与障碍物发生碰撞，或者超出边界
bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  // CHECK_NOTNULL(node);
  // CHECK_GT(node->GetStepSize(), 0U);

  if (obstacles_linesegments_vec_.empty()) {
    return true; //没有障碍物直接返回true
  }

  size_t node_step_size = node->GetStepSize();
  const auto &traversed_x = node->GetXs();
  const auto &traversed_y = node->GetYs();
  const auto &traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    //以行驶点为中心构建汽车的OBB包围框
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
    //是否可以取就近的障碍物线段做碰撞检测，这样遍历所有障碍物计算量是否太大
    for (const auto &obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d &linesegment :
           obstacle_linesegments) {
        { //碰撞，包围框是否覆盖障碍物的边
          if (bounding_box.HasOverlap(linesegment)) {
            std::cout << "collision start at x,y: " << linesegment.start().x()
                      << " , " << linesegment.start().y() << std::endl;
            std::cout << "collision end at   x,y: " << linesegment.end().x()
                      << " , " << linesegment.end().y() << std::endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  end_node->SetPre(current_node); // end_node的父节点设置为当前节点
  close_set_.emplace(end_node->GetIndex(), end_node);//结束了,放不放都无所谓吧
  return end_node;
}
// 3D，邻域点生成
std::shared_ptr<Node3d>
HybridAStar::Next_node_generator(std::shared_ptr<Node3d> current_node,
                                 size_t next_node_index) 
                                 {
  double steering = 0.0;
  double traveled_distance = 0.0;
  // steering angle计算原理
  //首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的
  //这里的if-else就是区分运动正反方向讨论的（前进和倒车）
  //其次，车辆在当前的姿态下，既可以向左转、又可以向右转，那么steering angle的
  //取值范围其实是[-max_steer_angle_, max_steer_angle_]，在正向或反向下，
  //能取next_node_num_/2个有效值。
  //即，把[-max_steer_angle_, max_steer_angle_]分为（next_node_num_/2-1）份
  //所以，steering = 初始偏移量 + 单位间隔 × index

  if (next_node_index < static_cast<double>(next_node_num_) / 2) { //正向，前5//next_node_index是不是从0开始 使用时是的
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_; // step_size_ 默认0.25，是每次向前运动的距离,为什么这个step是通过配置文件写死的，不是应该为 v*dt 吗？
  } else { //反向，后5
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid按照上面的运动方向，将车辆行驶向不同的栅格
  double arc =
      std::sqrt(2) *
      xy_grid_resolution_; // arc是根号2倍的栅格分辨率，根号（2）的栅格分辨率，一定能到下一个栅格 1.4*0.4
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);//每个节点的第一个值是上一个节点的最后一个值
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_;//实际上，一个节点就只包含一个离散点？？？不是
       ++i) { //寻找行走超过>栅格根号2倍的距离的作为next_node 运动学公式扩展的不是节点，而是节点包含的离散点，
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary检测是否超出边界
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);//什么意思  判断行驶方向的
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(
      current_node->GetTrajCost() +
      TrajCost(current_node,
               next_node)); //上一节点的轨迹代价+两个节点之间的轨迹代价g
  // evaluate heuristic cost启发cost h
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;//为什么要-1 // 步长个数*步长长度
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult *result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      std::cout << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      std::cout << "states sizes are not equal";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  if (!GetTemporalProfile(result)) {
    std::cout << "GetSpeedProfile from Hybrid Astar path fails" << std::endl;
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    std::cout << "[Error] State size mismatch" << std::endl;
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    std::cout << "[Error] Control size mismatch" << std::endl;
    return false;
  }
  return true;
}
//差分求速度，加速度

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult *result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    std::cout
        << "result size check when generating speed and acceleration fail";
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  //下面这个公式是什么？？？
  //delta_t_在构造函数中初始化，从配置文件中读取的,默认值0.2s,图片中是0.1s，具体如何设置的还不清楚，
  //是否是通过车辆最大加速度和最大速度计算出来的，原始论文pjso中好像涉及到这个公式
  //一个符合直觉的猜测是，由预估总行使时间/总点数 得到的每个点的时间间隔，那么预估总行使时间
  //应该是根据车辆最大速度和最大加速度和距离计算出来的
  //v的个数比x少1，因为v是两个点之间的量，但是这里直接把v的第一个点设为0了也把最后一个点设为0了，会不会有问题
  for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
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
  return true;
}

/**
 * @brief 使用 S 曲线（Piecewise Jerk）优化生成平滑的速度和加速度剖面
 * 
 * 该函数是 Hybrid A* 规划结果的后处理核心模块，主要完成以下任务：
 * 1. 数据验证：检查输入轨迹的有效性
 * 2. 档位识别：判断车辆是前进还是倒车
 * 3. 累计里程计算：沿路径计算弧长 s
 * 4. 时间估算：基于运动学约束估算总行驶时间
 * 5. QP 速度优化：使用 Piecewise Jerk 方法生成平滑的 s-t 曲线
 * 6. 轨迹融合：使用插值将优化后的速度曲线与原始路径融合
 * 
 * 核心思想：
 * - 将路径规划（x-y）和速度规划（s-t）解耦
 * - 在弧长-时间空间中进行速度优化，避免曲率耦合
 * - 最小化 Jerk（加加速度）以保证乘坐舒适性
 * 
 * @param result 输入输出参数，包含 Hybrid A* 的路径结果，输出填充 v, a, steer
 * @return true 成功生成速度曲线，false 失败
 */
bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult *result) {
  // ========================================================================
  // 第一步：数据合法性检查
  // ========================================================================
  // CHECK_NOTNULL(result);
  
  // 检查路径点数量是否足够（至少需要 2 个点才能计算速度）
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    std::cout
        << "result size check when generating speed and acceleration fail";
    return false;
  }
  
  // 检查各状态向量长度是否一致（避免数据不同步）
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    std::cout << "result sizes not equal";
    return false;
  }

  // ========================================================================
  // 第二步：档位（前进/倒车）识别
  // ========================================================================
  // 原理：通过比较车辆航向角与实际行驶方向的夹角来判断档位
  // - 航向角 (heading)：车头指向，即 phi
  // - 行驶方向 (tracking)：从当前点指向下一点的向量
  // - 若夹角 < 90°，则前进（gear = true）
  // - 若夹角 > 90°，则倒车（gear = false）
  
  double init_heading = result->phi.front();  // 起点航向角
  
  // 构造从第一个点到第二个点的向量（实际移动方向）
  const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                   result->y[1] - result->y[0]);
  
  // 计算航向角与移动方向的夹角
  // NormalizeAngle 将角度归一化到 [-π, π]
  // 若夹角绝对值 < π/2，说明车头朝向与运动方向基本一致，判定为前进
  const double gear =
      std::abs(common::math::NormalizeAngle(
          init_heading - init_tracking_vector.Angle())) < M_PI_2;

  // ========================================================================
  // 第三步：计算累计里程 (accumulated_s)
  // ========================================================================
  // 为什么需要重新计算？
  // - Hybrid A* 的节点中虽然包含了位置信息，但可能缺少准确的弧长信息
  // - 速度优化是在弧长-时间 (s-t) 空间进行的，需要精确的 s 作为优化变量
  // - 累计里程是路径积分，用于将位置映射到一维参数空间
  
  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.clear();
  auto last_x = result->x.front();
  auto last_y = result->y.front();
  
  // 遍历所有路径点，累加相邻点之间的欧氏距离
  for (size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    // 累加弧长：s_{i} = s_{i-1} + ||p_i - p_{i-1}||
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }
  
  // ========================================================================
  // 第四步：设置初始状态和物理约束参数
  // ========================================================================
  // 假设车辆从静止状态启动（泊车场景的常见假设）
  const double init_v = 0.0;  // 初始速度 0 m/s
  const double init_a = 0.0;  // 初始加速度 0 m/s²

  // minimum time speed optimization
  // TODO(Jinyun): move to confs
  // 物理约束参数（应从配置文件读取）
  const double max_forward_v = 2.0;      // 前进最大速度 2 m/s
  const double max_reverse_v = 1.0;      // 倒车最大速度 1 m/s
  const double max_forward_acc = 2.0;    // 前进最大加速度 2 m/s²
  const double max_reverse_acc = 1.0;    // 倒车最大加速度 1 m/s²
  const double max_acc_jerk = 0.5;       // 最大 Jerk 0.5 m/s³
  const double delta_t = 0.2;            // 时间离散步长 0.2s

  SpeedData speed_data;

  // ========================================================================
  // 第五步：估算总行驶时间 (total_t)
  // ========================================================================
  // TODO(Jinyun): explore better time horizon heuristic
  
  const double path_length = result->accumulated_s.back();  // 路径总长度
  
  // 【公式推导】最短行驶时间估算
  // 
  // 假设车辆采用"加速-匀速-减速"的梯形速度曲线：
  // 1. 加速阶段：从 0 加速到 v_max，耗时 t_1 = v_max / a_max
  //    行驶距离 s_1 = 0.5 * a_max * t_1² = v_max² / (2 * a_max)
  // 
  // 2. 减速阶段：从 v_max 减速到 0，耗时和距离与加速阶段对称
  //    s_2 = v_max² / (2 * a_max)
  // 
  // 3. 匀速阶段：行驶剩余距离 s_3 = path_length - s_1 - s_2
  //    耗时 t_3 = s_3 / v_max = (path_length - v_max² / a_max) / v_max
  // 
  // 总时间：
  // t_total = t_1 + t_2 + t_3
  //         = 2 * (v_max / a_max) + (path_length - v_max² / a_max) / v_max
  //         = (2 * v_max² + path_length * a_max - v_max²) / (a_max * v_max)
  //         = (v_max² + path_length * a_max) / (a_max * v_max)
  // 
  // 乘以 1.5 是安全系数，避免时间过紧导致优化无解
  // 取 max(..., 10.0) 保证至少有 10 秒的规划时间
  
  const double total_t = std::max(gear ? 1.5 *
                                             (max_forward_v * max_forward_v +
                                              path_length * max_forward_acc) /
                                             (max_forward_acc * max_forward_v)
                                       : 1.5 *
                                             (max_reverse_v * max_reverse_v +
                                              path_length * max_reverse_acc) /
                                             (max_reverse_acc * max_reverse_v),
                                  10.0);

  // 计算时间离散点数量：total_t / Δt，向上取整（+1）
  //也就是说，时间上每隔delta_t=0.2s取一个点，直到total_t结束，
  //num_of_knots的个数并不是混合A*得到的粗轨迹的点数
  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

  // ========================================================================
  // 第六步：构建 Piecewise Jerk QP 优化问题
  // ========================================================================
  // 优化变量：x = [s_0, s_1, ..., s_{n-1}, v_0, v_1, ..., a_0, a_1, ...]
  // 优化目标：min J = w_s·(s-s_ref)² + w_v·v² + w_a·a² + w_j·jerk²
  // 约束条件：运动学约束 + 边界约束
  
  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

  // ------------------------------------------------------------------------
  // 6.1 设置边界约束 (Bounds)
  // ------------------------------------------------------------------------
  // x_bounds: 位置（弧长）的上下界，范围 [0, path_length]
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;      // 根据档位选择速度上限
  const double max_acc = gear ? max_forward_acc : max_reverse_acc;  // 根据档位选择加速度上限

  // 速度上界：取 max_v 和初始速度的较大值（防止初速度超限导致无解）
  // fmax 和 abs 的区别：fmax 是比较两个数的大小，abs 是取绝对值
  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  
  // dx_bounds: 速度边界，范围 [0, upper_dx]
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  
  // ddx_bounds: 加速度边界，范围 [-max_acc, max_acc]
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});

  // ------------------------------------------------------------------------
  // 6.2 设置终点约束 (End Constraints)
  // ------------------------------------------------------------------------
  // 终点必须到达路径末端，且速度和加速度为 0（停车状态）
  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);  // s_end = path_length
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);                 // v_end = 0
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);                // a_end = 0

  // ------------------------------------------------------------------------
  // 6.3 设置参考值和权重 (References & Weights)
  // ------------------------------------------------------------------------
  // TODO(Jinyun): move to confs
  
  // 位置参考：希望车辆尽快到达终点，所有时刻的参考位置都设为 path_length
  // 这样优化器会倾向于生成更快的速度曲线
  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));  // 位置跟踪权重 10000
  
  // 加速度平滑性权重（惩罚大加速度）
  piecewise_jerk_problem.set_weight_ddx(10.0);
  
  // Jerk 平滑性权重（惩罚加加速度，保证舒适性）
  piecewise_jerk_problem.set_weight_dddx(10.0);
  
  // 应用边界约束
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  
  // Jerk 边界（对称上下限）
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);
// std::cout<<"test hybrid_a_star.cc 465"<<std::endl;
  // ========================================================================
  // 第七步：调用 OSQP 求解器
  // ========================================================================
  // solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    std::cout << "Piecewise jerk speed optimizer failed!";
    return false;
  }

  // ========================================================================
  // 第八步：提取优化结果
  // ========================================================================
  // extract output
  const std::vector<double> &s = piecewise_jerk_problem.opt_x();    // 优化后的位置序列
  const std::vector<double> &ds = piecewise_jerk_problem.opt_dx();  // 优化后的速度序列
  const std::vector<double> &dds = piecewise_jerk_problem.opt_ddx(); // 优化后的加速度序列

  // ------------------------------------------------------------------------
  // 8.1 构建速度数据结构 (SpeedData)
  // ------------------------------------------------------------------------
  // SpeedData 是一个时间序列容器，存储 (s, t, v, a, jerk) 五元组
  // 每个点代表在时刻 t，车辆行驶到弧长 s 处的运动状态
  // 
  // 【数据结构说明】
  // SpeedData 内部维护一个按时间排序的速度点序列：
  // - s (m):    累计行驶距离（弧长）
  // - t (s):    时间戳（相对于起始时刻）
  // - v (m/s):  瞬时速度
  // - a (m/s²): 瞬时加速度
  // - jerk (m/s³): 瞬时加加速度（加速度的变化率）
  
  // ------------------------------------------------------------------------
  // 步骤 1：添加起始点
  // ------------------------------------------------------------------------
  // 起点状态：
  // - s[0]: QP 优化得到的初始位置（通常为 0）
  // - t = 0: 时间起点
  // - ds[0]: 初始速度（通常为 0，静止启动）
  // - dds[0]: 初始加速度（通常为 0）
  // - jerk = 0: 初始 jerk 设为 0（因为没有前一时刻可以计算变化率）
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  
  // ------------------------------------------------------------------------
  // 步骤 2：设置数值容忍度
  // ------------------------------------------------------------------------
  // 为什么需要容忍度？
  // - 浮点运算存在舍入误差，严格的相等判断可能失效
  // - 例如：理论上 s[i] >= s[i-1]，但浮点误差可能导致 s[i] = s[i-1] - 1e-15
  // - 使用容忍度可以过滤这种微小的数值噪声
  
  const double kEpislon = 1.0e-6;   // 速度单调性检查的容忍度（1 微米）
  const double sEpislon = 1.0e-6;   // 终点判断的容忍度（1 微米）
  
  // ------------------------------------------------------------------------
  // 步骤 3：遍历时间节点，构建完整的速度剖面
  // ------------------------------------------------------------------------
  // 为什么从 i=1 开始？
  // - i=0 的起始点已经在上面添加过了
  // - 避免重复添加同一个点
  
  for (size_t i = 1; i < num_of_knots; ++i) {
    // ======================================================================
    // 检查点 A：单调性验证（调试和安全检查）
    // ======================================================================
    // 理论保证：QP 优化器应该保证 s 单调递增（因为 v >= 0）
    // 实际情况：数值优化可能因为：
    // 1. 求解器精度设置不当
    // 2. 约束冲突（例如时间太紧，物理上无法满足）
    // 3. 浮点误差累积
    // 导致 s 出现微小的回退
    // 
    // 判断逻辑：s[i-1] - s[i] > kEpislon
    // - 若 s[i] < s[i-1] - 1e-6，说明位置倒退超过容忍范围
    // - 这是异常情况，应该终止并报警
    if (s[i - 1] - s[i] > kEpislon) {
      std::cout << "unexpected decreasing s in speed smoothing at time "
                << static_cast<double>(i) * delta_t << "with total time "
                << total_t;
      break;  // 立即终止速度剖面构建
    }
    
    // ======================================================================
    // 核心操作：添加速度点
    // ======================================================================
    // 参数解析：
    // - s[i]:        当前时刻的累计弧长（QP 优化结果）
    // - delta_t * i: 当前时刻的绝对时间 t_i = i × Δt
    //                例如：Δt=0.2s, i=5 → t=1.0s
    // - ds[i]:       当前时刻的速度（QP 优化结果）
    // - dds[i]:      当前时刻的加速度（QP 优化结果）
    // - (dds[i] - dds[i-1]) / delta_t: 数值微分计算 jerk
    // 
    // 【Jerk 计算公式】
    // jerk = da/dt ≈ (a_i - a_{i-1}) / Δt
    // 
    // 物理意义：
    // - Jerk 描述加速度的变化快慢
    // - 高 jerk 会导致乘客感到不适（急刹车、急加速）
    // - 优化目标中包含 jerk 项，使得 jerk 尽可能平滑
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
    
    // ======================================================================
    // 检查点 B：提前终止条件
    // ======================================================================
    // 为什么需要提前终止？
    // 
    // 【场景 1：路径已到达终点】
    // - QP 优化的时间范围是估算的（total_t 是基于经验公式）
    // - 实际车辆可能提前到达终点（s ≈ path_length）
    // - 继续添加点没有物理意义（不能超出路径范围）
    // 
    // 【场景 2：避免数值振荡】
    // - 当 s 非常接近 path_length 时（例如 path_length - s < 1e-6）
    // - 优化器可能产生微小的振荡：s[i] 在 path_length 附近反复震荡
    // - 这些点对轨迹没有实际贡献，且可能引入噪声
    // 
    // 判断逻辑：path_length - s[i] < sEpislon
    // - 若剩余距离小于 1 微米，认为已到达终点
    // - 立即停止添加速度点
    // 
    // 注意：这里使用 break 而非 continue
    // - break: 终止整个循环，不再处理后续时间点
    // - 原因：一旦到达终点，后续所有点都会超出路径范围
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }

  // ========================================================================
  // 第九步：将路径数据转换为 DiscretizedPath 格式
  // ========================================================================
  // 【为什么需要这一步？】
  // - Hybrid A* 输出的是离散的 (x, y, phi, s) 点序列
  // - 需要封装成 DiscretizedPath 对象，支持弧长插值查询
  // - DiscretizedPath 提供 Evaluate(s) 方法，可根据弧长 s 插值得到路径点
  
  // combine speed and path profile
  DiscretizedPath path_data;
  for (size_t i = 0; i < path_points_size; ++i) {
    common::PathPoint path_point;
    path_point.set_x(result->x[i]);              // 设置 x 坐标
    path_point.set_y(result->y[i]);              // 设置 y 坐标
    path_point.set_theta(result->phi[i]);        // 设置航向角
    path_point.set_s(result->accumulated_s[i]);  // 设置累计弧长
    path_data.push_back(std::move(path_point));
  }

  HybridAStartResult combined_result;

  // ========================================================================
  // 第十步：融合路径和速度剖面（核心步骤）
  // ========================================================================
  // 
  // 【问题背景】
  // 经过前面的步骤，我们得到了两个独立的数据结构：
  // 1. path_data:  空间域的路径信息 (x, y, phi) 关于弧长 s 的函数
  // 2. speed_data: 时间域的速度信息 (s, v, a) 关于时间 t 的函数
  // 
  // 【融合目标】
  // 将两者结合，生成完整的时空轨迹：(x, y, phi, v, a) 关于时间 t 的函数
  // 
  // 【融合策略】
  // 采用"时间域重采样"方法：
  // - 在时间轴上均匀采样（每 kDenseTimeResoltuion 秒一个点）
  // - 对每个时间点 t：
  //   Step 1: 从 speed_data 查询 s(t), v(t), a(t)  [时间 → 弧长/速度]
  //   Step 2: 从 path_data 查询 x(s), y(s), phi(s)  [弧长 → 位置]
  //   Step 3: 组合成完整轨迹点 (x, y, phi, v, a, t)
  // 
  // 【为什么选择时间域采样？】
  // 1. 控制器需要等时间间隔的轨迹点（便于控制周期对齐）
  // 2. 速度优化结果本身就是时间参数化的（QP 在时间域求解）
  // 3. 轨迹执行时以时间为驱动（每个控制周期查询当前时刻的目标状态）
  // 
  // 【采样频率选择】
  // kDenseTimeResoltuion = 0.5s 意味着：
  // - 控制器将以 2Hz 的频率接收轨迹点
  // - 这个频率通常低于实际控制频率（例如 10Hz）
  // - 控制器会在相邻点之间进行插值以获得更高频率的目标
  // 
  // ========================================================================
  
  // TODO(Jinyun): move to confs
  
  // ------------------------------------------------------------------------
  // 参数设置
  // ------------------------------------------------------------------------
  const double kDenseTimeResoltuion = 0.5;  // 时间采样间隔 0.5 秒
  
  // 计算时间范围（稍微延长避免浮点误差）
  // speed_data.TotalTime() 返回最后一个点的时间戳
  // 加上微小的偏移量 1.0e-6 确保最后一个点能被采样到
  const double time_horizon =
      speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
  
  // 路径数据合法性检查
  if (path_data.empty()) {
    std::cout << "path data is empty";
    return false;
  }
  
  // ========================================================================
  // 核心融合循环：时间域重采样
  // ========================================================================
  // 
  // 【循环结构】
  // for (t = 0; t < time_horizon; t += 0.5s):
  //     查询 speed_data(t) → 得到 s, v, a
  //     查询 path_data(s)  → 得到 x, y, phi
  //     组合成轨迹点并存储
  // 
  // 【插值原理】
  // 两次插值过程：
  // 1. 时间 → 弧长：speed_data.EvaluateByTime(t) 使用线性插值
  // 2. 弧长 → 位置：path_data.Evaluate(s) 使用线性插值
  // 
  // 【数值稳定性】
  // - 使用容差检查避免浮点误差
  // - 边界条件处理：超出范围时自动终止
  // 
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResoltuion) {
    // ======================================================================
    // Step 1：时间 → 速度剖面查询
    // ======================================================================
    // 功能：根据当前时间 t，插值计算速度状态
    // 输入：cur_rel_time（相对起始时刻的时间）
    // 输出：speed_point 包含 {s(t), t, v(t), a(t), jerk(t)}
    // 
    // 插值逻辑（在 SpeedData::EvaluateByTime 内部实现）：
    // - 二分查找找到 t 所在的时间区间 [t_i, t_{i+1}]
    // - 线性插值：s(t) = s_i + (s_{i+1} - s_i) * (t - t_i) / (t_{i+1} - t_i)
    //             v(t) = v_i + (v_{i+1} - v_i) * (t - t_i) / (t_{i+1} - t_i)
    //             a(t) = a_i + (a_{i+1} - a_i) * (t - t_i) / (t_{i+1} - t_i)
    // 
    // 失败情况：
    // - speed_data 为空
    // - 时间 t 超出 speed_data 的时间范围
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      std::cout << "Fail to get speed point with relative time "
                << cur_rel_time;
      return false;
    }

    // ======================================================================
    // 边界检查：速度剖面的弧长是否超出路径长度
    // ======================================================================
    // 为什么需要这个检查？
    // - QP 优化可能产生略微超出路径长度的 s 值（数值误差）
    // - path_data.Evaluate(s) 对超出范围的 s 会返回端点，但语义不正确
    // - 提前终止避免生成无效的轨迹点
    // 
    // 示例场景：
    // - path_length = 10.0m
    // - 优化结果：s(t_final) = 10.0001m（浮点误差）
    // - 此时应停止采样，避免虚假的"超出路径"轨迹点
    if (speed_point.s() > path_data.Length()) {
      break;
    }

    // ======================================================================
    // Step 2：弧长 → 路径点查询
    // ======================================================================
    // 功能：根据弧长 s，插值计算路径上的几何位置
    // 输入：speed_point.s()（当前时刻对应的弧长）
    // 输出：path_point 包含 {x(s), y(s), phi(s), s}
    // 
    // 插值逻辑（在 DiscretizedPath::Evaluate 内部实现）：
    // - 二分查找找到 s 所在的弧长区间 [s_i, s_{i+1}]
    // - 线性插值：x(s) = x_i + (x_{i+1} - x_i) * (s - s_i) / (s_{i+1} - s_i)
    //             y(s) = y_i + (y_{i+1} - y_i) * (s - s_i) / (s_{i+1} - s_i)
    //             phi(s) = phi_i + (phi_{i+1} - phi_i) * (s - s_i) / (s_{i+1} - s_i)
    // 
    // 注意：航向角 phi 的插值需要考虑角度归一化（跨越 ±π 边界）
    common::PathPoint path_point = path_data.Evaluate(speed_point.s());

    // ======================================================================
    // Step 3：组合路径点和速度点，构建完整轨迹
    // ======================================================================
    // 融合结果：
    // - 位置来自 path_point: (x, y, phi)
    // - 速度来自 speed_point: (v, a)
    // - 时间：cur_rel_time
    // - 弧长：speed_point.s()（也可用 path_point.s()，数值应相同）
    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());
    
    // ======================================================================
    // Step 4：档位处理 - 调整速度和加速度的符号
    // ======================================================================
    // 【为什么需要符号调整？】
    // - QP 优化器假设速度 v >= 0（单向优化，简化约束）
    // - 但实际车辆倒车时速度应为负值（符合物理直觉）
    // - 需要根据档位（gear）调整符号
    // 
    // 【符号规则】
    // - 前进（gear = true）：v > 0, a 可正可负
    // - 倒车（gear = false）：v < 0, a 符号取反
    // 
    // 【物理意义】
    // - 正速度 + 正加速度：前进加速
    // - 正速度 + 负加速度：前进减速（刹车）
    // - 负速度 + 正加速度：倒车减速（减小倒车速度）
    // - 负速度 + 负加速度：倒车加速（增大倒车速度）
    // 
    // 示例：
    // - QP 结果：v = 1.5 m/s, a = -0.5 m/s²
    // - 前进模式：直接使用 v = 1.5, a = -0.5（减速）
    // - 倒车模式：取反 v = -1.5, a = 0.5（减小倒车速度，也是减速）
    if (!gear) {
      combined_result.v.push_back(-speed_point.v());   // 倒车：速度取反
      combined_result.a.push_back(-speed_point.a());   // 倒车：加速度取反
    } else {
      combined_result.v.push_back(speed_point.v());    // 前进：保持原值
      combined_result.a.push_back(speed_point.a());    // 前进：保持原值
    }
  }

  // ========================================================================
  // 后处理：移除最后一个加速度
  // ========================================================================
  // 【为什么要 pop_back()？】
  // 轨迹点和控制量的数量关系：
  // - 位置点数量：N（包括起点和终点）
  // - 速度点数量：N（每个位置点都有对应的速度）
  // - 加速度数量：N-1（只需要 N-1 个控制输入）
  // 
  // 【原因分析】
  // 1. 离散控制系统：状态转移需要 N-1 个控制输入
  //    x[0] --a[0]--> x[1] --a[1]--> ... --a[N-2]--> x[N-1]
  // 
  // 2. 终点处的加速度没有物理意义：
  //    - 终点是停车状态（v=0, a=0）
  //    - 终点的加速度不会影响后续运动（因为没有后续）
  // 
  // 3. 控制器期望的数据格式：
  //    - 输入：当前状态 x[i]，期望加速度 a[i]
  //    - 输出：下一状态 x[i+1]
  //    - 因此最后一个状态 x[N-1] 不需要对应的加速度
  // 
  // 【示例】
  // 假设轨迹有 5 个点：
  // - 位置：[p0, p1, p2, p3, p4]
  // - 速度：[v0, v1, v2, v3, v4]
  // - 加速度（pop前）：[a0, a1, a2, a3, a4]
  // - 加速度（pop后）：[a0, a1, a2, a3]
  // 
  // 控制序列：
  // t0: 在 p0，速度 v0，执行 a0 → 到达 p1
  // t1: 在 p1，速度 v1，执行 a1 → 到达 p2
  // t2: 在 p2，速度 v2，执行 a2 → 到达 p3
  // t3: 在 p3，速度 v3，执行 a3 → 到达 p4
  // t4: 在 p4，速度 v4=0，停车（无需加速度）
  combined_result.a.pop_back();

  // ========================================================================
  // 第十一步：计算转向角 (steering angle)
  // ========================================================================
  // recalc step size
  path_points_size = combined_result.x.size();

  // 根据阿克曼转向几何模型计算前轮转向角
  // 公式推导：
  // - 车辆航向角变化率：dφ/ds = κ（曲率）
  // - 阿克曼模型：tan(δ) = L * κ，其中 L 为轴距，δ 为前轮转向角
  // - 离散化：Δφ / Δs ≈ κ
  // - 因此：δ = arctan(L * Δφ / Δs)
  
  // load steering from phi
  for (size_t i = 0; i + 1 < path_points_size; ++i) {
    // 计算相邻点的航向角变化与弧长变化的比值（近似曲率）
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        vehicle_param_.wheel_base /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    
    // 根据档位调整转向角符号
    // 前进：δ = arctan(曲率 * L)
    // 倒车：δ = arctan(-曲率 * L)（倒车时转向逻辑相反）
    discrete_steer =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
    combined_result.steer.push_back(discrete_steer);
  }

  // ========================================================================
  // 第十二步：更新结果并返回
  // ========================================================================
  *result = combined_result;
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult &result,
    std::vector<HybridAStartResult> *partitioned_result) {
  const auto &x = result.x;
  const auto &y = result.y;
  const auto &phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    std::cout
        << "states sizes are not equal when do trajectory partitioning of "
           "Hybrid A Star result"
        << std::endl;
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto *current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2)*1.5;
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  //先分段，再求速度加速度

  for (auto &result : *partitioned_result) {
    bool FLAGS_use_s_curve_speed_smooth = 1;
    if (FLAGS_use_s_curve_speed_smooth) {
      // zhaokun20221013
      if (!GenerateSCurveSpeedAcceleration(&result)) {
        std::cout << "GenerateSCurveSpeedAcceleration fail" << std::endl;
        return false;
       }
      }
     else {
      if (!GenerateSpeedAcceleration(&result)) {
        std::cout << "GenerateSpeedAcceleration fail" << std::endl;
        return false;
      }
      //test start
      // if(!GenerateSCurveSpeedAcceleration(&result)){
      // std::cout << "GenerateSCurveSpeedAcceleration fail" << std::endl;
      //   return false;}
      //test end
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "[Hybrid A*] Speed profile: " << diff.count() * 1000.0 << " ms" << std::endl;
  return true;
}

/**
 * @brief 生成完整的时间-速度剖面（处理换挡场景的核心函数）
 * 
 * 【为什么需要这个函数？它看起来很多余？】
 * 
 * 在 Hybrid A* 规划中，车辆可能需要多次换挡（前进-倒车-前进...）
 * 例如：平行泊车场景
 *   1. 前进靠近车位
 *   2. 倒车进入车位
 *   3. 前进调整位置
 * 
 * 【核心问题】
 * 如果直接对整条轨迹（包含多个档位段）进行速度优化，会出现：
 * 
 * 问题 1：换挡点处的速度跳变
 * - 前进段：v > 0
 * - 倒车段：v < 0
 * - 换挡瞬间：速度从 +1.5m/s 跳变到 -1.0m/s
 * - 这在物理上不可能（需要无限大的加速度）
 * 
 * 问题 2：QP 优化器无法处理速度符号变化
 * - Piecewise Jerk QP 假设 v >= 0（单向优化）
 * - 无法在一个优化问题中同时处理正负速度
 * - 强行优化会导致约束冲突，求解失败
 * 
 * 【解决方案：分段优化 + 拼接】
 * 
 * Step 1: TrajectoryPartition() - 按档位分段
 *   输入：完整轨迹 (x, y, phi)
 *   输出：多个单档位子轨迹
 *   示例：[前进段1] [倒车段] [前进段2]
 * 
 * Step 2: GenerateSCurveSpeedAcceleration() - 分段速度优化
 *   对每个子轨迹独立优化速度曲线
 *   每段都是从静止启动到静止停止（v_start=0, v_end=0）
 *   避免了换挡点的速度跳变问题
 * 
 * Step 3: 本函数 - 拼接所有段
 *   将优化后的子轨迹拼接成完整轨迹
 *   移除重复的拼接点（每段的终点 = 下一段的起点）
 * 
 * 【为什么看起来"多余"？】
 * 
 * 如果轨迹不换挡（只有前进 OR 只有倒车）：
 * - TrajectoryPartition 会返回单段轨迹
 * - 本函数退化为简单的复制操作
 * - 看起来确实多余
 * 
 * 但对于换挡场景，这是必需的！
 * 
 * 【函数执行流程】
 * 
 * @param result 输入输出参数，包含完整轨迹，输出拼接后的速度剖面
 * @return true 成功生成速度剖面，false 失败
 */
bool HybridAStar::GetTemporalProfile(HybridAStartResult *result) {
  // ========================================================================
  // Step 1: 按档位分段
  // ========================================================================
  // TrajectoryPartition 会检测换挡点（前进↔倒车）
  // 将轨迹切分成多个单档位段
  // 
  // 检测原理：比较航向角与运动方向的夹角
  // - 夹角 < 90°：前进
  // - 夹角 > 90°：倒车
  // - 夹角变化时：换挡点
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    std::cout << "TrajectoryPartition fail" << std::endl;
    return false;
  }
  
  // 注意：TrajectoryPartition 内部会对每个分段调用速度优化
  // 所以此时 partitioned_results 中每段都已经包含 (v, a, steer)
  
  // ========================================================================
  // Step 2: 拼接所有段
  // ========================================================================
  // 【拼接策略】
  // 每个段的格式：[p0, p1, p2, ..., pn]
  // 相邻段共享边界点：段1的终点 = 段2的起点
  // 
  // 拼接时需要移除重复点：
  // 段1: [p0, p1, p2]
  // 段2: [p2, p3, p4]  // p2 是重复的
  // 拼接: [p0, p1, p2, p3, p4]  // 只保留一个 p2
  
  HybridAStartResult stitched_result;
  
  // 遍历所有分段
  for (const auto &result : partitioned_results) {
    // ======================================================================
    // 拼接策略：end() - 1
    // ======================================================================
    // 为什么所有状态量（x, y, phi, v）都是 end() - 1？
    // 
    // 【原因】：避免重复添加段间连接点
    // 
    // 示例场景：3段轨迹
    // 段1: [A, B, C]     // 前进段
    // 段2: [C, D, E]     // 倒车段（C 是换挡点）
    // 段3: [E, F, G]     // 前进段（E 是换挡点）
    // 
    // 拼接过程：
    // 1. 处理段1：添加 [A, B]（不添加 C）
    // 2. 处理段2：添加 [C, D]（不添加 E）
    // 3. 处理段3：添加 [E, F]（不添加 G）
    // 4. 最后单独添加 G（最后一段的终点）
    // 
    // 结果：[A, B, C, D, E, F, G]  ✓ 正确
    // 
    // 如果不减 1：
    // 1. 处理段1：添加 [A, B, C]
    // 2. 处理段2：添加 [C, D, E]  // C 重复！
    // 3. 处理段3：添加 [E, F, G]  // E 重复！
    // 
    // 结果：[A, B, C, C, D, E, E, F, G]  ✗ 错误
    
    // 拼接位置状态（x, y, phi）
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    
    // 拼接速度状态（v）
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    
    // ======================================================================
    // 拼接控制量：加速度和转向角
    // ======================================================================
    // 【为什么加速度和转向角不用 end() - 1？】
    // 
    // 【关键】：控制量的数量 = 状态量数量 - 1
    // 
    // 状态-控制对应关系：
    // 状态点：[p0, p1, p2, p3]  // 4 个点
    // 控制量：[u0, u1, u2]      // 3 个控制（少 1 个）
    // 
    // 物理意义：
    // - u0 驱动 p0 → p1
    // - u1 驱动 p1 → p2
    // - u2 驱动 p2 → p3
    // - p3 之后没有运动，不需要控制量
    // 
    // 拼接示例：
    // 段1 状态：[A, B, C]     段1 控制：[a0, a1]
    // 段2 状态：[C, D, E]     段2 控制：[a2, a3]
    // 段3 状态：[E, F, G]     段3 控制：[a4, a5]
    // 
    // 拼接后状态：[A, B, C, D, E, F, G]  // 7 个点
    // 拼接后控制：[a0, a1, a2, a3, a4, a5]  // 6 个控制 ✓ 正确
    // 
    // 【为什么直接 end()？】
    // 因为控制量本身就比状态少 1 个
    // - 段1 控制已经不包含 C→D 的控制（那是段2的）
    // - 段2 控制已经不包含 E→F 的控制（那是段3的）
    // - 直接全部复制即可，没有重复
    
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  
  // ========================================================================
  // Step 3: 补齐最后一段的终点
  // ========================================================================
  // 【为什么需要单独添加？】
  // 
  // 上面的循环中，每段都是 end() - 1，即都丢弃了终点
  // 这导致最后一段的终点也被丢弃了
  // 需要手动补回来
  // 
  // 【为什么只补状态，不补控制？】
  // - 状态点：需要完整的轨迹点序列（包括终点）
  // - 控制量：终点之后不再运动，不需要控制量
  // 
  // 【.back() 的含义】
  // partitioned_results.back()：最后一段轨迹
  // .x.back()：最后一段的最后一个 x 坐标（即整条轨迹的终点）
  
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());  // 终点速度应为 0
  
  // ========================================================================
  // Step 4: 更新结果并返回
  // ========================================================================
  *result = stitched_result;
  return true;
}

// ============================================================================
// 【总结：GetTemporalProfile 的价值】
// ============================================================================
//
// 单档位场景（只前进 OR 只倒车）：
//   - 确实有点"多余"，TrajectoryPartition 返回单段
//   - 但仍有拼接逻辑的统一性价值
//
// 多档位场景（前进-倒车-前进）：
//   - **必不可少**！是处理换挡的唯一方案
//   - 避免换挡点的速度跳变
//   - 确保每段都能正常优化
//
// 架构优势：
//   - 解耦：分段优化 vs 整体拼接
//   - 可扩展：便于增加更复杂的换挡策略
//   - 可维护：每个函数职责清晰
//
// 【类比】：像拼装乐高
//   - TrajectoryPartition：按颜色分类积木
//   - GenerateSCurveSpeedAcceleration：给每块积木上色
//   - GetTemporalProfile：按顺序拼成完整模型
// ============================================================================

// A*的主要流程
bool HybridAStar::Plan(double sx, double sy, double sphi, double ex, double ey,
                       double ephi,                         //起点、终点
                       const std::vector<double> &XYbounds, //地图？？
                       const std::vector<std::vector<common::math::Vec2d>>
                           &obstacles_vertices_vec,  //障碍物顶点信息
                       HybridAStartResult *result) { //结果
  // clear containers1.初始化清空
  open_set_.clear(); //无序图数据结构 开集 记录了所有待遍历和遍历过的节点
  close_set_.clear(); //闭集 记录了所有已经遍历过的节点
  open_pq_ = decltype(open_pq_)(); //队列根据节点cost由小到达的顺序排列
  final_node_ = nullptr;

  // 2.构造障碍物轮廓线段。为什么不把这一段代码封装在obstacle类中
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec; //障碍物线段数组
  for (const auto &obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    //网友反馈：我认为这里有错，少构造了一条线段。等待校验
    //(obstacle_vertices[vertices_num - 1], obstacle_vertices[0]
    for (size_t i = 0; i < vertices_num - 1; ++i) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment); //尾插
    }
    // zhaokun20221012增加最后一条线段A-B-C-D-A
    common::math::LineSegment2d last_line_segment = common::math::LineSegment2d(
        obstacle_vertices[vertices_num - 1], obstacle_vertices[0]);
    obstacle_linesegments.emplace_back(last_line_segment); //尾插

    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // load XYbounds
  // 3.载入起点终点并做碰撞校验
  XYbounds_ = XYbounds;
  // load nodes and obstacles
  // node3d相对于xybound的起点和-pi的格子数
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    std::cout << "start_node in collision with obstacles" << std::endl;
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    std::cout << "end_node in collision with obstacles" << std::endl;
    return false;
  }
  // 4.生成动态规划代价地图，以终点为搜索起点，遍历整个地图计算各个点到终点的代价。
  // f = g + h，在apollo中变量名称是：
  // cost_ = path_cost_ + heuristic_
  // 使用djkstra算法来计算目标点到图中任一点的path_cost
  // 相当于把地图里每个点都跑了一遍，知道了到每个点的距离
  // 为什么不用经典A*呢，因为经典A*适合点到点的最短距离，
  // 想起点到所有点的距离，用dijkstra更合适
  // dijkstra相当与把经典A*中 f=g+h中的h设为0，也就是不启发
  // 生成一个dp_map,这里的dp是动态规划的意思。这个dp_map是一个unordered map
  // 里面记录了一个二维网格地图中所有的格子，也就是所有节点，每一个节点上都有
  // 它到终点的path_cost，后续只需要查表，空间换时间


  double map_time = clock();
  //
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);
  std::cout << "[Hybrid A*] DP map: " << (clock() - map_time) / CLOCKS_PER_SEC * 1000.0 << " ms" << std::endl;
 
  // load open set, pq5.载入起点至open_set、open_pq
  // load open set, pq
  // 用emplace比push效率高，并且用push的话，需要写成 make_pair(next_node->GetIndex(), next_node)，多一个make_pair
  // open_set_是一个unordered_map的类型，用这个是因为这种键值对的形式比较方便，不需要排序所以用unordered可能节省资源。
  // open_pq_现在是pq的类型，主要是要有序这个特性。虽然set也可以有序，但是pq比set快，set多了一个唯一的属性
  // open_pq_的排序规则是自定义的，是按cost排序的


  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  size_t explored_node_num = 0;
  double astar_start_time = clock();
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first; //取代价最小的相邻节点
    open_pq_.pop(); //从队列中取出最小值
    std::shared_ptr<Node3d> current_node = open_set_[current_id]; //存储当前节点
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.如果有一条RS曲线连接当前点和终点无碰撞，那么终止搜索
    const double rs_start_time = clock();

    if (AnalyticExpansion(current_node)) {
      break;
    }
    

    const double rs_end_time = clock();
    rs_time += rs_end_time - rs_start_time;
    //将当前节点放入close_set
    close_set_.emplace(current_node->GetIndex(), current_node);
    //遍历邻接节点,并计算cost
    for (size_t i = 0; i < next_node_num_; ++i) { // 10个方向扩展，上5下5
      //以不同方向，向前行驶根号2倍的栅格分辨率向前行进
      std::shared_ptr<Node3d> next_node =
          Next_node_generator(current_node, i); //生成下一个节点  
      // boundary check failure handle
      if (next_node == nullptr) {
        continue; //超出边界，跳过
      }
      // check if the node is already in the close set检测是否已经在close_set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue; //已经检测到跳过
      }
      // collision check碰撞检测
      if (!ValidityCheck(next_node)) {
        continue; //跳出本次循环
      }
      if (open_set_.find(next_node->GetIndex()) ==
          open_set_.end()) // open_set中不存在则加入open_set
      {
        explored_node_num++;
        const double start_time = clock();
        CalculateNodeCost(current_node, next_node); //计算节点cost
        const double end_time = clock();
        heuristic_time += end_time - start_time;
        open_set_.emplace(next_node->GetIndex(),
                          next_node); //邻接节点放入open_set
        open_pq_.emplace(
            next_node->GetIndex(),
            next_node
                ->GetCost()); //邻接节点放入open_pq优先队列，open_pq只比open_set少了最优点
      }
    }
    

  }
  if (final_node_ == nullptr) {
    std::cout << "Hybrid A searching return null ptr(open_set ran out)"
              << std::endl;
    return false;
  }

  if (!GetResult(result)) {
    std::cout << "GetResult failed" << std::endl;
    return false;
  }
  
  std::cout << "[Hybrid A*] Done! nodes=" << explored_node_num 
            << ", total=" << (clock() - astar_start_time) / CLOCKS_PER_SEC * 1000.0 << "ms"
            << ", heuristic=" << heuristic_time / CLOCKS_PER_SEC * 1000.0 << "ms"
            << ", RS=" << rs_time / CLOCKS_PER_SEC * 1000.0 << "ms" << std::endl;
  return true;
}
