#include "common/math/vec2d.h"
#include "configs/open_space_trajectory_optimizer_config.h"
#include "eigen3/Eigen/Dense"
#include "open_space_map/open_space_map.h"
#include "safe_corridor_trajectory_optimizer/open_space_trajectory_optimizer.h"
#include "trajectory_smoother/open_space_roi_deal.h"
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <string>
#include "common/matplot/matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace common::math;

static void WriteHybridAStarCsv(const std::string &path,
                                const HybridAStartResult &r) {
  std::ofstream out(path);
  out << "x,y,phi,v,a,steer,accumulated_s\n";
  const size_t n = r.x.size();
  for (size_t i = 0; i < n; ++i) {
    const double x = r.x[i];
    const double y = (i < r.y.size()) ? r.y[i] : 0.0;
    const double phi = (i < r.phi.size()) ? r.phi[i] : 0.0;
    const double v = (i < r.v.size()) ? r.v[i] : 0.0;
    const double a = (i < r.a.size()) ? r.a[i] : 0.0;
    const double steer = (i < r.steer.size()) ? r.steer[i] : 0.0;
    const double s =
        (i < r.accumulated_s.size()) ? r.accumulated_s[i] : 0.0;
    out << x << "," << y << "," << phi << "," << v << "," << a << "," << steer
        << "," << s << "\n";
  }
}

static void WriteDiscretizedTrajectoryCsv(const std::string &path,
                                          const DiscretizedTrajectory &traj) {
  std::ofstream out(path);
  out << "x,y,theta,s,relative_time,v,a,steer\n";
  for (const auto &tp : traj) {
    const auto &pp = tp.path_point();
    out << pp.x() << "," << pp.y() << "," << pp.theta() << "," << pp.s() << ","
        << tp.relative_time() << "," << tp.v() << "," << tp.a() << ","
        << tp.steer() << "\n";
  }
}

// 创建结果目录，格式：result/year_month_day_hour_minute_seconds
std::string createResultDirectory() {
  std::time_t now = std::time(nullptr);
  std::tm* timeinfo = std::localtime(&now);
  
  std::ostringstream oss;
  oss << "result/"
      << std::setfill('0') << std::setw(4) << (1900 + timeinfo->tm_year) << "_"
      << std::setfill('0') << std::setw(2) << (timeinfo->tm_mon + 1) << "_"
      << std::setfill('0') << std::setw(2) << timeinfo->tm_mday << "_"
      << std::setfill('0') << std::setw(2) << timeinfo->tm_hour << "_"
      << std::setfill('0') << std::setw(2) << timeinfo->tm_min << "_"
      << std::setfill('0') << std::setw(2) << timeinfo->tm_sec;
  
  std::string result_dir = oss.str();
  
  // 创建目录（包括父目录）
  std::string cmd = "mkdir -p " + result_dir;
  system(cmd.c_str());
  
  std::cout << "结果将保存到目录: " << result_dir << std::endl;
  return result_dir;
}

int main() {

  std::cout << "hello" << std::endl;
  
  // 创建结果目录
  std::string result_dir = createResultDirectory();

  //设置地图边界
  std::unique_ptr<OpenSpaceMap> map = std::make_unique<OpenSpaceMap>();
  map->SetXYBounds(0, 100, 0, 100);
  //设置障碍物信息，逆时针顶点
  std::vector<Vec2d> obs1, obs2, obs3;
  

  //倒车入库场景 逆时针
  obs1.push_back(Vec2d(2, 10));
  obs1.push_back(Vec2d(2, 3));
  obs1.push_back(Vec2d(20, 3));
  obs1.push_back(Vec2d(20, 10));
  map->SetOnebstacle(obs1);
  //车2
  obs2.push_back(Vec2d(32.5, 10));
  obs2.push_back(Vec2d(32.5, 3));
  obs2.push_back(Vec2d(60, 3));
  obs2.push_back(Vec2d(60, 10));
  map->SetOnebstacle(obs2);
  //房子
  obs3.push_back(Vec2d(0, 20));
  obs3.push_back(Vec2d(0,30));
  obs3.push_back(Vec2d(60, 30));
  obs3.push_back(Vec2d(60, 20));
  map->SetOnebstacle(obs3);

  
  //膨胀障碍物
  double swelling_r =
      pow((2.11 / 2.0) * (2.11 / 2.0) + (4.99 / 4.0) * (4.99 / 4.0), 0.5);
  map->SwellingObstacles(swelling_r * 0.6);

  // map->PlotAll();

  //设置起点终点位置
  MapPoint start_pose(2, 15, 0);

  //侧方停车
  // MapPoint end_pose(40, 90,
  //                   1 / 2.0 * M_PI); //双精度运算避免写正数，导致取整
  // MapPoint end_pose(32 + 1.043 - 4.933 / 2, 58.75,
  //                   0); //双精度运算避免写正数，导致取整
  //   MapPoint end_pose(28.5, 57 + 1.043 - 4.933 / 2,
  //                     -1 / 2.0 * M_PI); //双精度运算避免写正数，导致取整

  auto XYbounds = map->XYbounds();

  auto swelling_obstacles_vec = map->swelling_obstacles_vec();
  auto obstacles_vertices_vec = map->obstacles_vertices_vec();
  //障碍物膨胀应该在外边做

  double rotate_angle = 0;
  common::math::Vec2d translate_origin(0, 0);
  // Eigen::MatrixXi obstacles_edges_num(1, 1);
  OpenSpaceTrajectoryOptimizerConfig open_space_config;

  MapPoint end_pose(25,13, 0);
  //倒车终点
 //MapPoint end_pose(31.25,7.5 - (4.933 / 2 - 1.043),1 / 2.0 * M_PI);
  // end_pose.push_back(31.25);
  // end_pose.push_back(7.5 - (4.933 / 2 - 1.043));
  // end_pose.push_back(1 / 2.0 * M_PI);
  // end_pose.push_back(0);
  // //侧方终点
  // end_pose.push_back(33 - ((4.933 / 2 - 1.043)));
  // end_pose.push_back(8.75);
  // end_pose.push_back(0);
  // end_pose.push_back(0);

  size_t obstacles_num = obstacles_vertices_vec.size();
  int obs_edges_num = 0;
  Eigen::MatrixXi obstacles_edges_num = Eigen::MatrixXi::Zero(obstacles_num, 1);
  for (int i = 0; i < obstacles_num; i++) {
    obstacles_edges_num(i, 0) = obstacles_vertices_vec[i].size();
  }

 

  double time_latency;
  std::unique_ptr<OpenSpaceTrajectoryOptimizer> optimizer =
      std::make_unique<OpenSpaceTrajectoryOptimizer>(open_space_config);

  // double time_latency;
  auto plan_start_time = clock();
  int plan_result = optimizer->Plan(start_pose, end_pose, XYbounds, rotate_angle, translate_origin, 
                                     obstacles_edges_num, obstacles_vertices_vec, &time_latency);
  double plan_time = (clock() - plan_start_time) * 1.0 / CLOCKS_PER_SEC;
  std::cout << "规划耗时: " << plan_time << "s" << std::endl;
  
  // 检查规划是否成功
  bool planning_success = (plan_result != 0);
  bool has_hybrid_result = !optimizer->hy_astar_result_.x.empty() && 
                           !optimizer->hy_astar_result_.y.empty();
  
  if (!planning_success) {
    std::cout << "警告: 路径规划失败！" << std::endl;
  }
  
  if (!has_hybrid_result) {
    std::cout << "警告: Hybrid A* 结果为空！" << std::endl;
  }

  // 无论成功与否，先把“数值结果”落盘，方便用 Python 画图对比
  const std::string hybrid_csv = result_dir + "/hybrid_astar.csv";
  const std::string coarse_csv = result_dir + "/coarse_trajectory.csv";
  const std::string opt_csv = result_dir + "/optimized_trajectory.csv";
  WriteHybridAStarCsv(hybrid_csv, optimizer->hy_astar_result_);

  // 保存 Hybrid A* 结果图（仅当有结果时）
  if (has_hybrid_result) {
    plt::plot(optimizer->hy_astar_result_.x, optimizer->hy_astar_result_.y, "red");
    plt::title("Hybrid A* Result");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::grid(true);
    std::string hy_astar_file = result_dir + "/hybrid_astar_result.png";
    plt::save(hy_astar_file);
    std::cout << "已保存 Hybrid A* 结果图: " << hy_astar_file << std::endl;
    plt::close();
  } else {
    // 创建一个简单的失败提示图
    std::vector<double> empty_x = {0};
    std::vector<double> empty_y = {0};
    plt::plot(empty_x, empty_y, "w");  // 白色点，不可见
    plt::text(0, 0, "Planning Failed - No Path Found");
    plt::title("Hybrid A* Result - Planning Failed");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::xlim(-10, 10);
    plt::ylim(-10, 10);
    plt::grid(true);
    std::string hy_astar_file = result_dir + "/hybrid_astar_result.png";
    plt::save(hy_astar_file);
    std::cout << "警告: Hybrid A* 规划失败，已保存失败提示图: " << hy_astar_file << std::endl;
    plt::close();
  }

  DiscretizedTrajectory coarse_trajectory;
  DiscretizedTrajectory optimized_trajectory;
  optimizer->GetCoarseTrajectory(&coarse_trajectory);
  map->SetCoarseTrajectory(coarse_trajectory);

  optimizer->GetOptimizedTrajectory(&optimized_trajectory);
  map->SetOptimizedTrajectory(optimized_trajectory);

  WriteDiscretizedTrajectoryCsv(coarse_csv, coarse_trajectory);
  WriteDiscretizedTrajectoryCsv(opt_csv, optimized_trajectory);
  std::cout << "已保存结果 CSV:\n"
            << "  - " << hybrid_csv << "\n"
            << "  - " << coarse_csv << "\n"
            << "  - " << opt_csv << std::endl;

  // 保存完整轨迹图（即使失败也保存，显示障碍物和地图信息）
  map->PlotAll();
  std::string full_trajectory_file = result_dir + "/full_trajectory.png";
  plt::save(full_trajectory_file);
  std::cout << "已保存完整轨迹图: " << full_trajectory_file << std::endl;
  
  // 保存规划信息到文本文件
  std::string info_file = result_dir + "/planning_info.txt";
  std::ofstream info_out(info_file);
  if (info_out.is_open()) {
    info_out << "路径规划结果信息\n";
    info_out << "==================\n\n";
    info_out << "规划状态: " << (planning_success ? "成功" : "失败") << "\n";
    info_out << "起点: (" << start_pose.x << ", " << start_pose.y << ", " << start_pose.phi << ")\n";
    info_out << "终点: (" << end_pose.x << ", " << end_pose.y << ", " << end_pose.phi << ")\n";
    info_out << "规划耗时: " << plan_time << " 秒\n";
    info_out << "粗轨迹点数: " << coarse_trajectory.size() << "\n";
    info_out << "优化轨迹点数: " << optimized_trajectory.size() << "\n";
    info_out << "Hybrid A* 路径点数: " << optimizer->hy_astar_result_.x.size() << "\n";
    if (!planning_success || !has_hybrid_result) {
      info_out << "\n警告: 规划失败或结果为空，请检查:\n";
      info_out << "1. 起点和终点是否在有效范围内\n";
      info_out << "2. 起点和终点是否被障碍物阻挡\n";
      info_out << "3. 是否存在从起点到终点的可行路径\n";
    }
    info_out.close();
    std::cout << "已保存规划信息: " << info_file << std::endl;
  }

  return 0;
}