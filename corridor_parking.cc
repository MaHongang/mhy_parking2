#include "common/math/vec2d.h"
#include "configs/open_space_trajectory_optimizer_config.h"
#include "eigen3/Eigen/Dense"
#include "open_space_map/open_space_map.h"
#include "open_space_trajectory_optimizer/open_space_trajectory_optimizer.h"
#include "trajectory_smoother/open_space_roi_deal.h"
#include <memory>
#include <vector>
#include "common/matplot/matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace common::math;
int main() {

  std::cout << "hello" << std::endl;

  //设置地图边界
  std::unique_ptr<OpenSpaceMap> map = std::make_unique<OpenSpaceMap>();
  map->SetXYBounds(0, 100, 0, 100);
  //设置障碍物信息，逆时针顶点
  std::vector<Vec2d> obs1, obs2, obs3;
  

  //倒车入库场景 逆时针
  obs1.push_back(Vec2d(2, 10));
  obs1.push_back(Vec2d(2, 3));
  obs1.push_back(Vec2d(5, 3));
  obs1.push_back(Vec2d(5, 10));
  map->SetOnebstacle(obs1);
  //车2
  obs2.push_back(Vec2d(8, 10));
  obs2.push_back(Vec2d(8, 3));
  obs2.push_back(Vec2d(40, 3));
  obs2.push_back(Vec2d(40, 10));
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

   
  //设置起点终点位置
  MapPoint start_pose(2, 12, 0);

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

  MapPoint end_pose(6.5,7, 1.7);
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
  
  Eigen::MatrixXi obstacles_edges_num = Eigen::MatrixXi::Zero(obstacles_num, 1);
  for (int i = 0; i < obstacles_num; i++) {
    obstacles_edges_num(i, 0) = obstacles_vertices_vec[i].size();
  }

 

  double time_latency;//什么用
  std::unique_ptr<OpenSpaceTrajectoryOptimizer> optimizer =
      std::make_unique<OpenSpaceTrajectoryOptimizer>(open_space_config);

  // double time_latency;
  auto plan_start_time = clock();
  optimizer->Plan(start_pose, end_pose,XYbounds, rotate_angle, translate_origin, obstacles_edges_num,                                      
    obstacles_vertices_vec,   &time_latency);
  std::cout << "plan_start_time耗时:"
            << (clock() - plan_start_time) * 1.0 / CLOCKS_PER_SEC << "s"
            << std::endl;
 
//test start
  plt::plot(optimizer->hy_astar_result_.x,optimizer->hy_astar_result_.y,"red");
  std::vector<double> x_test,y_test;
  for(int i=0;i<optimizer->state_result_ds_test_.cols();i++)
  {
    x_test.push_back(optimizer->state_result_ds_test_(0,i));
    y_test.push_back(optimizer->state_result_ds_test_(1,i));

  }
  plt::plot(x_test,y_test,"blue");
   //plot obstacles
   for (const auto &obs_vertices : obstacles_vertices_vec) {
    std::vector<double> x, y;
    for (const auto &obs_vertice : obs_vertices) {
      x.push_back(obs_vertice.x());
      y.push_back(obs_vertice.y());
    }
    std::map<std::string, std::string> keywords;
    keywords["color"] = "grey";
    plt::fill(x, y, keywords);
  }
  plt::show();
  //test end

  // DiscretizedTrajectory coarse_trajectory;
  // DiscretizedTrajectory optimized_trajectory;
  
  // optimizer->GetCoarseTrajectory(&coarse_trajectory);
  // map->SetCoarseTrajectory(coarse_trajectory);

  // optimizer->GetOptimizedTrajectory(&optimized_trajectory);
  // map->SetOptimizedTrajectory(optimized_trajectory);


  // DiscretizedTrajectoryComposition coarse_trajectory_composition;
  // DiscretizedTrajectoryComposition optimized_trajectory_composition;

  // optimizer->GetCoarseTrajectory(&coarse_trajectory_composition);
  // map->SetCoarseTrajectory(coarse_trajectory_composition);

  // optimizer->GetOptimizedTrajectory(&optimized_trajectory_composition);
  // map->SetOptimizedTrajectory(optimized_trajectory_composition);

  // map->PlotAll();

  return 0;
}