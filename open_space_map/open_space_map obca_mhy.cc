#include "open_space_map/open_space_map.h"
OpenSpaceMap::OpenSpaceMap(/* args */) {}

OpenSpaceMap::~OpenSpaceMap() {}

void OpenSpaceMap::PlotAll() {
  PlotObstacles(obstacles_vertices_vec_);



  PlotTrajectory(coarse_trajectory_, "k");


  PlotTrajectory(optimized_trajectory_, "b");

  plt::figure(2);
  PlotTrajectoryV(coarse_trajectory_, "k");
  PlotTrajectoryV(optimized_trajectory_, "b");

  plt::show(); // plot::show()是一个阻塞函数
}

void OpenSpaceMap::SetXYBounds(double x_min, double x_max, double y_min,
                               double y_max) {
  XYbounds_.push_back(x_min);
  XYbounds_.push_back(x_max);
  XYbounds_.push_back(y_min);
  XYbounds_.push_back(y_max);
}
void OpenSpaceMap::SetOnebstacle(
    std::vector<common::math::Vec2d> obstacles_vertices) {
  obstacles_vertices_vec_.push_back(obstacles_vertices);
}
void OpenSpaceMap::SetOptimizedTrajectory(
    DiscretizedTrajectory optimized_trajectory) {
  optimized_trajectory_ = optimized_trajectory;
}
//轨迹点即可
void OpenSpaceMap::SetCoarseTrajectory(
    DiscretizedTrajectory coarse_trajectory) {
  coarse_trajectory_ = coarse_trajectory;
}



void OpenSpaceMap::PlotObstacles(
    const std::vector<std::vector<common::math::Vec2d>>
        &obstacles_vertices_vec) {
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
}



void OpenSpaceMap::PlotTrajectory(const DiscretizedTrajectory &trajectory,
                                  const std::string &color) {

  std::vector<double> x, y;
  for (const auto point : trajectory) {
    x.push_back(point.path_point().x());
    y.push_back(point.path_point().y());
  }

  plt::plot(x, y, color);
}


//绘制速度曲线
//横坐标为时间，纵坐标为速度
//trajectory还包含a,jerk,steer等信息，这里只绘制速度曲线,可以根据需要扩展绘制其他量
void OpenSpaceMap::PlotTrajectoryV(const DiscretizedTrajectory &trajectory,
                                   const std::string &color) {

  std::vector<double> x, y;
  for (const auto point : trajectory) {
    x.push_back(point.relative_time());
    y.push_back(point.v());
  }
  plt::plot(x, y, color);
}