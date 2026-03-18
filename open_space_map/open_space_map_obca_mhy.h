/*该函数的作用
    //设置障碍物
    //障碍物形式有两种
    //1.std::vector<std::vector<common::math::Vec2d>>
    //2.std::vector<std::vector<common::math::LineSegment2d>>
    //本程序使用第一种形式
    //设置轨迹
    绘制图内的图形
    绘制路径
  后续添加速度，加速度曲线对比图，包括汽车的形状，
  stc算法的可视化与obca的可视化对比是否应该共同使用这个类？
  可以，类中的函数分别构造就行

  stc不使用这个类
*/
#pragma once

#include "common/math/vec2d.h"
#include "common/matplot/matplotlibcpp.h"
#include "Eigen/Dense"
#include "planning_data/trajectory/discretized_trajectory.h"
#include <map>
#include <string>
#include <vector>


namespace plt = matplotlibcpp;
// using Eigen::MatrixXd;
class MapPoint {
public:
  MapPoint(double x, double y, double phi) : x(x), y(y), phi(phi){};
  ~MapPoint() = default;
  double x;
  double y;
  double phi;
};

class OpenSpaceMap {

public:
  OpenSpaceMap(/* args */);
  ~OpenSpaceMap();

  void PlotAll();
  //1  边界与障碍物的设置

 //设置roi的边界
 //可以添加以下构造方式void SetXYBounds(vector<double> XYbounds);

  
  void SetXYBounds(double x_min, double x_max, double y_min, double y_max);
  //添加单个障碍物
  void SetOnebstacle(std::vector<common::math::Vec2d> obstacles_vertices);
 
  

  // 2 轨迹的绘制

  //设置粗规划轨迹，一般是Hybrid A*生成的轨迹
  void SetCoarseTrajectory(DiscretizedTrajectory coarse_trajectory);
  
  //设置优化后的轨迹
  void SetOptimizedTrajectory(
      DiscretizedTrajectory optimized_trajectory); //轨迹点即可


 
private:
  void PlotObstacles(const std::vector<std::vector<common::math::Vec2d>>
                         &obstacles_vertices_vec_);

  

  void PlotTrajectory(const DiscretizedTrajectory &trajectory,const std::string &color);
  void PlotTrajectoryV(const DiscretizedTrajectory &trajectory,const std::string &color);


                   

private:
 //障碍物，多边形表示
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;

  // x_min,x_max,y_min,y_max
  std::vector<double> XYbounds_;
//轨迹
  DiscretizedTrajectory optimized_trajectory_;
  DiscretizedTrajectory coarse_trajectory_;

  
};

