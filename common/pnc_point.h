#pragma once

#include <string>
namespace common {

// PathPoint: 表示路径上的一个点的几何信息和曲率信息
// 包含位置(x,y,z)、航向(theta)、曲率(kappa)、弧长(s)以及曲率的一阶/二阶导数
class PathPoint {
public:
  // 默认构造：将所有数值类型成员初始化为 0，字符串为空
  PathPoint()
      : x_(0.0), y_(0.0), z_(0.0), theta_(0.0), kappa_(0.0), s_(0.0), dkappa_(0.0),
        ddkappa_(0.0), x_derivative_(0.0), y_derivative_(0.0), lane_id_() {}

  // 坐标访问器（返回按值的 double）
  double x() const { return x_; };
  double y() const { return y_; };
  double z() const { return z_; };

  // 航向与曲率相关访问器
  double theta() const { return theta_; };
  double kappa() const { return kappa_; };
  double s() const { return s_; };
  double dkappa() const { return dkappa_; };
  double ddkappa() const { return ddkappa_; };
  // 关于参数化参数 t 的 x,y 导数（参考曲线参数化时的导数）
  double x_derivative() const { return x_derivative_; };
  double y_derivative() const { return y_derivative_; };

  // 设置器（按值设置成员）
  void set_x(const double x) { x_ = x; };
  void set_y(const double y) { y_ = y; };
  void set_z(const double z) { z_ = z; };
  void set_theta(const double theta) { theta_ = theta; };
  void set_kappa(const double kappa) { kappa_ = kappa; };
  void set_dkappa(const double dkappa) { dkappa_ = dkappa; };
  void set_ddkappa(const double ddkappa) { ddkappa_ = ddkappa; };
  void set_s(const double s) { s_ = s; };
  void set_x_derivative(const double x_derivative) { x_derivative_ = x_derivative; };
  void set_y_derivative(const double y_derivative) { y_derivative_ = y_derivative; };

  // lane_id 访问器
  const std::string &lane_id() const { return lane_id_; };
  void set_lane_id(const std::string &id) { lane_id_ = id; };

private:
  // coordinates: 在全局坐标系中的位置
  double x_;
  double y_;
  double z_;

  // direction on the x-y plane: 在 x-y 平面的航向角，单位为弧度（通常）
  double theta_;
  // curvature on the x-y planning: 曲率 kappa，单位为 1/m
  double kappa_;
  // accumulated distance from beginning of the path: 从路径起点累计得到的弧长 s，单位 m
  double s_;

  // derivative of kappa w.r.t s.: 曲率对弧长的一阶导 dkappa/ds
  double dkappa_;
  // derivative of derivative of kappa w.r.t s.: 曲率的二阶导 d2kappa/ds2
  double ddkappa_;
  // The lane ID where the path point is on: 该点对应的车道 ID
  std::string lane_id_;

  // derivative of x and y w.r.t parametric parameter t in
  // CosThetareferenceline: 参考线参数化时 x(t), y(t) 的导数
  double x_derivative_;
  double y_derivative_;
};

/*------------------------------------------------------------------------------------------------------------------*/
// TrajectoryPoint: 表示轨迹（时序）上的点，包含速度/加速度/时间等动力学信息
// 注意：原实现同时对 PathPoint 进行了继承和包含（即 class TrajectoryPoint : public PathPoint 且内部有 PathPoint path_point_），
// 这会带来语义混淆（数据冗余）。下面保留原有代码结构，仅添加注释以说明用途。
// 注释中也提示：Apollo 中通常使用组合（包含）而非继承。
class TrajectoryPoint {
  // path point
  // PathPoint path_point;
  // linear velocity

public:
  // 默认构造：初始化动力学量为 0
  TrajectoryPoint()
      : v_(0.0), a_(0.0), relative_time_(0.0), da_(0.0), steer_(0.0), path_point_() {}

  // 返回对内部的 PathPoint 的 const 引用（避免拷贝）
  const PathPoint &path_point() const { return path_point_; };

  // 返回指向内部 path_point_ 的可变指针，允许外部直接修改内部成员
  PathPoint *mutable_path_point() { return &path_point_; }; //返回指针

  // 动力学量访问器
  double v() const { return v_; };
  double a() const { return a_; };
  double relative_time() const { return relative_time_; };
  double da() const { return da_; };
  double steer() const { return steer_; };

  // 动力学量设置器
  void set_v(const double v) { v_ = v; };
  void set_a(const double a) { a_ = a; };
  void set_relative_time(const double relative_time) { relative_time_ = relative_time; };
  void set_da(const double da) { da_ = da; };
  void set_steer(const double steer) { steer_ = steer; };

public:
  double v_; // 线速度，单位：m/s
  double a_; // 线加速度，单位：m/s^2
  double relative_time_; // 相对于轨迹起点的时间，单位：s
  double da_; // 纵向 jerk，单位：m/s^3
  double steer_; // 转向角（前轮与车身纵轴之间的角度）

  PathPoint path_point_;
};

/*------------------------------------------------------------------------------------------------------------------*/
// SpeedPoint: 表示 s-t 曲线上的一个点，包含时间 t、弧长 s、速度 v、加速度 a、jerk da
class SpeedPoint {
public:
  // 默认构造：初始化数值为 0，并标记为未设置
  SpeedPoint()
      : s_(0.0), t_(0.0), v_(0.0), a_(0.0), da_(0.0), has_s_(false), has_v_(false),
        has_a_(false), has_da_(false) {}

  // 将所有值清零并重置 has_* 标志
  void Clear() {
    s_ = 0.0;
    t_ = 0.0;
    v_ = 0.0;
    a_ = 0.0;
    da_ = 0.0;
    has_s_ = has_v_ = has_a_ = has_da_ = false;
  };

  double s() const { return s_; };
  void set_s(double s) { s_ = s; has_s_ = true; };
  // 注意：使用 has_s_ 标志来标识是否显式设置过 s（比判断 s != 0 更可靠）
  bool has_s() const { return has_s_; };

  double t() const { return t_; };
  void set_t(double t) { t_ = t; };

  double v() const { return v_; };
  void set_v(double v) { v_ = v; has_v_ = true; };
  bool has_v() const { return has_v_; };

  double a() const { return a_; };
  void set_a(double a) { a_ = a; has_a_ = true; };
  bool has_a() const { return has_a_; };
  // 保留旧名以兼容现有调用
  bool hav_a() const { return has_a(); };

  double da() const { return da_; };
  void set_da(double da) { da_ = da; has_da_ = true; };
  bool has_da() const { return has_da_; };
  bool hav_da() const { return has_da(); };

private:
  double s_; // 弧长 s，单位 m
  double t_; // 时间 t，单位 s
  // speed (m/s)
  double v_; // 速度，单位 m/s
  // acceleration (m/s^2)
  double a_; // 加速度，单位 m/s^2
  // jerk (m/s^3)
  double da_; // 竖向 jerk，单位 m/s^3

  // 标志位表示该字段是否已被显式设置
  bool has_s_;
  bool has_v_;
  bool has_a_;
  bool has_da_;
};

/*------------------------------------------------------------------------------------------------------------------*/
// SLPoint: Frenet 坐标系下的点，s 为纵向弧长，l 为横向偏移
class SLPoint {
public:
  const double s() const { return s_; };
  const double l() const { return l_; };
  void set_s(const double s) { s_ = s; };
  void set_l(const double l) { l_ = l; };

private:
  double s_;
  double l_;
};

// FrenetFramePoint: 一个简单的 Frenet 帧表示（该类成员直接初始化为示例值），
// 注意：成员默认是 private，且没有访问器，实际使用时可能需要扩展。
class FrenetFramePoint {
  double s = 1;
  double l = 2;
  double dl = 3;
  double ddl = 4;
};

} // namespace common