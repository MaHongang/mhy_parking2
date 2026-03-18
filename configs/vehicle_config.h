/**
 * @file vehicle_config.h
 * @brief 车辆配置参数定义
 *
 * 本文件定义了车辆的各种物理参数和控制延迟参数。
 * 这些参数是规划和控制模块的基础配置，包括：
 * - 车辆几何尺寸（长宽高、轴距等）
 * - 动力学约束（最大加速度、最大转向角等）
 * - 控制延迟参数（转向、油门、制动的响应时间）
 *
 * 参数说明：
 * - 所有长度单位为米 [m]
 * - 所有角度单位为弧度 [rad]
 * - 所有速度单位为米/秒 [m/s]
 * - 所有加速度单位为米/秒² [m/s²]
 */

#pragma once

#include <string>

/**
 * @struct SteeringLatencyParam
 * @brief 转向系统延迟参数
 *
 * 描述从发出转向指令到方向盘实际响应的时间特性。
 * 用于控制器的前馈补偿和稳定性分析。
 */
struct SteeringLatencyParam {
  double dead_time = 0.1;       ///< 死区时间 [s]：指令发出到开始响应的延迟
  double rise_time = 0.17;      ///< 上升时间 [s]：从 10% 到 90% 响应的时间
  double peak_time = 0.0;       ///< 峰值时间 [s]：达到第一个峰值的时间
  double settling_time = 0.0;   ///< 稳定时间 [s]：进入稳态误差带的时间
};

/**
 * @struct ThrottleLatencyParam
 * @brief 油门系统延迟参数
 *
 * 描述油门踏板指令到实际加速响应的时间特性。
 */
struct ThrottleLatencyParam {
  double dead_time = 0.03;      ///< 死区时间 [s]
  double rise_time = 0.0;       ///< 上升时间 [s]
  double peak_time = 0.0;       ///< 峰值时间 [s]
  double settling_time = 0.0;   ///< 稳定时间 [s]
};

/**
 * @struct BrakeLatencyParam
 * @brief 制动系统延迟参数
 *
 * 描述制动踏板指令到实际减速响应的时间特性。
 */
struct BrakeLatencyParam {
  double dead_time = 0.0;       ///< 死区时间 [s]
  double rise_time = 0.0;       ///< 上升时间 [s]
  double peak_time = 0.0;       ///< 峰值时间 [s]
  double settling_time = 0.0;   ///< 稳定时间 [s]
};

/**
 * @class VehicleParam
 * @brief 车辆参数配置类
 *
 * 存储车辆的所有物理参数，用于：
 * - 碰撞检测：使用几何尺寸计算包围盒
 * - 运动学模型：使用轴距计算转弯半径
 * - 规划约束：使用动力学极限约束轨迹
 * - 控制补偿：使用延迟参数进行前馈
 *
 * 参考点说明：
 * - 车辆参考点（center）位于后轴中心
 * - 所有"到中心的距离"都是相对于后轴中心
 *
 * 默认参数基于 LINCOLN_MKZ 车型
 */
class VehicleParam {

public:
  VehicleParam() = default;
  ~VehicleParam() = default;

  // ==================== 基本信息 ====================
  std::string brand = "LINCOLN_MKZ";  ///< 车辆品牌/型号
  int vehicle_id = 2;                  ///< 车辆 ID

  // ==================== 几何尺寸 ====================
  // 车辆参考点（center）位于后轴中心
  double front_edge_to_center = 3.89;   ///< 车头到后轴中心的距离 [m]
  double back_edge_to_center = 1.043;   ///< 车尾到后轴中心的距离 [m]
  double left_edge_to_center = 1.055;   ///< 左侧边缘到后轴中心的距离 [m]
  double right_edge_to_center = 1.055;  ///< 右侧边缘到后轴中心的距离 [m]

  double length = 4.933;  ///< 车辆总长度 [m]
  double width = 2.11;    ///< 车辆总宽度 [m]
  double height = 1.48;   ///< 车辆总高度 [m]

  // ==================== 动力学约束 ====================
  double min_turn_radius = 5.05386147161;  ///< 最小转弯半径 [m]
  double max_acceleration = 2;              ///< 最大加速度 [m/s²]
  double max_deceleration = -6;             ///< 最大减速度 [m/s²]（负值）
  double max_parking_speed = 4;             ///< 泊车最大速度 [m/s]

  // ==================== 转向系统参数 ====================
  /**
   * 方向盘转角与车轮转角的关系：
   * 前轮转角 = 方向盘转角 / steer_ratio
   *
   * max_steer_angle 是最大方向盘转角（弧度）
   * 实际前轮最大转角 = max_steer_angle / steer_ratio ≈ 0.513 rad ≈ 29.4°
   */
  double max_steer_angle = 8.20304748437;       ///< 最大方向盘转角 [rad]
  double max_steer_angle_rate = 8.55211;        ///< 最大方向盘转速 [rad/s]
  double min_steer_angle_rate = -8.55211;       ///< 最小方向盘转速 [rad/s]
  double steer_ratio = 16;                      ///< 方向盘转角与车轮转角之比

  // ==================== 轮胎/底盘参数 ====================
  double wheel_base = 2.8448;           ///< 轴距：前后轮之间的距离 [m]
  double wheel_rolling_radius = 0.335;  ///< 轮胎有效滚动半径 [m]

  // ==================== 速度/制动参数 ====================
  float max_abs_speed_when_stopped = 0.2;  ///< 判定为停止的最大速度阈值 [m/s]
  double brake_deadzone = 14.5;            ///< 制动踏板死区 [%]
  double throttle_deadzone = 15.4;         ///< 油门踏板死区 [%]

  // ==================== 延迟参数 ====================
  SteeringLatencyParam steering_latency_param;  ///< 转向延迟参数
  ThrottleLatencyParam throttle_latency_param;  ///< 油门延迟参数
  BrakeLatencyParam brake_latency_param;        ///< 制动延迟参数
};
