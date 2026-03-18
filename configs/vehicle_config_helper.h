/**
 * @file vehicle_config_helper.h
 * @brief 车辆配置帮助类（Stub 文件）
 *
 * 本文件提供 VehicleConfigHelper 类的空实现，
 * 用于兼容可能的遗留代码引用。
 */

#pragma once

#include "configs/vehicle_config.h"

/**
 * @class VehicleConfigHelper
 * @brief 车辆配置帮助类
 *
 * 提供获取默认车辆参数的静态方法。
 * 当前实现返回默认的 VehicleParam 实例。
 */
class VehicleConfigHelper {
public:
  /**
   * @brief 获取车辆参数实例
   * @return 默认的车辆参数
   */
  static VehicleParam GetConfig() {
    return VehicleParam();
  }
};
