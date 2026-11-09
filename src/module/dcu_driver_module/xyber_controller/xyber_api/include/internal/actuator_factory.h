/*
 * @Author: zhangmy
 * @Date: 2026-10-08 10:30:00
 * @LastEditors: zhangmy
 * @LastEditTime: 2026-10-08 10:30:00
 */

#pragma once

// cpp
#include <functional>
#include <map>

// projects
#include "common_type.h"
#include "internal/actuator_base.h"

namespace xyber {

using ActuatorCreator = std::function<Actuator*(const ActuatorAttachInfo&)>;

/**
 * @description: Registry-style actuator factory. Maps an ActuatorType to its
 *               creator. New actuator classes plug in with a single Register()
 *               call in RegisterBuiltinActuators() (actuator_factory.cpp).
 */
class ActuatorFactory {
 public:
  /**
   * @brief: Register a creator under an actuator type key
   * @return {bool} false if the key is already taken or the creator is null
   */
  static bool Register(ActuatorType type, ActuatorCreator creator);

  /**
   * @brief: Create an actuator by ActuatorAttachInfo.type. Creators validate
   *         their own id fields (can_id / motor_id).
   * @return {Actuator*} nullptr for unsupported type or invalid params (logged)
   */
  static Actuator* Create(const ActuatorAttachInfo& info);

 private:
  static void RegisterBuiltinActuators();
  static std::map<ActuatorType, ActuatorCreator>& Registry();
};

}  // namespace xyber
