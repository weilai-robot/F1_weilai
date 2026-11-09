/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: richie.li
 * @LastEditTime: 2024-10-21 20:18:09
 */

#pragma once

// CPP
#include <cmath>
#include <cstdint>
#include <map>
#include <string>

#define M_2PI (2.0f * M_PI)

namespace xyber {

enum class CtrlChannel : uint8_t {
  CTRL_CH1 = 0,
  CTRL_CH2 = 1,
  CTRL_CH3 = 2,
};

enum class ActuatorType {
  POWER_FLOW_R86,
  POWER_FLOW_R52,
  POWER_FLOW_L28,
  OMNI_PICKER,
  ENCOS_MOTOR,        // generic Encos motor (same protocol as the two models below)
  ENCOS_EC_A8112,     // EC-A8112-P1-18H
  ENCOS_EC_A4314,     // EC-A4314-P2-36
  UNKOWN,
};

static ActuatorType StringToType(std::string type) {
  if (type == "POWER_FLOW_R86") return ActuatorType::POWER_FLOW_R86;
  if (type == "POWER_FLOW_R52") return ActuatorType::POWER_FLOW_R52;
  if (type == "POWER_FLOW_L28") return ActuatorType::POWER_FLOW_L28;
  if (type == "OMNI_PICKER") return ActuatorType::OMNI_PICKER;
  if (type == "ENCOS_MOTOR") return ActuatorType::ENCOS_MOTOR;
  if (type == "ENCOS_EC_A8112" || type == "EC-A8112-P1-18H") return ActuatorType::ENCOS_EC_A8112;
  if (type == "ENCOS_EC_A4314" || type == "EC-A4314-P2-36") return ActuatorType::ENCOS_EC_A4314;
  return ActuatorType::UNKOWN;
}

static bool IsEncosMotorType(ActuatorType type) {
  return type == ActuatorType::ENCOS_MOTOR || type == ActuatorType::ENCOS_EC_A8112 ||
         type == ActuatorType::ENCOS_EC_A4314;
}

enum ActautorState : uint8_t {
  STATE_DISABLE = 0,
  STATE_ENABLE = 1,
  STATE_CALIBRATION = 2,
};

enum ActautorMode : uint8_t {
  MODE_CURRENT = 0,
  MODE_CURRENT_RAMP = 1,
  MODE_VELOCITY = 2,
  MODE_VELOCITY_RAMP = 3,
  MODE_POSITION = 4,
  MODE_POSITION_RAMP = 5,
  MODE_MIT = 6,
};

enum class DcuImuCmd : uint8_t {
  NONE = 0,
  IMU_APPLY_OFFSET = 1,
};

struct DcuImu {
  float acc[3];
  float gyro[3];
  float quat[4];
};

/**
 * @description: Mit参数
 */
struct MitParam {
  float pos_min = 0.0;
  float pos_max = 0.0;
  float vel_min = 0.0;
  float vel_max = 0.0;
  float toq_min = 0.0;
  float toq_max = 0.0;
  float kp_min = 0.0;
  float kp_max = 0.0;
  float kd_min = 0.0;
  float kd_max = 0.0;
};

#define PF_R52_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -1.0f * M_2PI,       \
      .pos_max = 1.0f * M_2PI,        \
      .vel_min = -2.0f * M_2PI,       \
      .vel_max = 2.0f * M_2PI,        \
      .toq_min = -50.0f,              \
      .toq_max = 50.0f,               \
      .kp_min = 0.0f,                 \
      .kp_max = 500.0f,               \
      .kd_min = 0.0f,                 \
      .kd_max = 8.0f,                 \
  }

#define PF_R86_MIT_MODE_DEFAULT_PARAM \
  {                                   \
      .pos_min = -1.0f * M_2PI,       \
      .pos_max = 1.0f * M_2PI,        \
      .vel_min = -2.0f * M_2PI,       \
      .vel_max = 2.0f * M_2PI,        \
      .toq_min = -100.0f,             \
      .toq_max = 100.0f,              \
      .kp_min = 0.0f,                 \
      .kp_max = 500.0f,               \
      .kd_min = 0.0f,                 \
      .kd_max = 8.0f,                 \
  }

// Encos (因克斯) force-position hybrid / MIT ranges from motor_control.c
#define ENCOS_MIT_MODE_DEFAULT_PARAM \
  {                                \
      .pos_min = -12.5f,           \
      .pos_max = 12.5f,            \
      .vel_min = -18.0f,           \
      .vel_max = 18.0f,            \
      .toq_min = -30.0f,           \
      .toq_max = 30.0f,            \
      .kp_min = 0.0f,              \
      .kp_max = 500.0f,            \
      .kd_min = 0.0f,              \
      .kd_max = 5.0f,              \
  }

/**
 * @description: Free-form key/value params block attached to slave / actuator
 *               configs. Values are stored as strings; readers convert with
 *               the GetParam helpers (internal/param_utils.h). The set of keys
 *               is owned by the slave / actuator type that consumes it, so
 *               common structs never grow per-type fields again.
 */
using ParamMap = std::map<std::string, std::string>;

/**
 * @description: Slave creation params shared by all slave types. Fixed fields
 *               hold what every slave has (identity on the bus); everything
 *               type-specific (can_ide for encos, ...) goes into params and
 *               is read by the slave's own creator.
 */
struct SlaveConfig {
  std::string type;        // "dcu" | "encos" | ...
  std::string name;
  int32_t ecat_id = 0;
  ParamMap params;         // e.g. encos: can_ide; dcu: none
};

/**
 * @description: Actuator creation + binding params. Fixed fields hold what
 *               every actuator has (identity); binding fields (channel /
 *               can_id for dcu, passage / motor_id for encos, ...) go into
 *               params and are read by the actuator creator and the slave's
 *               RegisterActuator.
 */
struct ActuatorAttachInfo {
  ActuatorType type = ActuatorType::UNKOWN;
  std::string name;
  ParamMap params;         // e.g. dcu: channel, can_id; encos: passage, motor_id
};

}  // namespace xyber