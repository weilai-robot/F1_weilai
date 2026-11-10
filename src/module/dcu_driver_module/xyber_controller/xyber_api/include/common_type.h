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
  ENCOS_EC_A8116,     // EC-A8116-P1-18H
  ENCOS_EC_A4315,     // EC-A4315-P2-36
  UNKOWN,
};

static ActuatorType StringToType(std::string type) {
  if (type == "POWER_FLOW_R86") return ActuatorType::POWER_FLOW_R86;
  if (type == "POWER_FLOW_R52") return ActuatorType::POWER_FLOW_R52;
  if (type == "POWER_FLOW_L28") return ActuatorType::POWER_FLOW_L28;
  if (type == "OMNI_PICKER") return ActuatorType::OMNI_PICKER;
  if (type == "ENCOS_EC_A8116" || type == "EC-A8116-P1-18H") return ActuatorType::ENCOS_EC_A8116;
  if (type == "ENCOS_EC_A4315" || type == "EC-A4315-P2-36") return ActuatorType::ENCOS_EC_A4315;
  return ActuatorType::UNKOWN;
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
  // MIT type-1 feedback phase-current range (Encos only; PowerFlow ignores it)
  float cur_min = -30.0f;
  float cur_max = 30.0f;
  // Torque constant (N·m/A) to convert feedback current to torque; 0 = unknown
  float kt = 0.0f;
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

// EC-A8116-P1-18H: TOR ±130 N·m, CUR ±70 A, Kt 2.34 N·m/A
// (bench-queried 2026-11-09, doc/电机参数.txt; NOTE: differs from
// HW-SPEC-0005 §2.3 which records ±90/±60/2.1 — spec row pending update)
#define ENCOS_EC_A8116_MIT_PARAM \
  {                              \
      .pos_min = -12.5f,         \
      .pos_max = 12.5f,          \
      .vel_min = -18.0f,         \
      .vel_max = 18.0f,          \
      .toq_min = -130.0f,        \
      .toq_max = 130.0f,         \
      .kp_min = 0.0f,            \
      .kp_max = 500.0f,          \
      .kd_min = 0.0f,            \
      .kd_max = 5.0f,            \
      .cur_min = -70.0f,         \
      .cur_max = 70.0f,          \
      .kt = 2.34f,               \
  }

// EC-A4315-P2-36: TOR ±70 N·m, CUR ±30 A, Kt 2.8 N·m/A
// (HW-SPEC-0005 §2.3; all values bench-verified 2026-11-09, doc/电机参数.txt)
#define ENCOS_EC_A4315_MIT_PARAM \
  {                              \
      .pos_min = -12.5f,         \
      .pos_max = 12.5f,          \
      .vel_min = -18.0f,         \
      .vel_max = 18.0f,          \
      .toq_min = -70.0f,         \
      .toq_max = 70.0f,          \
      .kp_min = 0.0f,            \
      .kp_max = 500.0f,          \
      .kd_min = 0.0f,            \
      .kd_max = 5.0f,            \
      .cur_min = -30.0f,         \
      .cur_max = 30.0f,          \
      .kt = 2.8f,                \
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
 *               type-specific goes into params and is read by the slave's
 *               own creator.
 */
struct SlaveConfig {
  std::string type;        // "dcu" | "encos" | ...
  std::string name;
  int32_t ecat_id = 0;
  ParamMap params;         // per-type; read by the slave's own creator
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