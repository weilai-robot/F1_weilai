/*
 * @Date: 2026-10-08 10:30:00
 * @LastEditTime: 2026-10-08 10:30:00
 */

#include "internal/actuator_factory.h"

// projects
#include "internal/common_utils.h"
#include "internal/encos_motor.h"
#include "internal/omni_picker.h"
#include "internal/param_utils.h"
#include "internal/power_flow.h"

namespace xyber {

namespace {

// Common binding params for DCU-side actuators (PowerFlow / OmniPicker):
// channel 1..3 + can_id 1..255. Returns false (logged) on bad values.
bool ParseCanfdBinding(const ActuatorAttachInfo& i, uint8_t& can_id, CtrlChannel& ch) {
  int channel = 0;
  if (!GetRequiredParam(i.params, "can_id", can_id) ||
      !GetRequiredParam(i.params, "channel", channel)) {
    return false;
  }
  if (channel < 1 || channel > 3) {
    LOG_ERROR("Actuator %s invalid channel %d (expect 1~3), create failed.", i.name.c_str(),
              channel);
    return false;
  }
  ch = (CtrlChannel)(channel - 1);
  return true;
}

}  // namespace

std::map<ActuatorType, ActuatorCreator>& ActuatorFactory::Registry() {
  static std::map<ActuatorType, ActuatorCreator> registry;
  return registry;
}

bool ActuatorFactory::Register(ActuatorType type, ActuatorCreator creator) {
  if (!creator) {
    LOG_ERROR("ActuatorFactory register type %d failed: null creator.", (int)type);
    return false;
  }
  if (!Registry().emplace(type, std::move(creator)).second) {
    LOG_ERROR("ActuatorFactory register type %d failed: type already registered.", (int)type);
    return false;
  }
  return true;
}

void ActuatorFactory::RegisterBuiltinActuators() {
  static bool done = [] {
    Register(ActuatorType::POWER_FLOW_R86, [](const ActuatorAttachInfo& i) -> Actuator* {
      uint8_t can_id = 0;
      CtrlChannel ch = CtrlChannel::CTRL_CH1;
      if (!ParseCanfdBinding(i, can_id, ch)) return nullptr;
      WarnUnusedParams(i.params, {"can_id", "channel"}, i.name);
      return new xyber_power_flow::PowerFlowR(i.type, i.name, can_id, ch,
                                              PF_R86_MIT_MODE_DEFAULT_PARAM);
    });
    Register(ActuatorType::POWER_FLOW_R52, [](const ActuatorAttachInfo& i) -> Actuator* {
      uint8_t can_id = 0;
      CtrlChannel ch = CtrlChannel::CTRL_CH1;
      if (!ParseCanfdBinding(i, can_id, ch)) return nullptr;
      WarnUnusedParams(i.params, {"can_id", "channel"}, i.name);
      return new xyber_power_flow::PowerFlowR(i.type, i.name, can_id, ch,
                                              PF_R52_MIT_MODE_DEFAULT_PARAM);
    });
    Register(ActuatorType::POWER_FLOW_L28, [](const ActuatorAttachInfo& i) -> Actuator* {
      uint8_t can_id = 0;
      CtrlChannel ch = CtrlChannel::CTRL_CH1;
      if (!ParseCanfdBinding(i, can_id, ch)) return nullptr;
      WarnUnusedParams(i.params, {"can_id", "channel"}, i.name);
      return new xyber_power_flow::PowerFlowL(i.name, can_id, ch);
    });
    Register(ActuatorType::OMNI_PICKER, [](const ActuatorAttachInfo& i) -> Actuator* {
      uint8_t can_id = 0;
      CtrlChannel ch = CtrlChannel::CTRL_CH1;
      if (!ParseCanfdBinding(i, can_id, ch)) return nullptr;
      WarnUnusedParams(i.params, {"can_id", "channel"}, i.name);
      return new xyber_omni_picker::OmniPicker(i.name, can_id, ch);
    });
    // Both Encos models share the same CAN protocol / actuator class; the type
    // only selects the per-model MIT ranges (common_type.h).
    for (ActuatorType type : {ActuatorType::ENCOS_EC_A8116, ActuatorType::ENCOS_EC_A4315}) {
      Register(type, [](const ActuatorAttachInfo& i) -> Actuator* {
        uint8_t motor_id = 0;
        if (!GetRequiredParam(i.params, "motor_id", motor_id)) {
          // Legacy YAML wrote "can_id" on motors entries; keep the fallback.
          motor_id = GetParam(i.params, "can_id", motor_id);
        }
        if (motor_id == 0) {
          LOG_ERROR("Actuator %s invalid motor_id 0, create failed.", i.name.c_str());
          return nullptr;
        }
        WarnUnusedParams(i.params, {"motor_id", "can_id", "passage"}, i.name);
        return new xyber_encos::EncosMotor(i.type, i.name, motor_id,
                                           GetParam<uint8_t>(i.params, "passage", 0));
      });
    }
    return true;
  }();
  (void)done;
}

Actuator* ActuatorFactory::Create(const ActuatorAttachInfo& info) {
  RegisterBuiltinActuators();

  auto it = Registry().find(info.type);
  if (it == Registry().end()) {
    LOG_ERROR("Actuator %s type %d is not supported.", info.name.c_str(), (int)info.type);
    return nullptr;
  }
  return it->second(info);
}

}  // namespace xyber
