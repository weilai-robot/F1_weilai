/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditTime: 2026-10-08 11:00:00
 */

#include "xyber_controller.h"

#include <functional>
#include <map>
#include <stdexcept>
#include <vector>

#include "common_type.h"
#include "internal/actuator_factory.h"
#include "internal/ethercat_manager.h"
#include "internal/slave_base.h"
#include "internal/slave_factory.h"
#include "internal/version.h"

using namespace ethercat_manager;

namespace xyber {

// global hidden variable
XyberController* XyberController::instance_ = nullptr;

static EthercatConfig ecat_config_;
static EthercatManager ecat_manager_;
static std::unordered_map<std::string, SlaveBase*> slave_map_;
static std::unordered_map<std::string, SlaveBase*> actuator_slave_map_;

XyberController::XyberController() {
  LOG_INFO("XyberController is created. Ver %d.%d.%d, Compiled in %s %s.", MAIN_VERSION,
           SUB_VERSION, PATCH_VERSION, __DATE__, __TIME__);
}

XyberController::~XyberController() {
  Stop();
  for (auto& slave : slave_map_) {
    delete slave.second;
  }
}

XyberController* XyberController::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new XyberController();
  }
  return instance_;
}

std::string XyberController::GetVersion() {
  return std::to_string(MAIN_VERSION) + "." + std::to_string(SUB_VERSION) + "." +
         std::to_string(PATCH_VERSION);
}

bool XyberController::CreateSlave(const SlaveConfig& cfg) {
  if (cfg.name.empty()) {
    LOG_ERROR("CreateSlave failed: empty name.");
    return false;
  }
  if (slave_map_.find(cfg.name) != slave_map_.end()) {
    LOG_ERROR("Slave %s is already created, add failed.", cfg.name.c_str());
    return false;
  }

  // slave type string ("dcu" / "encos" / ...) is resolved by the factory
  SlaveBase* slave = SlaveFactory::Create(cfg);
  if (slave == nullptr) {
    LOG_ERROR("CreateSlave %s failed: unknown or invalid type %s.", cfg.name.c_str(),
              cfg.type.c_str());
    return false;
  }
  slave_map_[cfg.name] = slave;
  return true;
}

bool XyberController::AttachActuator(const std::string& slave_name,
                                     const ActuatorAttachInfo& info) {
  if (info.name.empty()) {
    LOG_ERROR("AttachActuator failed: empty actuator name.");
    return false;
  }
  auto slave_it = slave_map_.find(slave_name);
  if (slave_it == slave_map_.end()) {
    LOG_ERROR("Can`t find slave %s, attach actuator %s failed.", slave_name.c_str(),
              info.name.c_str());
    return false;
  }
  if (actuator_slave_map_.find(info.name) != actuator_slave_map_.end()) {
    LOG_ERROR("Actuator %s is already attached on slave %s.", info.name.c_str(),
              actuator_slave_map_[info.name]->GetName().c_str());
    return false;
  }

  // create actuator via factory, then let the slave bind it to its own PDO layout
  Actuator* atcr = ActuatorFactory::Create(info);
  if (atcr == nullptr) {
    LOG_ERROR("Actuator %s create failed (type %d).", info.name.c_str(), (int)info.type);
    return false;
  }
  if (!slave_it->second->RegisterActuator(atcr, info)) {
    delete atcr;
    return false;
  }

  LOG_DEBUG("Attach actuator %s type %d on slave %s.", info.name.c_str(), (int)info.type,
            slave_name.c_str());
  actuator_slave_map_[info.name] = slave_it->second;
  return true;
}

// ---------------------------------------------------------------------------
// Deprecated APIs below, kept as thin wrappers for backward compatibility.
// In-repo callers have moved to CreateSlave / AttachActuator.
// ---------------------------------------------------------------------------

bool XyberController::CreateDcu(std::string name, uint8_t id) {
  return CreateSlave(
      SlaveConfig{.type = "dcu", .name = name, .ecat_id = (int32_t)id, .params = {}});
}

bool XyberController::AttachActuator(std::string dcu_name, CtrlChannel ch, ActuatorType type,
                                     std::string actuator_name, uint8_t id) {
  return AttachActuator(dcu_name, ActuatorAttachInfo{
      .type = type,
      .name = actuator_name,
      .params = {{"channel", std::to_string((int)ch + 1)}, {"can_id", std::to_string(id)}}});
}

bool XyberController::CreateEncosSlave(std::string name, uint8_t ethercat_id) {
  return CreateSlave(SlaveConfig{.type = "encos", .name = name,
                                  .ecat_id = (int32_t)ethercat_id, .params = {}});
}

bool XyberController::AttachEncosMotor(std::string slave_name, uint8_t passage, ActuatorType type,
                                     std::string actuator_name, uint8_t motor_id) {
  return AttachActuator(slave_name, ActuatorAttachInfo{
      .type = type,
      .name = actuator_name,
      .params = {{"passage", std::to_string(passage)},
                 {"motor_id", std::to_string(motor_id)}}});
}

bool XyberController::SetRealtime(int rt_priority, int bind_cpu) {
  if (is_running_) {
    LOG_WARN("XyberController is running, can not set realtime.");
    return false;
  }
  ecat_config_.bind_cpu = bind_cpu;
  ecat_config_.rt_priority = rt_priority;
  return true;
}

bool XyberController::Start(std::string ifname, uint64_t cycle_ns, bool enable_dc) {
  if (is_running_) {
    LOG_WARN("XyberController is already running.");
    return true;
  }
  // register all slaves (dcu / encos / ...) to ethercat manager
  for (const auto& slave : slave_map_) {
    ecat_manager_.RegisterNode(slave.second);
  }
  ecat_config_.ifname = ifname;
  ecat_config_.enable_dc = enable_dc;
  ecat_config_.cycle_time_ns = cycle_ns;
  is_running_ = ecat_manager_.Start(ecat_config_);
  if (is_running_) {
    LOG_INFO("XyberController start successfully.");
  } else {
    LOG_ERROR("XyberController start failed.");
  }
  return is_running_;
}

void XyberController::Stop() {
  if (!is_running_) return;

  ecat_manager_.Stop();
  is_running_ = false;
  LOG_INFO("XyberController stop successfully.");
}

DcuImu XyberController::GetDcuImuData(const std::string& name) {
  auto it = slave_map_.find(name);
  if (it == slave_map_.end()) return DcuImu();

  return it->second->GetImuData();
}

void XyberController::ApplyDcuImuOffset(const std::string& name) {
  auto it = slave_map_.find(name);
  if (it == slave_map_.end()) return;
  it->second->ApplyImuOffset();
}

bool XyberController::EnableAllActuator() {
  for (const auto& [name, slave] : slave_map_) {
    if (!slave->EnableAllActuator()) return false;
  }
  return true;
}

bool XyberController::EnableActuator(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->EnableActuator(name);

  return false;
}

void XyberController::ClearError(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) it->second->ClearError(name);
}

bool XyberController::DisableAllActuator() {
  for (const auto& [name, slave] : slave_map_) {
    if (!slave->DisableAllActuator()) return false;
  }
  return true;
}

bool XyberController::DisableActuator(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->DisableActuator(name);

  return false;
}

float XyberController::GetTempure(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetTempure(name);

  return 0;
}

ActautorState XyberController::GetPowerState(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetPowerState(name);

  return STATE_DISABLE;
}

ActautorMode XyberController::GetMode(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetMode(name);

  return MODE_CURRENT;
}

float XyberController::GetEffort(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetEffort(name);

  return 0;
}

float XyberController::GetVelocity(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetVelocity(name);

  return 0;
}

float XyberController::GetPosition(const std::string& name) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) return it->second->GetPosition(name);

  return 0;
}

void XyberController::SetMitParam(const std::string& name, MitParam param) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) it->second->SetMitParam(name, param);
}

void XyberController::SetMitCmd(const std::string& name, float pos, float vel, float effort,
                                float kp, float kd) {
  auto it = actuator_slave_map_.find(name);
  if (it != actuator_slave_map_.end()) it->second->SetMitCmd(name, pos, vel, effort, kp, kd);
}

}  // namespace xyber
