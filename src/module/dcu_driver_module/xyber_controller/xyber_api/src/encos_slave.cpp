/*
 * @Date: 2026-08-13 16:39:00
 * @LastEditTime: 2026-10-08 11:00:00
 */

#include "internal/encos_slave.h"

#include <cstring>
#include <algorithm>

#include "internal/common_utils.h"
#include "internal/param_utils.h"

#define COM_TIME_UP_MS 1000

using namespace ethercat_manager;
using namespace std::chrono_literals;

namespace xyber {

EncosSlave::EncosSlave(std::string name, int32_t ecat_id)
    : SlaveBase(ecat_id, name) {
  LOG_DEBUG("EncosSlave %s id %u created.", name_.c_str(), id_);
  memset(&send_buf_, 0, sizeof(send_buf_));
  memset(&recv_buf_, 0, sizeof(recv_buf_));
  // Manual §1: control/query frames use standard 11-bit IDs (0x01~0x7FF).
  // can_ide stays a packet field for board-firmware layout compatibility.
  send_buf_.can_ide = 0;
  passage_slots_.fill(nullptr);
}

EncosSlave::~EncosSlave() {
  for (auto& it : actuator_map_) {
    delete it.second;
  }
}

bool EncosSlave::Init() {
  // EasyCAT advertises a PDO larger than the 86B struct (e.g. 240B); only the
  // first sizeof(EncosSlavePacket) bytes are meaningful. Require the PDO to be at
  // least as large as the struct; the surplus is ignored (see Pull/Push below).
  if (send_size_ < sizeof(EncosSlavePacket)) {
    LOG_ERROR("EncosSlave %s send_size too small: %u < %lu", name_.c_str(), send_size_,
              sizeof(EncosSlavePacket));
    return false;
  }
  if (recv_size_ < sizeof(EncosSlavePacket)) {
    LOG_ERROR("EncosSlave %s recv_size too small: %u < %lu", name_.c_str(), recv_size_,
              sizeof(EncosSlavePacket));
    return false;
  }
  if (send_size_ != sizeof(EncosSlavePacket) || recv_size_ != sizeof(EncosSlavePacket)) {
    LOG_INFO("EncosSlave %s PDO larger than struct: send %u (use %lu), recv %u (use %lu). "
             "Surplus bytes ignored.", name_.c_str(), send_size_, sizeof(EncosSlavePacket),
             recv_size_, sizeof(EncosSlavePacket));
  }
  return true;
}

void EncosSlave::PullSendData(uint8_t* buf, uint16_t len) {
  std::lock_guard<std::mutex> lock(send_mtx_);
  uint16_t n = sizeof(EncosSlavePacket);
  if (len < n) n = len;
  const uint8_t* src = reinterpret_cast<const uint8_t*>(&send_buf_);
  std::copy(src, src + n, buf);
}

void EncosSlave::PushRecvData(const uint8_t* buf, uint16_t len) {
  std::lock_guard<std::mutex> lock(recv_mtx_);
  uint16_t n = sizeof(EncosSlavePacket);
  if (len < n) n = len;
  uint8_t* dst = reinterpret_cast<uint8_t*>(&recv_buf_);
  std::copy(buf, buf + n, dst);
}

void EncosSlave::OnStateChangeHook(NodeState next) {
  LOG_DEBUG("EncosSlave %s %d, state change: %s -> %s", name_.c_str(), id_,
            NodeStateStr(node_state_).c_str(), NodeStateStr(next).c_str());
}

void EncosSlave::UpdateMotorNum() {
  // motor_num is informational only (motor count): the board firmware scans
  // ALL 6 slots and skips idle ones (dlc==0) - it never uses motor_num as a
  // scan bound (the vendor demo master leaves it at 0 and motors still work).
  // Gapped layouts (e.g. CAN1 x2 + CAN2 x3, empty slot 2) are therefore fine
  // as long as unused slots stay zeroed, which send_buf_ guarantees.
  uint8_t count = 0;
  for (auto* actr : passage_slots_) {
    if (actr) ++count;
  }
  send_buf_.motor_num = count;
}

bool EncosSlave::RegisterActuator(Actuator* actr, const ActuatorAttachInfo& info) {
  if (!actr) {
    LOG_ERROR("EncosSlave %s RegisterActuator failed: null actuator.", name_.c_str());
    return false;
  }
  const uint8_t passage = GetParam<uint8_t>(info.params, "passage", 0);
  if (passage < 1 || passage > kEncosMaxMotorNum) {
    LOG_ERROR("EncosSlave %s RegisterActuator %s failed: invalid passage %u (expect 1~%u).",
              name_.c_str(), actr->GetName().c_str(), passage, kEncosMaxMotorNum);
    return false;
  }

  const uint8_t idx = passage - 1;
  if (passage_slots_[idx] != nullptr) {
    LOG_ERROR("EncosSlave %s RegisterActuator %s failed: passage %u already used by %s.",
              name_.c_str(), actr->GetName().c_str(), passage,
              passage_slots_[idx]->GetName().c_str());
    return false;
  }
  if (actuator_map_.find(actr->GetName()) != actuator_map_.end()) {
    LOG_ERROR("EncosSlave %s RegisterActuator failed: name %s already registered.", name_.c_str(),
              actr->GetName().c_str());
    return false;
  }

  // Soft check: CAN1 uses slots 1~3, CAN2 uses 4~6 (firmware auto-routes).
  uint8_t can_group_count = 0;
  const uint8_t begin = (passage <= kEncosCan1MotorNum) ? 0 : kEncosCan1MotorNum;
  const uint8_t end = begin + ((passage <= kEncosCan1MotorNum) ? kEncosCan1MotorNum : kEncosCan2MotorNum);
  for (uint8_t i = begin; i < end; ++i) {
    if (passage_slots_[i]) ++can_group_count;
  }
  if (can_group_count >= kEncosCan1MotorNum) {
    LOG_ERROR("EncosSlave %s RegisterActuator %s failed: CAN%u already has %u motors.", name_.c_str(),
              actr->GetName().c_str(), (passage <= kEncosCan1MotorNum) ? 1 : 2, can_group_count);
    return false;
  }

  actr->SetDataFiled(reinterpret_cast<uint8_t*>(&send_buf_.motor[idx]),
                     reinterpret_cast<uint8_t*>(&recv_buf_.motor[idx]));
  passage_slots_[idx] = actr;
  actuator_map_[actr->GetName()] = actr;
  UpdateMotorNum();

  // A gap below the highest passage (e.g. CAN1 x2 + CAN2 x3 -> empty slot 2)
  // is legal: idle slots stay dlc==0 (send_buf_ is zeroed and only registered
  // motors write) and the board skips them when scanning all 6 slots.
  for (uint8_t i = 0; i < idx; ++i) {
    if (passage_slots_[i] == nullptr) {
      LOG_WARN(
          "EncosSlave %s: passage gap detected (passage %u empty while %u is used); "
          "idle slot stays dlc==0 and is skipped by the board.",
          name_.c_str(), i + 1, passage);
      break;
    }
  }

  LOG_DEBUG("EncosSlave %s attach %s to passage %u (CAN%u).", name_.c_str(), actr->GetName().c_str(),
            passage, (passage <= kEncosCan1MotorNum) ? 1 : 2);
  return true;
}

Actuator* EncosSlave::GetActautor(const std::string& name) {
  auto it = actuator_map_.find(name);
  if (it != actuator_map_.end()) return it->second;
  return nullptr;
}

bool EncosSlave::EnableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    (void)actr;
    if (!EnableActuator(name)) return false;
  }
  return true;
}

bool EncosSlave::EnableActuator(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return false;

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->RequestState(STATE_ENABLE);
  }

  ActautorState ret = STATE_DISABLE;
  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);
    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == STATE_ENABLE) break;
  }
  if (ret == STATE_ENABLE) {
    LOG_DEBUG("Encos actuator %s enable success.", name.c_str());
  } else {
    LOG_ERROR("Encos actuator %s enable failed (protocol may be unimplemented).", name.c_str());
  }
  return ret == STATE_ENABLE;
}

bool EncosSlave::DisableAllActuator() {
  for (const auto& [name, actr] : actuator_map_) {
    (void)actr;
    if (!DisableActuator(name)) return false;
  }
  return true;
}

bool EncosSlave::DisableActuator(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return false;

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->RequestState(STATE_DISABLE);
  }

  ActautorState ret = STATE_ENABLE;
  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);
    std::lock_guard<std::mutex> lock(recv_mtx_);
    ret = actr->GetPowerState();
    if (ret == STATE_DISABLE) break;
  }
  return ret == STATE_DISABLE;
}

void EncosSlave::ClearError(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->ClearError();
}

void EncosSlave::SetHomingPosition(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetHomingPosition();
}

void EncosSlave::SaveConfig(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SaveConfig();
}

float EncosSlave::GetTempure(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetTempure();
}

std::string EncosSlave::GetErrorString(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return "";
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetErrorString();
}

ActautorState EncosSlave::GetPowerState(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return STATE_DISABLE;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPowerState();
}

bool EncosSlave::SetMode(const std::string& name, ActautorMode mode) {
  auto actr = GetActautor(name);
  if (!actr) return false;

  {
    std::lock_guard<std::mutex> lock(send_mtx_);
    actr->SetMode(mode);
  }
  for (size_t i = 0; i < COM_TIME_UP_MS / 10; i++) {
    std::this_thread::sleep_for(10ms);
    std::lock_guard<std::mutex> lock(recv_mtx_);
    if (actr->GetMode() == mode) return true;
  }
  return false;
}

ActautorMode EncosSlave::GetMode(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return MODE_CURRENT;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetMode();
}

void EncosSlave::SetEffort(const std::string& name, float cur) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetEffort(cur);
}

float EncosSlave::GetEffort(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetEffort();
}

void EncosSlave::SetVelocity(const std::string& name, float vel) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetVelocity(vel);
}

float EncosSlave::GetVelocity(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetVelocity();
}

void EncosSlave::SetPosition(const std::string& name, float pos) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetPosition(pos);
}

float EncosSlave::GetPosition(const std::string& name) {
  auto actr = GetActautor(name);
  if (!actr) return 0;
  std::lock_guard<std::mutex> lock(recv_mtx_);
  return actr->GetPosition();
}

void EncosSlave::SetMitParam(const std::string& name, MitParam param) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitParam(param);
}

void EncosSlave::SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                         float kd) {
  auto actr = GetActautor(name);
  if (!actr) return;
  std::lock_guard<std::mutex> lock(send_mtx_);
  actr->SetMitCmd(pos, vel, effort, kp, kd);
}

}  // namespace xyber
