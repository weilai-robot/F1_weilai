/*
 * @Author: zhangmy
 * @Date: 2026-08-13 14:10:06
 * @LastEditors: zhangmy
 * @LastEditTime: 2026-10-08 11:00:00
 */

#pragma once

// cpp
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

// projects
#include "common_type.h"
#include "internal/actuator_base.h"
#include "internal/slave_base.h"

namespace xyber {

static constexpr uint8_t kEncosMaxMotorNum = 6;
static constexpr uint8_t kEncosCan1MotorNum = 3;  // motor[0..2] -> CAN1
static constexpr uint8_t kEncosCan2MotorNum = 3;  // motor[3..5] -> CAN2

#pragma pack(1)

struct Motor_Msg {
  uint32_t id;
  uint8_t rtr;
  uint8_t dlc;
  uint8_t data[8];
};

// PDO layout matches encos board firmware (86B).
// motor[0..2] auto-routed to CAN1, motor[3..5] to CAN2.
struct EncosSlavePacket {
  uint8_t motor_num;
  uint8_t can_ide;
  Motor_Msg motor[kEncosMaxMotorNum];
};

#pragma pack()

static_assert(sizeof(Motor_Msg) == 14, "Motor_Msg size mismatch");
static_assert(sizeof(EncosSlavePacket) == 86, "EncosSlavePacket size mismatch");

class EncosSlave : public SlaveBase {
 public:
  /**
   * @param can_ide CAN frame type: 1=extended (force-position hybrid / MIT),
   *               0=standard. Written into the PDO at creation.
   */
  explicit EncosSlave(std::string name, int32_t ecat_id, uint8_t can_ide = 1);
  virtual ~EncosSlave();

  /**
   * @param info.params["passage"] PDO slot in [1, 6] (0 / missing is rejected).
   *        1~3 -> CAN1, 4~6 -> CAN2 (board firmware auto-routes).
   */
  bool RegisterActuator(Actuator* actr, const ActuatorAttachInfo& info) override;
  Actuator* GetActautor(const std::string& name) override;

  void SetCanIde(uint8_t can_ide) override;

 public:  // Actuator Stuff
  bool EnableAllActuator() override;
  bool EnableActuator(const std::string& name) override;
  bool DisableAllActuator() override;
  bool DisableActuator(const std::string& name) override;
  void ClearError(const std::string& name) override;
  void SetHomingPosition(const std::string& name) override;
  void SaveConfig(const std::string& name) override;
  float GetTempure(const std::string& name) override;
  std::string GetErrorString(const std::string& name) override;
  ActautorState GetPowerState(const std::string& name) override;

  bool SetMode(const std::string& name, ActautorMode mode) override;
  ActautorMode GetMode(const std::string& name) override;

  void SetEffort(const std::string& name, float cur) override;
  float GetEffort(const std::string& name) override;
  void SetVelocity(const std::string& name, float vel) override;
  float GetVelocity(const std::string& name) override;
  void SetPosition(const std::string& name, float pos) override;
  float GetPosition(const std::string& name) override;

  void SetMitParam(const std::string& name, MitParam param) override;
  void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                 float kd) override;

 private:
  virtual bool Init() override;
  virtual void OnStateChangeHook(ethercat_manager::NodeState state) override;
  virtual uint8_t* GetSendBuf() override { return (uint8_t*)&send_buf_; }
  virtual uint8_t* GetRecvBuf() override { return (uint8_t*)&recv_buf_; }

  // The EasyCAT board advertises a larger PDO (e.g. 240B) but only the first
  // sizeof(EncosSlavePacket)=86B are meaningful. Cap process-data copies to the
  // struct size to avoid overreading send_buf_/overwriting recv_buf_.
  virtual void PullSendData(uint8_t* buf, uint16_t len) override;
  virtual void PushRecvData(const uint8_t* buf, uint16_t len) override;

  void UpdateMotorNum();

 private:
  std::unordered_map<std::string, Actuator*> actuator_map_;
  std::array<Actuator*, kEncosMaxMotorNum> passage_slots_{};  // index 0..5 <-> passage 1..6

  EncosSlavePacket send_buf_;
  EncosSlavePacket recv_buf_;
};

}  // namespace xyber
