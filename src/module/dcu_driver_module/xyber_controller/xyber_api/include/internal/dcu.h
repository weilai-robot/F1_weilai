/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditors: zhangmy
 * @LastEditTime: 2026-10-08 11:00:00
 */

#pragma once

// cpp
#include <unordered_map>
#include <vector>

// projects
#include "common_type.h"
#include "internal/actuator_base.h"
#include "internal/slave_base.h"

namespace xyber {

#pragma pack(1)

struct DcuSendPacket {
  struct {
    uint8_t ctrl;
    uint8_t data[64];
  } canfd[3];
  uint8_t imu_cmd;
  uint8_t reserved[44];
};

struct DcuRecvPacket {
  uint8_t canfd[3][64];
  struct {
    uint32_t acc[3];
    uint32_t gyro[3];
    uint32_t quat[4];
  } imu;
  uint8_t reserved[8];
};

#pragma pack()

class Dcu : public SlaveBase {
 public:
  explicit Dcu(std::string name, int32_t ecat_id);
  virtual ~Dcu();

  // Binding uses the actuator's own ctrl channel + can id; the info is for
  // logging and future extensions. Returns false when the channel is full.
  bool RegisterActuator(Actuator* actr, const ActuatorAttachInfo& info) override;
  Actuator* GetActautor(const std::string& name) override;
  void SetChannelId(CtrlChannel ch, uint8_t id);

  DcuImu GetImuData() override;
  void ApplyImuOffset() override;
  bool HasImu() const override { return true; }

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

 private:
  std::unordered_map<std::string, Actuator*> actuator_map_;
  std::unordered_map<CtrlChannel, std::vector<Actuator*>> ctrl_channel_map_;

  DcuSendPacket send_buf_;
  DcuRecvPacket recv_buf_;
};

}  // namespace xyber
