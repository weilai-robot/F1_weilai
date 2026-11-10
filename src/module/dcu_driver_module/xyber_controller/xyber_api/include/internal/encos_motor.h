/*
 * @Date: 2026-08-13 16:39:00
 * @LastEditTime: 2026-08-14 09:40:00
 */

#pragma once

// projects
#include "common_type.h"
#include "internal/actuator_base.h"
#include "internal/encos_slave.h"

namespace xyber_encos {

/**
 * Encos (因克斯) motor actuator.
 * Protocol follows soem_demo_cpp/app_cpp/motor_control.c
 * Primary path for robot control: force-position hybrid (SetMitCmd).
 */
class EncosMotor : public xyber::Actuator {
 public:
  /**
   * @param type ENCOS_EC_A8112 / ENCOS_EC_A4315 (same CAN protocol)
   * @param name actuator name
   * @param motor_id CAN id written into Motor_Msg.id
   * @param passage PDO slot in [1, 6]
   */
  explicit EncosMotor(xyber::ActuatorType type, std::string name, uint8_t motor_id, uint8_t passage);
  ~EncosMotor() override = default;

  uint8_t GetPassage() const { return passage_; }

 public:
  void RequestState(xyber::ActautorState state) override;
  void ClearError() override;
  void SetHomingPosition() override;
  void SaveConfig() override;

  void SetMode(xyber::ActautorMode mode) override;
  std::string GetErrorString() override;
  float GetTempure() override;
  xyber::ActautorState GetPowerState() override;
  xyber::ActautorMode GetMode() override;

  void SetEffort(float cur) override;
  float GetEffort() override;
  void SetVelocity(float vel) override;
  float GetVelocity() override;
  void SetPosition(float pos) override;
  float GetPosition() override;

  void SetMitParam(xyber::MitParam param) override { mit_param_ = param; }
  void SetMitCmd(float pos, float vel, float toq, float kp, float kd) override;

 public:
  void SetDataFiled(uint8_t* send, uint8_t* recv) override;

 private:
  static float Clamp(float x, float lo, float hi);
  static int FloatToUint(float x, float x_min, float x_max, int bits);
  static float UintToFloat(int x_int, float x_min, float x_max, int bits);

  void UpdateFeedback();
  void WriteCurTorCmd(int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);

 private:
  uint8_t passage_ = 0;
  xyber::MitParam mit_param_ = ENCOS_EC_A4315_MIT_PARAM;  // overridden per model in ctor
  xyber::Motor_Msg* cmd_msg_ = nullptr;
  xyber::Motor_Msg* state_msg_ = nullptr;

  xyber::ActautorMode mode_ = xyber::MODE_MIT;
  xyber::ActautorState power_state_ = xyber::STATE_DISABLE;
  uint8_t error_ = 0;
  float position_ = 0.0f;  // rad
  float velocity_ = 0.0f;  // rad/s
  float current_ = 0.0f;   // phase current (A) from feedback frames; GetEffort() applies Kt
  float temperature_ = 0.0f;
};

}  // namespace xyber_encos
