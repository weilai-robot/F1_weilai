/*
 * @Author: zhangmy
 * @Date: 2026-08-13 16:39:00
 * @LastEditTime: 2026-08-14 09:40:00
 */

#include "internal/encos_motor.h"

#include <cmath>
#include <cstring>

#include "internal/common_utils.h"

namespace xyber_encos {

namespace {

constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;

// Feedback current range for MIT response frame 1 (motor_control.c I_MIN/I_MAX)
constexpr float kEncosIMin = -30.0f;
constexpr float kEncosIMax = 30.0f;

}  // namespace

EncosMotor::EncosMotor(xyber::ActuatorType type, std::string name, uint8_t motor_id, uint8_t passage)
    // CtrlChannel is unused for Encos; keep a placeholder for Actuator base ctor.
    // EC-A8112-P1-18H and EC-A4314-P2-36 share the same control protocol.
    : Actuator(name, type, motor_id, xyber::CtrlChannel::CTRL_CH1), passage_(passage) {}

void EncosMotor::SetDataFiled(uint8_t* send, uint8_t* recv) {
  // Bound to a whole Motor_Msg slot (not DCU-style 8B * (id-1) offset).
  cmd_msg_ = reinterpret_cast<xyber::Motor_Msg*>(send);
  state_msg_ = reinterpret_cast<xyber::Motor_Msg*>(recv);
  send_buf_ = send;
  recv_buf_ = recv;

  if (cmd_msg_) {
    memset(cmd_msg_, 0, sizeof(xyber::Motor_Msg));
    cmd_msg_->id = id_;
    cmd_msg_->rtr = 0;
    cmd_msg_->dlc = 8;
  }
}

float EncosMotor::Clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int EncosMotor::FloatToUint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  return static_cast<int>((x - x_min) * (static_cast<float>((1 << bits) - 1)) / span);
}

float EncosMotor::UintToFloat(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  return (static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1)) + x_min;
}

void EncosMotor::UpdateFeedback() {
  if (!state_msg_ || state_msg_->dlc == 0) return;

  // Config / broadcast reply on 0x7FF
  if (state_msg_->id == 0x7FF) {
    if (state_msg_->dlc >= 4 && state_msg_->data[2] == 0x01) {
      // motor_fbd: 0x03 = zero set ok
      if (state_msg_->data[3] == 0x03) {
        // keep current power state
      }
    }
    return;
  }

  // Ask / response mode (comm_mode = 0): ack_status in data[0] high 3 bits
  const uint8_t ack_status = state_msg_->data[0] >> 5;
  error_ = state_msg_->data[0] & 0x1F;

  if (ack_status == 1) {
    // Response frame 1 — used by force-position hybrid (MIT)
    const int pos_int = (state_msg_->data[1] << 8) | state_msg_->data[2];
    const int spd_int = (state_msg_->data[3] << 4) | ((state_msg_->data[4] & 0xF0) >> 4);
    const int cur_int = ((state_msg_->data[4] & 0x0F) << 8) | state_msg_->data[5];

    position_ = UintToFloat(pos_int, mit_param_.pos_min, mit_param_.pos_max, 16);
    velocity_ = UintToFloat(spd_int, mit_param_.vel_min, mit_param_.vel_max, 12);
    effort_ = UintToFloat(cur_int, kEncosIMin, kEncosIMax, 12);
    if (state_msg_->dlc >= 7) {
      temperature_ = (static_cast<float>(state_msg_->data[6]) - 50.0f) / 2.0f;
    }
  } else if (ack_status == 2 && state_msg_->dlc >= 8) {
    // Response frame 2 — angle float (deg) + current int
    union {
      float f;
      uint8_t b[4];
    } conv;
    conv.b[0] = state_msg_->data[4];
    conv.b[1] = state_msg_->data[3];
    conv.b[2] = state_msg_->data[2];
    conv.b[3] = state_msg_->data[1];
    position_ = conv.f * kDegToRad;
    const int16_t cur_int =
        static_cast<int16_t>((state_msg_->data[5] << 8) | state_msg_->data[6]);
    effort_ = cur_int / 100.0f;
    temperature_ = (static_cast<float>(state_msg_->data[7]) - 50.0f) / 2.0f;
  } else if (ack_status == 3 && state_msg_->dlc >= 8) {
    // Response frame 3 — speed float + current int
    union {
      float f;
      uint8_t b[4];
    } conv;
    conv.b[0] = state_msg_->data[4];
    conv.b[1] = state_msg_->data[3];
    conv.b[2] = state_msg_->data[2];
    conv.b[3] = state_msg_->data[1];
    velocity_ = conv.f;  // protocol uses float speed; treat as rad/s if already converted upstream
    const int16_t cur_int =
        static_cast<int16_t>((state_msg_->data[5] << 8) | state_msg_->data[6]);
    effort_ = cur_int / 100.0f;
    temperature_ = (static_cast<float>(state_msg_->data[7]) - 50.0f) / 2.0f;
  }
}

void EncosMotor::WriteCurTorCmd(int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status) {
  if (!cmd_msg_) return;
  if (ack_status > 3 || ctrl_status > 7) return;

  if (ctrl_status != 0) {
    if (cur_tor > 3000) cur_tor = 3000;
    if (cur_tor < -3000) cur_tor = -3000;
  } else {
    if (cur_tor > 2000) cur_tor = 2000;
    if (cur_tor < -2000) cur_tor = -2000;
  }

  // set_motor_cur_tor: standard frame, needs slave can_ide=0
  cmd_msg_->id = id_;
  cmd_msg_->rtr = 0;
  cmd_msg_->dlc = 3;
  cmd_msg_->data[0] = static_cast<uint8_t>(0x60 | (ctrl_status << 2) | ack_status);
  cmd_msg_->data[1] = static_cast<uint8_t>(cur_tor >> 8);
  cmd_msg_->data[2] = static_cast<uint8_t>(cur_tor & 0xff);
  memset(cmd_msg_->data + 3, 0, 5);
}

void EncosMotor::RequestState(xyber::ActautorState state) {
  if (!cmd_msg_) return;

  if (state == xyber::STATE_ENABLE) {
    // Encos has no PowerFlow-style enable bit; enter MIT hold (zero cmd) and mark enabled.
    // Requires EncosSlave can_ide=1 (extended frame).
    SetMitCmd(position_, 0.0f, 0.0f, 0.0f, 0.0f);
    power_state_ = xyber::STATE_ENABLE;
    return;
  }

  if (state == xyber::STATE_DISABLE) {
    // Full brake (ctrl_status=2). Requires EncosSlave can_ide=0 for this frame type.
    // Prefer zero-torque MIT hold when board stays in extended-frame MIT mode.
    SetMitCmd(position_, 0.0f, 0.0f, 0.0f, 0.0f);
    power_state_ = xyber::STATE_DISABLE;
  }
}

void EncosMotor::ClearError() {
  // Protocol has no dedicated clear-error frame; keep last cmd and clear cached flag.
  error_ = 0;
}

void EncosMotor::SetHomingPosition() {
  if (!cmd_msg_) return;
  // Motor_Setzero: id=0x7FF, dlc=4, data = motor_id_hi, motor_id_lo, 0x00, 0x03
  cmd_msg_->id = 0x7FF;
  cmd_msg_->rtr = 0;
  cmd_msg_->dlc = 4;
  cmd_msg_->data[0] = static_cast<uint8_t>(id_ >> 8);  // id_ is uint8_t, high byte 0
  cmd_msg_->data[1] = id_;
  cmd_msg_->data[2] = 0x00;
  cmd_msg_->data[3] = 0x03;
  memset(cmd_msg_->data + 4, 0, 4);
}

void EncosMotor::SaveConfig() {
  // Not used on Encos path currently.
}

void EncosMotor::SetMode(xyber::ActautorMode mode) {
  mode_ = mode;
  // Runtime switching of ask/auto is MotorSetting(cmd) on CAN1/0x7FF; not per-slot here.
}

std::string EncosMotor::GetErrorString() {
  UpdateFeedback();
  if (error_ == 0) return "None";
  return "EncosError_" + std::to_string(error_);
}

float EncosMotor::GetTempure() {
  UpdateFeedback();
  return temperature_;
}

xyber::ActautorState EncosMotor::GetPowerState() {
  UpdateFeedback();
  return power_state_;
}

xyber::ActautorMode EncosMotor::GetMode() { return mode_; }

void EncosMotor::SetEffort(float cur) {
  // Torque mode: cur_tor = Nm * 100 (protocol 100:1), ctrl_status=1, ack=1
  mode_ = xyber::MODE_CURRENT;
  const int16_t cur_tor = static_cast<int16_t>(Clamp(cur, -30.0f, 30.0f) * 100.0f);
  WriteCurTorCmd(cur_tor, /*ctrl_status=*/1, /*ack_status=*/1);
}

float EncosMotor::GetEffort() {
  UpdateFeedback();
  return effort_;
}

void EncosMotor::SetVelocity(float vel) {
  // set_motor_speed: spd is float rpm in demo; convert rad/s -> rpm
  if (!cmd_msg_) return;
  mode_ = xyber::MODE_VELOCITY;

  const float rpm = vel * 60.0f / (2.0f * static_cast<float>(M_PI));
  union {
    float f;
    uint8_t b[4];
  } conv;
  conv.f = rpm;

  constexpr uint16_t kCurLimit = 3000;  // 300.0A @ 10:1
  constexpr uint8_t kAck = 1;

  cmd_msg_->id = id_;
  cmd_msg_->rtr = 0;
  cmd_msg_->dlc = 7;
  cmd_msg_->data[0] = static_cast<uint8_t>(0x40 | kAck);
  cmd_msg_->data[1] = conv.b[3];
  cmd_msg_->data[2] = conv.b[2];
  cmd_msg_->data[3] = conv.b[1];
  cmd_msg_->data[4] = conv.b[0];
  cmd_msg_->data[5] = static_cast<uint8_t>(kCurLimit >> 8);
  cmd_msg_->data[6] = static_cast<uint8_t>(kCurLimit & 0xff);
  cmd_msg_->data[7] = 0;
}

float EncosMotor::GetVelocity() {
  UpdateFeedback();
  return velocity_;
}

void EncosMotor::SetPosition(float pos) {
  // set_motor_position: pos in degrees
  if (!cmd_msg_) return;
  mode_ = xyber::MODE_POSITION;

  union {
    float f;
    uint8_t b[4];
  } conv;
  conv.f = pos * kRadToDeg;

  constexpr uint16_t kSpd = 18000;  // max
  constexpr uint16_t kCur = 3000;
  constexpr uint8_t kAck = 1;

  cmd_msg_->id = id_;
  cmd_msg_->rtr = 0;
  cmd_msg_->dlc = 8;
  cmd_msg_->data[0] = static_cast<uint8_t>(0x20 | (conv.b[3] >> 3));
  cmd_msg_->data[1] = static_cast<uint8_t>((conv.b[3] << 5) | (conv.b[2] >> 3));
  cmd_msg_->data[2] = static_cast<uint8_t>((conv.b[2] << 5) | (conv.b[1] >> 3));
  cmd_msg_->data[3] = static_cast<uint8_t>((conv.b[1] << 5) | (conv.b[0] >> 3));
  cmd_msg_->data[4] = static_cast<uint8_t>((conv.b[0] << 5) | (kSpd >> 10));
  cmd_msg_->data[5] = static_cast<uint8_t>((kSpd & 0x3FC) >> 2);
  cmd_msg_->data[6] = static_cast<uint8_t>(((kSpd & 0x03) << 6) | (kCur >> 6));
  cmd_msg_->data[7] = static_cast<uint8_t>(((kCur & 0x3F) << 2) | kAck);
}

float EncosMotor::GetPosition() {
  UpdateFeedback();
  return position_;
}

void EncosMotor::SetMitCmd(float pos, float vel, float toq, float kp, float kd) {
  if (!cmd_msg_) return;

  // send_motor_ctrl_cmd — force-position hybrid, extended frame (can_ide=1 on slave)
  mode_ = xyber::MODE_MIT;

  kp = Clamp(kp, mit_param_.kp_min, mit_param_.kp_max);
  kd = Clamp(kd, mit_param_.kd_min, mit_param_.kd_max);
  pos = Clamp(pos, mit_param_.pos_min, mit_param_.pos_max);
  vel = Clamp(vel, mit_param_.vel_min, mit_param_.vel_max);
  toq = Clamp(toq, mit_param_.toq_min, mit_param_.toq_max);

  const int kp_int = FloatToUint(kp, mit_param_.kp_min, mit_param_.kp_max, 12);
  const int kd_int = FloatToUint(kd, mit_param_.kd_min, mit_param_.kd_max, 9);
  const int pos_int = FloatToUint(pos, mit_param_.pos_min, mit_param_.pos_max, 16);
  const int spd_int = FloatToUint(vel, mit_param_.vel_min, mit_param_.vel_max, 12);
  const int tor_int = FloatToUint(toq, mit_param_.toq_min, mit_param_.toq_max, 12);

  cmd_msg_->id = id_;
  cmd_msg_->rtr = 0;
  cmd_msg_->dlc = 8;
  cmd_msg_->data[0] = static_cast<uint8_t>(0x00 | (kp_int >> 7));
  cmd_msg_->data[1] =
      static_cast<uint8_t>(((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8));
  cmd_msg_->data[2] = static_cast<uint8_t>(kd_int & 0xFF);
  cmd_msg_->data[3] = static_cast<uint8_t>(pos_int >> 8);
  cmd_msg_->data[4] = static_cast<uint8_t>(pos_int & 0xFF);
  cmd_msg_->data[5] = static_cast<uint8_t>(spd_int >> 4);
  cmd_msg_->data[6] = static_cast<uint8_t>(((spd_int & 0x0F) << 4) | (tor_int >> 8));
  cmd_msg_->data[7] = static_cast<uint8_t>(tor_int & 0xff);
}

}  // namespace xyber_encos
