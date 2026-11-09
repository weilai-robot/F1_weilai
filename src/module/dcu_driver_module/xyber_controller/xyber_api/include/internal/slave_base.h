/*
 * @Author: zhangmy
 * @Date: 2026-10-08 10:30:00
 * @LastEditors: zhangmy
 * @LastEditTime: 2026-10-08 10:30:00
 */

#pragma once

// cpp
#include <string>

// projects
#include "common_type.h"
#include "internal/actuator_base.h"
#include "internal/ethercat_node.h"

namespace xyber {

/**
 * @description: Common base of all EtherCAT slave drivers (Dcu, EncosSlave, ...).
 *               SlaveFactory returns this type so callers (XyberController,
 *               DcuDriverModule) never branch on the concrete slave class.
 *               Subclasses keep their own PDO layout and override the
 *               per-actuator API below.
 */
class SlaveBase : public ethercat_manager::EthercatNode {
 public:
  SlaveBase(int32_t ecat_id, std::string name) : EthercatNode(ecat_id), name_(name) {}
  virtual ~SlaveBase() {}

  /**
   * @brief: Attach an actuator (already created by ActuatorFactory) on this
   *         slave. The subclass binds it to its own PDO layout and validates
   *         slot / channel usage. On failure the caller keeps ownership of actr.
   * @param {Actuator*} actuator instance
   * @param {ActuatorAttachInfo&} creation + binding params
   * @return {bool} true for success
   */
  virtual bool RegisterActuator(Actuator* actr, const ActuatorAttachInfo& info) = 0;
  virtual Actuator* GetActautor(const std::string& name) = 0;
  virtual std::string GetName() override { return name_; }

 public:  // Actuator Stuff
  virtual bool EnableAllActuator() = 0;
  virtual bool EnableActuator(const std::string& name) = 0;
  virtual bool DisableAllActuator() = 0;
  virtual bool DisableActuator(const std::string& name) = 0;
  virtual void ClearError(const std::string& name) = 0;
  virtual void SetHomingPosition(const std::string& name) = 0;
  virtual void SaveConfig(const std::string& name) = 0;
  virtual float GetTempure(const std::string& name) = 0;
  virtual std::string GetErrorString(const std::string& name) = 0;
  virtual ActautorState GetPowerState(const std::string& name) = 0;

  virtual bool SetMode(const std::string& name, ActautorMode mode) = 0;
  virtual ActautorMode GetMode(const std::string& name) = 0;

  virtual void SetEffort(const std::string& name, float cur) = 0;
  virtual float GetEffort(const std::string& name) = 0;
  virtual void SetVelocity(const std::string& name, float vel) = 0;
  virtual float GetVelocity(const std::string& name) = 0;
  virtual void SetPosition(const std::string& name, float pos) = 0;
  virtual float GetPosition(const std::string& name) = 0;

  virtual void SetMitParam(const std::string& name, MitParam param) = 0;
  virtual void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp,
                         float kd) = 0;

 public:  // Optional capabilities, safe no-op defaults for slaves without them
  virtual bool HasImu() const { return false; }
  virtual DcuImu GetImuData() { return DcuImu(); }
  virtual void ApplyImuOffset() {}
  virtual void SetCanIde(uint8_t can_ide) { (void)can_ide; }

 protected:
  std::string name_;
};

}  // namespace xyber
