/*
 * @Author: richie.li
 * @Date: 2024-10-21 14:10:06
 * @LastEditTime: 2026-10-08 11:00:00
 */

#pragma once

// cpp
#include <atomic>
#include <memory>
#include <string>

// projects
#include "common_type.h"

namespace xyber {

class XyberController {
 public:
  // Singletons
  XyberController(XyberController& other) = delete;
  void operator=(const XyberController&) = delete;
  virtual ~XyberController();

  /**
   * @brief: Get or create XyberController instance
   * @return {*}
   */
  static XyberController* GetInstance();

  /**
   * @brief Get the Version object
   *
   * @return std::string, "main.sub.patch"
   */
  std::string GetVersion();

  /**
   * @brief: Create a slave (dcu / encos / ...) via SlaveFactory, by cfg.type
   * @param {SlaveConfig&} slave type / name / ecat_id (+ params; each slave
   *        type reads the keys it owns)
   * @return {bool} true for success
   */
  bool CreateSlave(const SlaveConfig& cfg);

  /**
   * @brief: Create an actuator via ActuatorFactory and attach it on a slave.
   *         Binding fields (channel / can_id for dcu, passage / motor_id for
   *         encos, ...) travel inside info.params and are read by the
   *         actuator / slave type that owns them.
   * @param {string&} slave name
   * @param {ActuatorAttachInfo&} actuator type / name + binding params
   * @return {bool} true for success
   */
  bool AttachActuator(const std::string& slave_name, const ActuatorAttachInfo& info);

  /**
   * @brief: Create a dcu instance in XyberController
   * @param {string} dcu name
   * @param {uint8_t} ethercat_id
   * @return {bool} true for success
   * @deprecated: use CreateSlave({.type = "dcu", ...}) instead
   */
  bool CreateDcu(std::string name, uint8_t ethercat_id);

  /**
   * @brief: Create an actuator instance in XyberController and attch it on the dcu
   * @param {string} dcu name
   * @param {CtrlChannel} dcu ctrl channel, see CtrlChannel
   * @param {ActuatorType} actuator type in ActuatorType
   * @param {string} actuator name
   * @param {string} actuator can id in canfd bus
   * @return {bool} true for success
   * @deprecated: use AttachActuator(slave_name, info) instead
   */
  bool AttachActuator(std::string dcu_name, CtrlChannel ch, ActuatorType type,
                      std::string actuator_name, uint8_t can_id);

  /**
   * @brief: Create an Encos slave instance in XyberController
   * @param name slave name
   * @param ethercat_id EtherCAT station id on the bus
   * @return {bool} true for success
   * @deprecated: use CreateSlave({.type = "encos", ...}) instead
   */
  bool CreateEncosSlave(std::string name, uint8_t ethercat_id);

  /**
   * @brief: Create an Encos motor and attach it to an Encos slave PDO slot
   * @param slave_name Encos slave name
   * @param passage PDO slot in [1, 6]; 1~3 -> CAN1, 4~6 -> CAN2
   * @param type ENCOS_EC_A8116 / ENCOS_EC_A4315 (same protocol)
   * @param actuator_name actuator name used by upper layer
   * @param motor_id CAN id written into Motor_Msg.id
   * @return {bool} true for success
   * @deprecated: use AttachActuator(slave_name, info) instead
   */
  bool AttachEncosMotor(std::string slave_name, uint8_t passage, ActuatorType type,
                      std::string actuator_name, uint8_t motor_id);

  /**
   * @brief: make realtime priority for ecat io thread
   * @param {int} rt_priority for thread, in range [0, 99], 99 for highest, -1 for disable
   * @param {int} bind cpu core num, -1 for disable
   * @return {*}
   */
  bool SetRealtime(int rt_priority, int bind_cpu);

  /**
   * @brief: Start the ethercat communication
   * @param {string} ifname
   * @param {uint64_t} cycle ns
   * @param {bool} enable dc
   * @return {bool} true for success
   */
  bool Start(std::string ifname, uint64_t cycle_ns, bool enable_dc);

  /**
   * @brief: Stop the ethercat communication
   * @return {*}
   */
  void Stop();

 public:  // DCU API
  /**
   * @brief: Get dcu report imu data
   * @param {string&} dcu name
   * @return {DcuImu} imu data
   */
  DcuImu GetDcuImuData(const std::string& name);

  /**
   * @brief: Enable imu and apply offset
   * @param {string&} dcu name
   * @return {*}
   */
  void ApplyDcuImuOffset(const std::string& name);

 public:  // PowerFlowR Actuator API
  /**
   * @brief Enable all actuator
   *
   * @return true
   * @return false
   */
  bool EnableAllActuator();

  /**
   * @brief Enable an actuator
   *
   * @param  actuator name
   * @return true
   * @return false
   */
  bool EnableActuator(const std::string& name);

  /**
   * @brief Disable all actuator
   *
   * @return true
   * @return false
   */
  bool DisableAllActuator();

  /**
   * @brief Disable an actuator
   *
   * @param name
   * @return true
   * @return false
   */
  bool DisableActuator(const std::string& name);

  /**
   * @brief clear actuator fault (faulted motor ignores enable/MIT commands)
   *
   * @param name actuator name
   */
  void ClearError(const std::string& name);

  /**
   * @brief get tempure
   *
   * @param name
   * @return float unite Volt
   */
  float GetTempure(const std::string& name);

  /**
   * @brief Get the Power State
   *
   * @param name
   * @return ActautorState
   */
  ActautorState GetPowerState(const std::string& name);

  /**
   * @brief Get the Mode
   *
   * @param name
   * @return ActautorMode
   */
  ActautorMode GetMode(const std::string& name);

  /**
   * @brief Get the Torque
   *        Working for PowerFlowL and OminiPicker too.
   *        Encos: feedback phase current × model Kt (N·m).
   *
   * @param name
   * @return float (Nm)
   */
  float GetEffort(const std::string& name);

  /**
   * @brief Get the Velocity
   *        Working for PowerFlowL and OminiPicker too.
   *
   * @param name
   * @return float (rad/s)
   */
  float GetVelocity(const std::string& name);

  /**
   * @brief Get the Position
   *        Working for PowerFlowL and OminiPicker too.
   *
   * @param name
   * @return float (rad)
   */
  float GetPosition(const std::string& name);

  /**
   * @brief Set the Mit Param object
   *
   * @param name
   * @param param see MitParam
   */
  void SetMitParam(const std::string& name, MitParam param);

  /**
   * @brief Set the PD cmd using MIT mode
   *        For PowerFlowL and OminiPicker, only pos and effort work, keep others as 0.
   *
   * @param name
   * @param pos (rad)
   * @param vel (rad/s)
   * @param effort (Nm)
   * @param kp
   * @param kd
   */
  void SetMitCmd(const std::string& name, float pos, float vel, float effort, float kp, float kd);

 protected:
  XyberController();

 private:
  std::atomic_bool is_running_{false};
  static XyberController* instance_;
};

using XyberControllerPtr = std::shared_ptr<XyberController>;

}  // namespace xyber