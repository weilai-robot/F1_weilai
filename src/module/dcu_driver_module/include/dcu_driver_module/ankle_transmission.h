// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#pragma once
#include <memory>
#include "dcu_driver_module/transmission.h"

namespace xyber_x1_infer::dcu_driver_module {
struct AnkleModel;

// New F1 geometry only; existing derived-class registration signatures remain.
class AnkleParallelTransmission : public Transmission {
 public:
  void TransformActuatorToJoint() override;
  void TransformJointToActuator() override;
 protected:
  AnkleParallelTransmission(std::string name, std::string path, bool is_left,
                            ActuatorHandle motor5, ActuatorHandle motor6,
                            JointHandle joint_pitch, JointHandle joint_roll);
 private:
  std::shared_ptr<const AnkleModel> model_;
  bool is_left_;
  ActuatorHandle motor5_, motor6_;
  JointHandle joint_pitch_, joint_roll_;
};

class LeftAnkleParallelTransmission : public AnkleParallelTransmission {
 public:
  LeftAnkleParallelTransmission(std::string name, std::string param_path,
                                ActuatorHandle actr_left, ActuatorHandle actr_right,
                                JointHandle joint_pitch, JointHandle joint_roll);
};

class RightAnkleParallelTransmission : public AnkleParallelTransmission {
 public:
  RightAnkleParallelTransmission(std::string name, std::string param_path,
                                 ActuatorHandle actr_left, ActuatorHandle actr_right,
                                 JointHandle joint_pitch, JointHandle joint_roll);
};
}  // namespace xyber_x1_infer::dcu_driver_module
