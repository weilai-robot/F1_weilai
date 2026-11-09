// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.
#include "dcu_driver_module/ankle_transmission.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include "dcu_driver_module/ankle_kinematics.h"
#include "yaml-cpp/yaml.h"

namespace xyber_x1_infer::dcu_driver_module {
struct AnkleModel {
  ankle::Geometry left, right;
  ankle::Vec2 origin{};
  double step{}, node_tolerance{}, inverse_motor_tolerance{}, interpolation_tolerance{}, roundtrip_tolerance{}, epsilon{};
  std::size_t rows{}, columns{};
  std::vector<ankle::Vec2> table;
};

namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool Finite(const ankle::Vec2& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]);
}
double Maximum(const ankle::Vec2& value) {
  return std::max(std::abs(value[0]),std::abs(value[1]));
}
double Positive(const YAML::Node& node) {
  const auto value=node.as<double>();
  Require(std::isfinite(value) && value>0,"Invalid ankle numerical setting");
  return value;
}
ankle::Geometry ParseGeometry(const YAML::Node& node) {
  Require(node.IsSequence() && node.size()==2,"Expected long and short ankle branches");
  std::array<double,22> packed{};
  for (std::size_t k=0;k<2;++k) {
    const auto motor=node[k]["motor_center"].as<std::vector<double>>();
    const auto foot=node[k]["foot_attachment"].as<std::vector<double>>();
    Require(motor.size()==3 && foot.size()==3,"Invalid ankle point coordinates");
    auto* b=packed.data()+11*k;
    std::copy(motor.begin(),motor.end(),b);
    std::copy(foot.begin(),foot.end(),b+3);
    b[6]=node[k]["radius"].as<double>();
    b[7]=node[k]["rod_length"].as<double>();
    b[8]=node[k]["zero_angle"].as<double>();
    b[9]=node[k]["assembly_sign"].as<double>();
    b[10]=node[k]["motor_direction"].as<double>();
  }
  return ankle::unpack(packed.data(),packed.size());
}
std::shared_ptr<const AnkleModel> LoadModel(const std::string& path) {
  const auto config_path=std::filesystem::absolute(path).lexically_normal();
  static std::mutex cache_mutex;
  static std::unordered_map<std::string,std::weak_ptr<const AnkleModel>> cache;
  std::lock_guard<std::mutex> lock(cache_mutex);
  if (auto shared=cache[config_path.string()].lock()) return shared;

  const auto config=YAML::LoadFile(config_path.string());
  Require(config["schema_version"].as<int>()==1,"Unsupported F1 ankle configuration schema");
  auto model=std::make_shared<AnkleModel>();
  model->left=ParseGeometry(config["left_geometry"]);
  model->right=ParseGeometry(config["right_geometry"]);
  const auto info=config["table"];
  model->rows=info["rows"].as<std::size_t>();
  model->columns=info["columns"].as<std::size_t>();
  Require(model->rows>=2 && model->columns>=2 &&
          model->rows<=std::numeric_limits<std::size_t>::max()/model->columns,
          "Invalid ankle table dimensions");
  const auto origin=info["motor_origin_rad"].as<std::vector<double>>();
  Require(origin.size()==2,"Expected two ankle table origins");
  model->origin={origin[0],origin[1]};
  Require(Finite(model->origin),"Non-finite ankle table origin");
  model->step=Positive(info["motor_step_rad"]);
  model->node_tolerance=Positive(info["node_length_tolerance_m"]);
  model->inverse_motor_tolerance=Positive(info["inverse_motor_tolerance_rad"]);
  model->interpolation_tolerance=Positive(info["interpolated_length_tolerance_m"]);
  model->roundtrip_tolerance=Positive(info["joint_roundtrip_tolerance_rad"]);
  model->epsilon=Positive(info["coordinate_epsilon"]);
  const auto count=model->rows*model->columns;
  model->table.resize(count);
  std::vector<bool> loaded(count,false);
  const auto table_path=config_path.parent_path()/info["file"].as<std::string>();
  const auto table=YAML::LoadFile(table_path.string());
  Require(table.IsMap() && table.size()==count,"Incomplete ankle FK table");
  for (const auto& entry:table) {
    const auto key=entry.first.as<std::string>();
    std::size_t i=0,j=0;
    int consumed=0;
    Require(std::sscanf(key.c_str(),"qm5qm6_%zu_%zu%n",&i,&j,&consumed)==2 &&
            consumed==static_cast<int>(key.size()) && i>=1 && i<=model->rows &&
            j>=1 && j<=model->columns,"Invalid ankle FK table key");
    const auto index=(i-1)*model->columns+j-1;
    Require(!loaded[index],"Duplicate ankle FK table key");
    Require(entry.second.IsMap() && entry.second.size()==2,"Invalid ankle FK table value");
    const ankle::Vec2 q{entry.second["q5"].as<double>(),entry.second["q6"].as<double>()};
    Require(Finite(q),"Non-finite ankle FK table value");
    const ankle::Vec2 m{model->origin[0]+(i-1)*model->step,
                        model->origin[1]+(j-1)*model->step};
    Require(Maximum(ankle::constraint_residual(model->right,q,m))<=model->node_tolerance,
            "Ankle FK table does not match configured physical geometry");
    const ankle::Vec2 left_q{q[0],-q[1]},left_m{-m[0],-m[1]};
    Require(Maximum(ankle::constraint_residual(model->left,left_q,left_m))<=model->node_tolerance,
            "Left/right mirror geometry does not match shared ankle FK table");
    const auto right_back=ankle::inverse_position(model->right,q);
    const auto left_back=ankle::inverse_position(model->left,left_q);
    Require(Maximum({right_back[0]-m[0],right_back[1]-m[1]})<model->inverse_motor_tolerance &&
            Maximum({left_back[0]-left_m[0],left_back[1]-left_m[1]})<model->inverse_motor_tolerance,
            "Ankle FK table node does not match current IK assembly branch");
    model->table[index]=q;
    loaded[index]=true;
  }
  Require(std::all_of(loaded.begin(),loaded.end(),[](bool v){return v;}),"Missing ankle FK node");
  cache[config_path.string()]=model;
  return model;
}

ankle::Vec2 Lookup(const AnkleModel& model,const ankle::Vec2& raw_motor,bool left) {
  Require(Finite(raw_motor),"Non-finite ankle motor feedback");
  const ankle::Vec2 m=left?ankle::Vec2{-raw_motor[0],-raw_motor[1]}:raw_motor;
  ankle::Vec2 u{(m[0]-model.origin[0])/model.step,(m[1]-model.origin[1])/model.step};
  const std::array<std::size_t,2> shape{model.rows,model.columns};
  std::array<std::size_t,2> base{};
  ankle::Vec2 fraction{};
  for (std::size_t k=0;k<2;++k) {
    const double last=static_cast<double>(shape[k]-1);
    Require(std::isfinite(u[k]) && u[k]>=-model.epsilon && u[k]<=last+model.epsilon,
            "Ankle motor angle outside FK table node range");
    u[k]=std::clamp(u[k],0.0,last); // Only roundoff at an already-checked endpoint.
    base[k]=std::min(static_cast<std::size_t>(std::floor(u[k])),shape[k]-2);
    fraction[k]=u[k]-base[k];
  }
  const auto i=base[0],j=base[1];
  const std::array<std::size_t,4> indices{i*model.columns+j,(i+1)*model.columns+j,
                                        i*model.columns+j+1,(i+1)*model.columns+j+1};
  const auto a=fraction[0],b=fraction[1];
  const std::array<double,4> weights{(1-a)*(1-b),a*(1-b),(1-a)*b,a*b};
  ankle::Vec2 q{};
  for (std::size_t k=0;k<4;++k) {
    q[0]+=weights[k]*model.table[indices[k]][0];
    q[1]+=weights[k]*model.table[indices[k]][1];
  }
  Require(Finite(q) && Maximum(ankle::constraint_residual(model.right,q,m))<=model.interpolation_tolerance,
          "Ankle interpolated FK violates the rod constraints");
  if (left) q[1]=-q[1];
  return q;
}
ankle::Vec2 MotorPosition(const ActuatorHandle& m5,const ActuatorHandle& m6) {
  return {m5.handle->state.position*m5.direction,m6.handle->state.position*m6.direction};
}
void CheckHandle(const ActuatorHandle& motor) {
  Require(motor.handle && (motor.direction==1 || motor.direction==-1),"Invalid ankle actuator binding/direction");
}
}  // namespace

AnkleParallelTransmission::AnkleParallelTransmission(
    std::string name,std::string path,bool is_left,ActuatorHandle motor5,ActuatorHandle motor6,
    JointHandle joint_pitch,JointHandle joint_roll)
    : Transmission(std::move(name)),model_(LoadModel(path)),is_left_(is_left),
      motor5_(motor5),motor6_(motor6),joint_pitch_(joint_pitch),joint_roll_(joint_roll) {
  CheckHandle(motor5_);
  CheckHandle(motor6_);
  Require(motor5_.handle!=motor6_.handle && joint_pitch_.handle && joint_roll_.handle &&
          joint_pitch_.handle!=joint_roll_.handle,"Invalid ankle joint/actuator binding");
}

void AnkleParallelTransmission::TransformActuatorToJoint() {
  const auto m=MotorPosition(motor5_,motor6_);
  const auto q=Lookup(*model_,m,is_left_);
  const auto& geometry=is_left_?model_->left:model_->right;
  const auto j=ankle::jacobian(geometry,q,m);
  const ankle::Vec2 motor_velocity{motor5_.handle->state.velocity*motor5_.direction,
                                   motor6_.handle->state.velocity*motor6_.direction};
  // Hardware feedback effort units are intentionally unchanged in this patch.
  const ankle::Vec2 motor_effort{motor5_.handle->state.effort*motor5_.direction,
                                 motor6_.handle->state.effort*motor6_.direction};
  Require(Finite(motor_velocity) && Finite(motor_effort),"Non-finite ankle velocity/effort feedback");
  const auto velocity=ankle::motor_to_joint_velocity(j,motor_velocity);
  const auto effort=ankle::motor_to_joint_torque(j,motor_effort);
  Require(Finite(velocity) && Finite(effort),"Non-finite ankle joint feedback");
  // Commit the complete pair after all calculations succeed.
  joint_pitch_.handle->state.position=q[0];
  joint_roll_.handle->state.position=q[1];
  joint_pitch_.handle->state.velocity=velocity[0];
  joint_roll_.handle->state.velocity=velocity[1];
  joint_pitch_.handle->state.effort=effort[0];
  joint_roll_.handle->state.effort=effort[1];
}

void AnkleParallelTransmission::TransformJointToActuator() {
  const auto& geometry=is_left_?model_->left:model_->right;
  const ankle::Vec2 desired_q{joint_pitch_.handle->cmd.position,joint_roll_.handle->cmd.position};
  const ankle::Vec2 desired_velocity{joint_pitch_.handle->cmd.velocity,joint_roll_.handle->cmd.velocity};
  const ankle::Vec2 desired_effort{joint_pitch_.handle->cmd.effort,joint_roll_.handle->cmd.effort};
  const ankle::Vec2 kp{joint_pitch_.handle->cmd.kp,joint_roll_.handle->cmd.kp};
  const ankle::Vec2 kd{joint_pitch_.handle->cmd.kd,joint_roll_.handle->cmd.kd};
  Require(Finite(desired_q) && Finite(desired_velocity) && Finite(desired_effort) && Finite(kp) && Finite(kd),
          "Non-finite ankle command");
  Require(kp[0]>=0 && kp[1]>=0 && kd[0]>=0 && kd[1]>=0,"Negative ankle MIT gain");
  const auto desired_m=ankle::inverse_position(geometry,desired_q);
  const auto recovered=Lookup(*model_,desired_m,is_left_);
  Require(Maximum({recovered[0]-desired_q[0],recovered[1]-desired_q[1]})<=model_->roundtrip_tolerance,
          "Ankle target is outside supported FK/IK assembly branch");
  // Use the same measured motor snapshot and its FK pose for the Jacobian.
  const auto measured_m=MotorPosition(motor5_,motor6_);
  const auto measured_q=Lookup(*model_,measured_m,is_left_);
  const auto j=ankle::jacobian(geometry,measured_q,measured_m);
  const auto velocity=ankle::joint_to_motor_velocity(j,desired_velocity);
  const auto effort=ankle::joint_to_motor_torque(j,desired_effort);
  Require(Finite(desired_m) && Finite(velocity) && Finite(effort),"Non-finite ankle motor command");
  const std::array<ActuatorHandle,2> motors{motor5_,motor6_};
  for (std::size_t k=0;k<2;++k) {
    auto& cmd=motors[k].handle->cmd;
    cmd.position=desired_m[k]*motors[k].direction;
    cmd.velocity=velocity[k]*motors[k].direction;
    cmd.effort=effort[k]*motors[k].direction;
    // Retain the existing gain pass-through convention; RL sets both to zero.
    cmd.kp=kp[k];
    cmd.kd=kd[k];
  }
}

LeftAnkleParallelTransmission::LeftAnkleParallelTransmission(
    std::string name,std::string path,ActuatorHandle actr_left,ActuatorHandle actr_right,
    JointHandle joint_pitch,JointHandle joint_roll)
    : AnkleParallelTransmission(std::move(name),std::move(path),true,actr_right,actr_left,joint_pitch,joint_roll) {}

RightAnkleParallelTransmission::RightAnkleParallelTransmission(
    std::string name,std::string path,ActuatorHandle actr_left,ActuatorHandle actr_right,
    JointHandle joint_pitch,JointHandle joint_roll)
    : AnkleParallelTransmission(std::move(name),std::move(path),false,actr_left,actr_right,joint_pitch,joint_roll) {}
}  // namespace xyber_x1_infer::dcu_driver_module
