// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// cpp
#include <string>
#include <vector>

// projects
#include "dcu_driver_module/xyber_controller/xyber_api/include/common_type.h"
#include "util/log_util.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

struct EthercatConfig {
  std::string ifname;
  bool enable_dc;
  int bind_cpu;
  int rt_priority;
  uint64_t cycle_time_ns;
};

template <>
struct convert<EthercatConfig> {
  static bool decode(const Node& node, EthercatConfig& rhs) {
    try {
      rhs.ifname = node["ifname"].as<std::string>();
      rhs.enable_dc = node["enable_dc"].as<bool>();
      rhs.bind_cpu = node["bind_cpu"].as<int>();
      rhs.rt_priority = node["rt_priority"].as<int>();
      rhs.cycle_time_ns = node["cycle_time_ns"].as<uint64_t>();
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse EtherConfig failed, {}", e.what());
      return false;
    }
  }
};

// Keys consumed by this parser itself (not forwarded into params). Every other
// scalar key on a slave / actuator entry is a free-form param owned by the
// slave / actuator type that reads it.
static const char* const kSlaveReservedKeys[] = {
    "name",    "type",    "enable",     "imu_enable", "ecat_id",
    "actuators",          // canonical actuator list (list, not a param)
    "channel_1",          // legacy dcu lists (expanded into actuators)
    "channel_2",
    "channel_3",
    "motors",             // legacy encos list (expanded into actuators)
};
static const char* const kActuatorReservedKeys[] = {"name", "type"};

inline bool IsReservedKey(const Node& node, const char* const reserved[], size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (node.Scalar() == reserved[i]) return true;
  }
  return false;
}

// Copy scalar keys of an entry into params, skipping the reserved keys. Only
// name/type are mandatory; anything else a slave type understands goes here.
// Non-scalar values (maps / lists) are rejected so typos surface at startup.
inline bool CollectParams(const Node& entry, xyber::ParamMap& params,
                          const char* const reserved[], size_t reserved_count,
                          const std::string& who) {
  auto lgr = aimrt::common::util::SimpleLogger();
  for (const auto& kv : entry) {
    const std::string key = kv.first.Scalar();
    if (IsReservedKey(kv.first, reserved, reserved_count)) continue;
    if (!kv.second.IsScalar()) {
      AIMRT_HL_ERROR(lgr, "{}: param \"{}\" is not a scalar, unsupported.", who, key);
      return false;
    }
    params[key] = kv.second.Scalar();
  }
  return true;
}

struct DcuConfig {
  std::string name;
  std::string type = "dcu";  // "dcu" | "encos" | ...; default "dcu" for backward compatible YAML
  bool enable = false;
  bool imu_enable = false;
  uint32_t ecat_id = 0;
  xyber::ParamMap params;  // free-form slave params (e.g. encos can_ide), read by the slave creator

  struct Actuator {
    std::string name;
    std::string type;
    xyber::ParamMap params;  // free-form binding params (channel / can_id / passage / motor_id / ...)
  };
  std::vector<Actuator> actuators;  // normalized: channel_1/2/3 and motors all land here
};
using DcuNetworkConfig = std::vector<DcuConfig>;

template <>
struct convert<DcuNetworkConfig> {
  static bool decode(const Node& node, DcuNetworkConfig& rhs) {
    try {
      for (auto& dcu_node : node) {
        DcuConfig dcu_cfg;
        dcu_cfg.name = dcu_node["name"].as<std::string>();
        dcu_cfg.ecat_id = dcu_node["ecat_id"].as<uint32_t>();
        dcu_cfg.enable = dcu_node["enable"].as<bool>();
        dcu_cfg.imu_enable = dcu_node["imu_enable"].as<bool>();
        dcu_cfg.type = dcu_node["type"] ? dcu_node["type"].as<std::string>() : "dcu";
        if (!CollectParams(dcu_node, dcu_cfg.params, kSlaveReservedKeys,
                           sizeof(kSlaveReservedKeys) / sizeof(kSlaveReservedKeys[0]),
                           dcu_cfg.name)) {
          return false;
        }

        // Normalized actuator list. "actuators" is the canonical form; legacy
        // "channel_1/2/3" and "motors" lists are expanded into it, so new
        // slave types only ever need "actuators".
        const bool has_new = dcu_node["actuators"].IsDefined();
        if (has_new) {
          for (auto& actr_node : dcu_node["actuators"]) {
            DcuConfig::Actuator actr_cfg;
            actr_cfg.name = actr_node["name"].as<std::string>();
            actr_cfg.type = actr_node["type"].as<std::string>();
            if (!CollectParams(actr_node, actr_cfg.params, kActuatorReservedKeys,
                               sizeof(kActuatorReservedKeys) / sizeof(kActuatorReservedKeys[0]),
                               actr_cfg.name)) {
              return false;
            }
            dcu_cfg.actuators.push_back(actr_cfg);
          }
        }

        // Legacy DCU: channel_1 / channel_2 / channel_3 -> inject "channel" param.
        std::vector<std::string> ch_names{"channel_1", "channel_2", "channel_3"};
        auto lgr = aimrt::common::util::SimpleLogger();
        for (size_t i = 0; i < ch_names.size(); i++) {
          if (!dcu_node[ch_names[i]].IsDefined()) continue;
          if (has_new) {
            AIMRT_HL_WARN(lgr, "Slave {}: both \"actuators\" and \"{}\" present, \"{}\" ignored.",
                          dcu_cfg.name, ch_names[i], ch_names[i]);
            continue;
          }
          for (auto& actr_node : dcu_node[ch_names[i]]) {
            DcuConfig::Actuator actr_cfg;
            actr_cfg.name = actr_node["name"].as<std::string>();
            actr_cfg.type = actr_node["type"].as<std::string>();
            if (!CollectParams(actr_node, actr_cfg.params, kActuatorReservedKeys,
                               sizeof(kActuatorReservedKeys) / sizeof(kActuatorReservedKeys[0]),
                               actr_cfg.name)) {
              return false;
            }
            actr_cfg.params["channel"] = std::to_string(i + 1);
            dcu_cfg.actuators.push_back(actr_cfg);
          }
        }

        // Legacy Encos: motors[{name, type, passage, motor_id}] -> params as-is.
        if (dcu_node["motors"].IsDefined()) {
          if (has_new) {
            AIMRT_HL_WARN(lgr, "Slave {}: both \"actuators\" and \"motors\" present, \"motors\" ignored.",
                          dcu_cfg.name);
          } else {
            for (auto& actr_node : dcu_node["motors"]) {
              DcuConfig::Actuator actr_cfg;
              actr_cfg.name = actr_node["name"].as<std::string>();
              actr_cfg.type = actr_node["type"].as<std::string>();
              if (!CollectParams(actr_node, actr_cfg.params, kActuatorReservedKeys,
                                 sizeof(kActuatorReservedKeys) / sizeof(kActuatorReservedKeys[0]),
                                 actr_cfg.name)) {
                return false;
              }
              // motors entries may use can_id instead of motor_id; the encos
              // creator keeps that fallback.
              dcu_cfg.actuators.push_back(actr_cfg);
            }
          }
        }

        rhs.push_back(dcu_cfg);
      }
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse DcuNetworkConfig failed, {}", e.what());
      return false;
    }
  }
};

}  // namespace YAML
