#pragma once

// cpp
#include <cerrno>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <string>
#include <type_traits>

// projects
#include "common_type.h"
#include "internal/common_utils.h"

namespace xyber {

namespace param_utils_detail {

// Shared string -> number conversion. Values go through long long / long
// double so a string like "300" still range-checks into uint8_t correctly
// (direct strtoul into uint8_t would silently truncate).
inline bool Parse(const std::string& s, long long& out) {
  char* end = nullptr;
  errno = 0;
  out = std::strtoll(s.c_str(), &end, 10);
  return end != nullptr && *end == '\0' && errno != ERANGE;
}

inline bool Parse(const std::string& s, long double& out) {
  char* end = nullptr;
  out = std::strtold(s.c_str(), &end);
  return end != nullptr && *end == '\0';
}

}  // namespace param_utils_detail

/**
 * @description: Read a numeric param; return def when missing. A present but
 *               malformed value logs ERROR and also returns def - caller sees
 *               the bad key in the log.
 */
template <typename T>
inline T GetParam(const ParamMap& params, const std::string& key, T def) {
  auto it = params.find(key);
  if (it == params.end()) return def;

  if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    long long v = 0;
    if (!param_utils_detail::Parse(it->second, v) ||
        v < (long long)std::numeric_limits<T>::min() ||
        v > (long long)std::numeric_limits<T>::max()) {
      LOG_ERROR("Param %s = \"%s\" is not a valid %s, using default %lld.", key.c_str(),
                it->second.c_str(), "integer", (long long)def);
      return def;
    }
    return (T)v;
  } else if constexpr (std::is_floating_point_v<T>) {
    long double v = 0;
    if (!param_utils_detail::Parse(it->second, v)) {
      LOG_ERROR("Param %s = \"%s\" is not a valid number, using default.", key.c_str(),
                it->second.c_str());
      return def;
    }
    return (T)v;
  } else if constexpr (std::is_same_v<T, std::string>) {
    return it->second;
  } else {
    static_assert(!sizeof(T), "GetParam: unsupported type");
  }
}

/**
 * @description: Read a required numeric param; false when missing or
 *               malformed (malformed also logs ERROR). Callers turn false
 *               into a create failure so config errors surface at startup.
 */
template <typename T>
inline bool GetRequiredParam(const ParamMap& params, const std::string& key, T& out) {
  auto it = params.find(key);
  if (it == params.end()) {
    LOG_ERROR("Required param %s is missing.", key.c_str());
    return false;
  }

  if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    long long v = 0;
    if (!param_utils_detail::Parse(it->second, v) ||
        v < (long long)std::numeric_limits<T>::min() ||
        v > (long long)std::numeric_limits<T>::max()) {
      LOG_ERROR("Required param %s = \"%s\" is not a valid integer in range.", key.c_str(),
                it->second.c_str());
      return false;
    }
    out = (T)v;
    return true;
  } else if constexpr (std::is_floating_point_v<T>) {
    long double v = 0;
    if (!param_utils_detail::Parse(it->second, v)) {
      LOG_ERROR("Required param %s = \"%s\" is not a valid number.", key.c_str(),
                it->second.c_str());
      return false;
    }
    out = (T)v;
    return true;
  } else if constexpr (std::is_same_v<T, std::string>) {
    out = it->second;
    return true;
  } else {
    static_assert(!sizeof(T), "GetRequiredParam: unsupported type");
  }
}

/**
 * @description: WARN once per unconsumed key, so a typo like "can_idd" in the
 *               YAML is visible instead of silently ignored. who names the
 *               slave / actuator for the log.
 */
inline void WarnUnusedParams(const ParamMap& params,
                             std::initializer_list<const char*> consumed,
                             const std::string& who) {
  for (const auto& [key, value] : params) {
    bool known = false;
    for (const char* c : consumed) {
      if (key == c) {
        known = true;
        break;
      }
    }
    if (!known) {
      LOG_WARN("%s: param \"%s\" (\"%s\") is not recognized by this slave/actuator type.",
               who.c_str(), key.c_str(), value.c_str());
    }
  }
}

}  // namespace xyber
