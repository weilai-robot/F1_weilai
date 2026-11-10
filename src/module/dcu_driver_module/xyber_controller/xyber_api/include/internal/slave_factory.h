/*
 * @Date: 2026-10-08 10:30:00
 * @LastEditTime: 2026-10-08 10:30:00
 */

#pragma once

// cpp
#include <functional>
#include <string>
#include <unordered_map>

// projects
#include "common_type.h"

namespace xyber {

class SlaveBase;

using SlaveCreator = std::function<SlaveBase*(const SlaveConfig&)>;

/**
 * @description: Registry-style slave factory. Maps a slave type string
 *               ("dcu", "encos", ...) to its creator. New slave types plug
 *               in with a single Register() call in RegisterBuiltinSlaves()
 *               (slave_factory.cpp); callers stay type-agnostic.
 */
class SlaveFactory {
 public:
  /**
   * @brief: Register a creator under a slave type key
   * @return {bool} false if the key is already taken or the creator is null
   */
  static bool Register(const std::string& type, SlaveCreator creator);

  /**
   * @brief: Create a slave by SlaveConfig.type
   * @return {SlaveBase*} nullptr for unknown type (logged)
   */
  static SlaveBase* Create(const SlaveConfig& cfg);

 private:
  // Builtin types are registered here on first Create(): file-scope
  // self-registration would rely on the linker keeping otherwise
  // unreferenced object files of a static library, which is not guaranteed.
  static void RegisterBuiltinSlaves();
  static std::unordered_map<std::string, SlaveCreator>& Registry();
};

}  // namespace xyber
