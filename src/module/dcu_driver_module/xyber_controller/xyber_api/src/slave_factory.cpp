/*
 * @Author: zhangmy
 * @Date: 2026-10-08 10:30:00
 * @LastEditors: zhangmy
 * @LastEditTime: 2026-10-08 10:30:00
 */

#include "internal/slave_factory.h"

// projects
#include "internal/common_utils.h"
#include "internal/dcu.h"
#include "internal/encos_slave.h"
#include "internal/param_utils.h"
#include "internal/slave_base.h"

namespace xyber {

// 声明Registry()方法，返回函数的内部静态 map 变量 registry（全局唯一）
std::unordered_map<std::string, SlaveCreator>& SlaveFactory::Registry() {
  static std::unordered_map<std::string, SlaveCreator> registry;
  return registry;
}

// 注册函数，注册成功返回true，注册失败返回false
bool SlaveFactory::Register(const std::string& type, SlaveCreator creator) {
  if (!creator) {
    LOG_ERROR("SlaveFactory register %s failed: null creator.", type.c_str());
    return false;
  }
  if (!Registry().emplace(type, std::move(creator)).second) {
    LOG_ERROR("SlaveFactory register %s failed: type already registered.", type.c_str());
    return false;
  }
  return true;
}

void SlaveFactory::RegisterBuiltinSlaves() {
  // 立即执行的 lambda 表达式，done 是静态局部变量，只在第一次调用时初始化一次
  // 注册代码只会执行一次，之后再调用 RegisterBuiltinSlaves() 什么都不做
  static bool done = [] {
    const bool dcu_ok = Register("dcu", [](const SlaveConfig& cfg) -> SlaveBase* {
      // dcu takes no slave-level params; anything present is a config mistake.
      WarnUnusedParams(cfg.params, {}, cfg.name);
      return new Dcu(cfg.name, cfg.ecat_id);
    });
    const bool encos_ok = Register("encos", [](const SlaveConfig& cfg) -> SlaveBase* {
      // can_ide: 1 = extended frame (force-position hybrid / MIT), legacy default.
      const uint8_t can_ide = GetParam<uint8_t>(cfg.params, "can_ide", 1);
      WarnUnusedParams(cfg.params, {"can_ide"}, cfg.name);
      return new EncosSlave(cfg.name, cfg.ecat_id, can_ide);
    });
    if (!dcu_ok || !encos_ok) {
      LOG_ERROR("SlaveFactory: builtin registration incomplete (dcu=%d, encos=%d).",
                static_cast<int>(dcu_ok), static_cast<int>(encos_ok));
    }
    return dcu_ok && encos_ok;
  }();

  // 消除“变量未使用”的编译警告
  (void)done;
}

SlaveBase* SlaveFactory::Create(const SlaveConfig& cfg) {
  RegisterBuiltinSlaves();

  auto it = Registry().find(cfg.type);
  if (it == Registry().end()) {
    LOG_ERROR("SlaveFactory: unknown slave type %s (name %s).", cfg.type.c_str(),
              cfg.name.c_str());
    return nullptr;
  }
  return it->second(cfg);
}

}  // namespace xyber
