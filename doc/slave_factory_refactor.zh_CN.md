# Slave 工厂模式重构方案（dcu / encos 统一创建）

> **软件基线**：`agibot_x1_infer`（本仓库）
> **关联文档**：[混合 EtherCAT 架构融合方案](hybrid_dcu_demo_integration.zh_CN.md)、[混合 EtherCAT 双协议底层链路说明](hybrid_ethercat_dual_protocol_link.zh_CN.md)
> **涉及范围**：`xyber_controller/xyber_api`（核心）+ `dcu_driver_module`（InitDcu）+ example；**YAML 配置零改动**

---

## 1. 背景与目标

### 1.1 现状问题

当前创建从站与执行器时，"类型判断"散落在两层：

| 现状 | 位置 | 问题 |
|---|---|---|
| `InitDcu()` 里 `if (type == "encos") {...} else {...}` | `src/module/dcu_driver_module/src/dcu_driver_module.cc`（InitDcu） | 每加一种从站，加一个分支、一套调用（`CreateDcu` vs `CreateEncosSlave`） |
| `XyberController` 维护 **4 张 map**：`dcu_map_` / `encos_slave_map_` + `actuator_dcu_map_` / `actuator_encos_map_` | `xyber_controller/xyber_api/src/xyber_controller.cpp` | 每个执行器 API 都是"先查 DCU、再查 Encos"双重查找；每加一种从站再加一张 map |
| `AttachActuator` 内 `switch(type)` 创建 PowerFlow 系；`AttachEncosMotor` 另写一套创建 `EncosMotor` | 同上 | 执行器创建逻辑分散在两处 |

之后新增从站类型（自研 DCU、其他厂商板卡），这三处全要动，**分支数随从站类型线性增长**。

### 1.2 目标

**工厂模式 + 公共从站基类**：

1. `DcuDriverModule::InitDcu()` 对 `type` **无感知**：一个 `CreateSlave()` 统一给出从站、一个 `AttachActuator()` 统一挂执行器；
2. `XyberController` 内部单 map、单查找，对上层 API 语义零变化；
3. 新增从站类型 = **新子类文件 + 注册一行**，现有代码零改动（开闭原则）；
4. YAML 配置与解析逻辑（`config_parse.h`）**零改动**，行为兼容（除两处修正，见 §4）。

---

## 2. 工厂模式说明

### 2.1 定义：解决什么问题

工厂模式是**创建型**设计模式：把"调用哪个构造函数、new 哪个具体类"的决策从调用方移走，封装进工厂。调用方从此只依赖两样东西——**抽象基类指针 + 类型标识**，不再出现 `if/switch (type)` 式的创建分支。

### 2.2 三种形态与选型

> 三种形态的代码级对比示例（含"加新类型时改哪里"的逐步演示）见 [工厂模式三种形态对比示例](factory_pattern_comparison.zh_CN.md)。

| 形态 | 做法 | 加新类型时 | 评价（针对本项目） |
|---|---|---|---|
| **简单工厂** | 一个工厂函数里 if/switch 判断 type，`new` 对应类 | **仍要改工厂函数本体** | 只是把 InitDcu 的分支搬了家，开闭原则只做一半 |
| **工厂方法** | 每种产品配一个工厂子类（`DcuFactory`、`EncosSlaveFactory`…） | 新工厂类 + 新产品类 | C++ 样板代码翻倍，本场景过重 |
| **注册式工厂（Registry）** ✅ | 工厂内部一张 `map<类型标识, 创建函数>`，各类型向工厂**注册**自己的创建函数 | **新子类文件 + 注册一行**，现有代码零改动 | 本方案采用 |

选注册式的核心理由：目标是"便于添加新 slave 类型"——只有注册式能做到"新增类型不触碰任何现有文件"。

### 2.3 两个决定成败的工程要点

**要点 ①：先有统一产品类型，工厂才有意义。**

工厂的返回值必须是同一个基类指针。`Dcu` 与 `EncosSlave` 目前只共同继承 `EthercatNode`（协议收发层），但两者各自实现了**完全同名**的一组执行器 API（`EnableActuator` / `SetMitCmd` / `GetPosition`…）。把这层公共接口提炼为抽象基类 `SlaveBase`，工厂返回 `SlaveBase*`，上层从此只见基类。**没有公共基类的工厂是伪工厂**——返回 `variant` 的话，调用方照样得 switch。

**要点 ②：注册时机——静态库下不能用"文件级静态对象自注册"。**

`xyber_controller` 是 `add_library(STATIC)` + `file(GLOB_RECURSE)`。常见写法是在 `dcu.cpp` 里放一个静态 bool 触发自注册——但当该 .o 无外部符号引用时，链接器可能整文件丢弃 → 注册丢失 → 工厂返回空指针，且这类问题在链接选项变化时才暴露，很隐蔽。**改用**：`SlaveFactory::Create()` 首次调用时惰性执行 `RegisterBuiltinSlaves()`（在工厂自己的 .cpp 里集中注册 `"dcu"` / `"encos"`），行为确定、无链接陷阱；新增类型 = 在这一处加一行。

### 2.4 本项目映射：两个工厂 + 一张统一挂载签名

只做从站工厂不够——挂执行器时 `AttachActuator`（DCU）与 `AttachEncosMotor`（Encos）仍是两套方法，内部创建执行器也是两套逻辑，因此需要**两个工厂**：

```text
                EthercatNode（现有，协议收发层）
                        ▲
              SlaveBase（新增抽象基类）
              ├─ RegisterActuator(Actuator*, const ActuatorAttachInfo&) = 0   ← 统一挂载
              ├─ EnableActuator / SetMitCmd / GetPosition / ...（纯虚，签名照抄现状）
              └─ HasImu() / GetImuData() / ApplyImuOffset() / SetCanIde()（默认安全空实现）
        ┌────────────┴────────────┐
       Dcu（240B PDO）        EncosSlave（86B PDO）          ← 未来 XxxSlave
        │                        │
   PowerFlow系/L28/OmniPicker  EncosMotor                    ← 由 ActuatorFactory 创建

  SlaveFactory:     "dcu"|"encos"|...  ──→  SlaveBase*
  ActuatorFactory:  ActuatorType        ──→  Actuator*
```

DCU 与 Encos 的挂载参数差异（`channel + can_id` vs `passage + motor_id`）不靠 if 分支处理，而是装进参数结构体 `ActuatorAttachInfo`，**各取所需**：DCU 读 `channel/can_id`，Encos 读 `passage/motor_id`，互不干涉——以数据驱动取代类型分支。

改后调用链：

```text
InitDcu（改后）
  ├─ CreateSlave({type, name, ecat_id, can_ide})        → SlaveFactory::Create → slave_map_
  └─ AttachActuator(slave_name, {type, name, 绑定参数})   → ActuatorFactory::Create → Actuator*
                                                            → slave->RegisterActuator(actr, info)
                                                            → actuator_slave_map_
```

### 2.5 收益与边界（诚实评估）

**收益**：InitDcu 零类型分支；XyberController 4 张 map → 2 张，所有 API 双查找 → 单查找；新增从站类型时 `InitDcu` / `XyberController` / 其他从站类零改动。

**不解决的**：两套 PDO 协议、CAN 编解码差异仍然各自封装在 `Dcu` / `EncosSlave` / `PowerFlow` / `EncosMotor` 内部——工厂只解耦**创建侧**，不消除协议实现差异（这正是想要的边界）。

**代价**：一层间接（工厂查找）；两处行为差异属修正性质，见 §4。

---

## 3. 修改方案

### 3.1 新增数据结构（`xyber_api/include/common_type.h`）

```cpp
struct SlaveConfig {          // 从站创建参数：各类型消费自己认识的字段
  std::string type;           // "dcu" | "encos" | ...
  std::string name;
  int32_t ecat_id = 0;
  uint8_t can_ide = 1;        // 仅 encos 用（MIT 力位混合需扩展帧），dcu 忽略
};

struct ActuatorAttachInfo {   // 执行器创建+绑定参数，映射自 YAML 的 channel_x / motors
  ActuatorType type = ActuatorType::UNKOWN;
  std::string name;
  CtrlChannel channel = CtrlChannel::CTRL_CH1;  // dcu canfd 通道
  uint8_t can_id = 0;                           // dcu can id
  uint8_t passage = 0;                          // encos PDO 槽 1~6
  uint8_t motor_id = 0;                         // encos 电机 CAN id
};
```

### 3.2 新增文件（5 个；`file(GLOB_RECURSE)` 自动纳入编译，CMake 无需改动）

| 文件 | 内容 |
|---|---|
| `xyber_api/include/internal/slave_base.h` | 抽象基类：统一 `RegisterActuator(Actuator*, info) → bool`、全组执行器纯虚 API、IMU / can_ide 默认空实现 |
| `xyber_api/include/internal/slave_factory.h` + `src/slave_factory.cpp` | `Register(type, creator)` / `Create(cfg)`；集中注册 `"dcu"` → `new Dcu`、`"encos"` → `new EncosSlave(name, id, can_ide)`；未知类型返回 nullptr + 日志 |
| `xyber_api/include/internal/actuator_factory.h` + `src/actuator_factory.cpp` | 同构；注册 R86 / R52（各自 MIT 量程）/ L28 / OmniPicker / 三种 Encos 型号（同一 `EncosMotor` 类）；创建器内校验 `can_id` / `motor_id` 非零 |

### 3.3 现有类修改

**`Dcu`（`internal/dcu.h` / `src/dcu.cpp`）**

- 基类 `EthercatNode` → `SlaveBase`（`name_` 上移至基类 protected）；
- `RegisterActuator` 统一为 `(Actuator*, const ActuatorAttachInfo&) → bool`：内部逻辑不变（通道容量校验 + `SetDataFiled(canfd[ch].data, canfd[ch])`，通道取 `actr->GetCtrlChannel()`），**超限由静默失败改为返回 false**；
- `ImuAppleUserOffset` 改名 override 为 `ApplyImuOffset`，新增 `HasImu() { return true; }`；
- `SetChannelId` 保持 Dcu 具体方法不变。

**`EncosSlave`（`internal/encos_slave.h` / `src/encos_slave.cpp`）**

- 基类 → `SlaveBase`；构造函数**增加 `can_ide` 参数**（默认 1，YAML 的 `can_ide` 随创建传入，替代"创建后再 SetCanIde"的时序）；
- `RegisterActuator` 统一签名，passage 取 `info.passage`，现有槽位校验 / CAN 组容量校验 / 绑定逻辑不变；
- `SetCanIde` 变为基类虚函数的 override。

**`XyberController`（`include/xyber_controller.h` / `src/xyber_controller.cpp`，改动核心）**

- 4 张静态 map → 2 张：`slave_map_`（name → `SlaveBase*`）、`actuator_slave_map_`（执行器名 → `SlaveBase*`）；
- 新统一 API：

```cpp
bool CreateSlave(const SlaveConfig& cfg);
bool AttachActuator(const std::string& slave_name, const ActuatorAttachInfo& info);
```

- 统一挂载流程：查 `slave_map_` → 执行器名查重 → `ActuatorFactory::Create(info)` → `slave->RegisterActuator(actr, info)`（失败则 `delete actr`）→ 登记 `actuator_slave_map_`。原 `motor_id == 0` / passage 范围 / `IsEncosMotorType` 校验随创建逻辑移入工厂与 `EncosSlave`；
- **旧 API（`CreateDcu` / `CreateEncosSlave` / 旧 5 参 `AttachActuator` / `AttachEncosMotor`）保留为 `@deprecated` 兼容薄封装**——外部旧代码不破，仓库内两处调用方（`dcu_driver_module.cc`、`example/main.cpp`）改走新 API；`SetEncosCanIde` 经基类虚函数保留；
- 所有执行器 API、`Start()` 节点注册、`EnableAllActuator` 等双查找 / 双循环 → 单查找 / 单循环；
- `ApplyDcuImuOffset` 经基类虚函数接通 `imu_cmd` 置位逻辑（见 §4.2）。

**`DcuDriverModule::InitDcu()`（`src/dcu_driver_module.cc`）——类型分支消失：**

```cpp
for (const auto& slave_cfg : dcu_network_cfg_) {
  if (!slave_cfg.enable) continue;
  bool ret = xyber_ctrl_->CreateSlave(
      {.type = slave_cfg.type, .name = slave_cfg.name,
       .ecat_id = (int32_t)slave_cfg.ecat_id, .can_ide = slave_cfg.can_ide});
  AIMRT_CHECK_ERROR_THROW(ret, "Slave {} (type {}) register failed.", slave_cfg.name, slave_cfg.type);

  auto attach_fn = [&](const DcuConfig::Actuator& actr, CtrlChannel ch) {
    if (不在 actuator_name_list_) { AIMRT_WARN("Actuator {} not found in actuator_list. ignored.", actr.name); return; }
    AIMRT_CHECK_ERROR_THROW(
        xyber_ctrl_->AttachActuator(slave_cfg.name,
            {.type = StringToType(actr.type), .name = actr.name, .channel = ch,
             .can_id = (uint8_t)actr.can_id, .passage = actr.passage, .motor_id = actr.motor_id}),
        "Actuator {} register failed.", actr.name);
  };
  for (size_t ch = 0; ch < 3; ch++)
    for (const auto& actr : slave_cfg.ch[ch]) attach_fn(actr, (CtrlChannel)ch);      // dcu 路径
  for (const auto& actr : slave_cfg.motors) attach_fn(actr, CtrlChannel::CTRL_CH1); // encos 路径（channel 不被消费）
}

// IMU 段：type=="encos" 判断消失——无 IMU 的从站其 ApplyImuOffset 是默认空实现
for (const auto& slave_cfg : dcu_network_cfg_) {
  if (!slave_cfg.imu_enable) continue;
  xyber_ctrl_->ApplyDcuImuOffset(slave_cfg.name);
}
```

**`example/main.cpp`**：改用新 API 作示范（`CreateSlave` + `AttachActuator(info)`）。

**YAML（`cfg/dcu_x1.yaml`）与 `config_parse.h` 零改动**：`channel_1/2/3` 与 `motors[]` 两个列表在上层汇入同一个 `AttachActuator`。

### 3.4 两处有意的行为差异（修正性质）

1. **Dcu 通道超限**：现状 `Dcu::RegisterActuator` 返回 void，单通道 8 设备上限超限只打日志、XyberController 仍登记成功 → 运行期该执行器无响应且无报错。改后 attach 失败、`InitDcu` 抛错终止，配置错误尽早暴露。
2. **`ApplyDcuImuOffset` 接通**：现状该函数找到 DCU 后**空返回**，`ImuAppleUserOffset`（`imu_cmd=1` → 30ms → `0`）从未被调用，而模块侧明确调用了它。改后经虚函数真正生效。若需严格保持旧行为，可保留空实现。

### 3.5 重构收益兑现：之后新增一种从站类型

1. 新建 `xxx_slave.h/cpp` 继承 `SlaveBase`（PDO 布局 + 执行器 API + `Init/Pull/Push`）；
2. `slave_factory.cpp` 的 `RegisterBuiltinSlaves()` 加一行 `Register("xxx", ...)`；
3. 带新执行器协议的话：`XxxMotor : Actuator` + `actuator_factory.cpp` 注册 + `common_type.h` 加 enum；
4. YAML 加 `type: xxx` 从站段。**`InitDcu` / `XyberController` / 其他从站类零改动。**

---

## 4. 实施清单

| # | 事项 | 涉及文件 | 状态 |
|---|---|---|---|
| 1 | 新增 `SlaveConfig` / `ActuatorAttachInfo` | `common_type.h` | ✅ 完成 |
| 2 | 新增 `SlaveBase` 抽象基类 | `internal/slave_base.h` | ✅ 完成 |
| 3 | 新增 `SlaveFactory` | `internal/slave_factory.h` + `src/slave_factory.cpp` | ✅ 完成 |
| 4 | 新增 `ActuatorFactory` | `internal/actuator_factory.h` + `src/actuator_factory.cpp` | ✅ 完成 |
| 5 | `Dcu` 换基类 + 统一 `RegisterActuator` | `dcu.h` / `dcu.cpp` | ✅ 完成 |
| 6 | `EncosSlave` 换基类 + ctor 加 `can_ide` | `encos_slave.h` / `encos_slave.cpp` | ✅ 完成 |
| 7 | `XyberController` 单 map + 新 API + 旧 API 兼容封装 | `xyber_controller.h` / `xyber_controller.cpp` | ✅ 完成 |
| 8 | `InitDcu()` 去类型分支 | `dcu_driver_module.cc` | ✅ 完成 |
| 9 | example 改用新 API | `example/main.cpp` | ✅ 完成 |

> 编译验证进度：`dcu.cpp` / `encos_slave.cpp` / `encos_motor.cpp` / `power_flow.cpp` / `slave_factory.cpp` / `actuator_factory.cpp` / `xyber_controller.cpp` / `example/main.cpp` 已通过 `g++ -std=c++20 -fsyntax-only` 语法检查；`dcu_driver_module.cc` 依赖 AimRT/ROS2 头文件，需在 Linux 构建机全量编译确认（见 §5）。

---

## 5. 验证方案

1. **编译**（Linux 构建机）：
   - 最小闭环：`cd src/module/dcu_driver_module/xyber_controller && ./build.sh`（soem 内置，验证库 + example）；
   - 全量：仓库根 `./build.sh $DOWNLOAD_FLAGS`（AimRT 依赖）；
2. **真机回归**（body DCU + waist Encos 板最小拓扑，YAML 不变）：
   - 启动日志确认从站经工厂创建（`EncosSlave waist id 2 created` / `Dcu body id 1 created`）、`Init()` 通过、两从站进 OP、WKC 稳定；
   - `/joint_states` 中 encos 路径（`lumbar_yaw_actuator`）与 dcu 路径（`left_shoulder_yaw_actuator`）反馈正常，量纲 rad / rad·s / Nm；
   - 指令下发后两路径执行器均响应 `SetMitCmd`；
3. **兼容性**：旧 API 封装编译通过（example 改造前后均可编）；
4. **行为差异回归**：故意配错（单通道超 8 设备 / passage 重复），确认 `InitDcu` 明确报错而非静默通过。

---

## 6. 备注

1. 代码中类名为 `SlaveBase` / `SlaveFactory` / `ActuatorFactory`；YAML 配置字段（`type: dcu` / `type: encos`）与解析逻辑保持不变；
2. 若后续从站固件 PDO 布局或 CAN 协议变更，仍按原路径更新对应从站/执行器子类，工厂层不受影响；
3. 旧 API 兼容封装为过渡措施，待全部调用方迁移后可移除。
