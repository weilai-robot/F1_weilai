# 混合 EtherCAT 异构从站双协议底层链路说明

> **任务目标**：打通新架构下底层链路通信，实现两种协议转发  
> **工作令**：YF24251JY-004  
> **软件工程**：`agibot_x1_infer`  
> **关联方案**：[混合 EtherCAT 架构融合方案](hybrid_dcu_demo_integration.zh_CN.md)

---

## 1. 背景与目标

### 1.1 任务背景

F1 人形机器人以智元 X1 为技术基线，底层通信需满足 1kHz 控制频率、低抖动、高可靠等实时性要求。控制指令从主站下发，经协议转换节点转发至各关节电机，执行器状态再沿原路径回传——其中**指令传输与状态回传**是整个控制链路的物理基础。

原 X1 架构为「主控 + 双 DCU」：两台 DCU 均以 240B 固定 PDO 经 EtherCAT 与主站通信，再经 CANFD 广播转发至 PowerFlow 电机。新架构调整为：

- **上肢**：保留智元 `body` DCU + PowerFlow 电机（协议路径一）；
- **下肢/腰部**：改用 Encos（因克斯/encos）EtherCAT 开发板 + Encos 电机（协议路径二）；
- **合网**：两类从站挂载于**同一条 EtherCAT 总线**，主站统一调度。

### 1.2 本文任务范围

本文聚焦**底层链路打通**与**双协议转发实现**，不涉及上层运动控制算法。

| 在范围内 | 不在范围内 |
|----------|------------|
| EtherCAT 主站周期收发 | RL 控制 / MPC / WBC 算法 |
| DCU ↔ CANFD（PowerFlow）协议转发 | 传动层运动学标定 |
| Encos 板 ↔ CAN（Encos 电机）协议转发 | 自研 DCU 硬件 PCB 设计 |
| 异构从站同网共存 | 完整 31 自由度整机联调 |

### 1.3 任务目标

1. 在同一条 EtherCAT 总线上，实现 **DCU 从站** 与 **Encos 从站** 共存并稳定进入 OP；
2. 主站软件完成 **两种协议的双向转发**：
   - 下行：统一 `SetMitCmd()` → 分别编码为 PowerFlow CANFD 帧 / Encos CAN 帧 → 写入各自 PDO；
   - 上行：分别解析 PDO 反馈 → 统一 `GetPosition/Velocity/Effort()`；
3. 满足 1kHz DC 同步通信，WKC 稳定，为上层 `DcuDriverModule` 提供可用执行器接口。

---

## 2. 结论概览

### 2.1 可行性

**已实现，方案可行。** SOEM 支持同一总线挂载不同 PDO 大小的从站。当前代码已完成：

- `Dcu`（240B PDO）与 `EncosSlave`（86B 有效载荷）同网注册；
- `XyberController` 统一 API 按执行器名自动路由至两类节点；
- `DcuDriverModule::InitDcu()` 按 YAML `type: dcu / Encos` 分流初始化；
- encos 从站 SDOCA 问题已修复（见 `doc/ethercat_sdoca_issue.md`）。

### 2.2 推荐拓扑

| ecat_id | 从站名 | 类型 | 协议转发路径 | 当前状态 |
|---------|--------|------|--------------|----------|
| 1 | `body` | DCU | EtherCAT 240B → CANFD → PowerFlow | 已配置（上肢） |
| 2 | `waist` | Encos | EtherCAT 86B → CAN → Encos 电机 | 已配置（腰部 yaw） |
| 3 | `left_leg` | Encos | 同上，6 路 passage | 配置已备，待启用 |
| 4 | `right_leg` | Encos | 同上，6 路 passage | 配置已备，待启用 |

当前 `dcu_x1.yaml` 处于**渐进联调**状态：仅启用 `body` + `waist` 两个从站，验证异构合网最小链路。

---

## 3. 两套从站协议差异

底层「两种协议转发」的本质：主站将统一的 MIT 控制语义，分别编码为两套不同的 PDO/CAN 协议，由各自从站硬件转发至电机总线。

### 3.1 协议路径一：DCU → CANFD → PowerFlow

**EtherCAT PDO**（`dcu.h`，严格 240B）：

```text
下发 DcuSendPacket (240B)
  └─ canfd[3].{ctrl, data[64]}   # 3 路 CANFD，每路广播 64B
  └─ imu_cmd

反馈 DcuRecvPacket (240B)
  └─ canfd[3][64]                # 各设备独立回传
  └─ imu{acc, gyro, quat}
```

**CANFD 转发规则**（`dcu.cpp`）：

- 每路 CANFD 发送 1 帧广播，Payload 64B；
- 各 PowerFlow 执行器以自身 `can_id` 为 offset，从 `data[]` 中读取 8B 控制指令；
- `PowerFlowR/L` 等 `Actuator` 子类负责 MIT 命令的位域打包与反馈解包；
- 每路最多 8 个设备，3 路合计最多 24 个。

**配置字段**：`channel_1/2/3` + `can_id` + `type: POWER_FLOW_R86/R52/L28/...`

### 3.2 协议路径二：Encos 板 → CAN → Encos 电机

**EtherCAT PDO**（`encos_slave.h`，有效 86B）：

```text
EncosSlavePacket (86B)
  └─ motor_num                  # 有效电机数
  └─ can_ide                    # 0=标准帧, 1=扩展帧（MIT 需 1）
  └─ motor[6]                   # 每路 Motor_Msg (14B)
       └─ {id, rtr, dlc, data[8]}
```

**CAN 转发规则**（`encos_slave.cpp` + `encos_motor.cpp`）：

- `passage` 1~3 → 板端 CAN1；`passage` 4~6 → 板端 CAN2；
- `EncosMotor::SetMitCmd()` 将 pos/vel/toq/kp/kd 编码为 8B CAN 数据，写入对应 `Motor_Msg` 槽位；
- `EncosMotor::UpdateFeedback()` 从反馈 `Motor_Msg` 解包位置、速度、力矩；
- PDO 上报可能大于 86B（如 186/192B），主站 `Pull/PushRecvData` 仅读写前 86B。

**配置字段**：`motors[]` + `passage(1~6)` + `motor_id` + `can_ide`

### 3.3 双协议对比

| 维度 | DCU / PowerFlow | Encos 板 / Encos 电机 |
|------|-----------------|-------------------|
| PDO 大小 | 固定 240B（严格匹配） | 有效 86B（PDO 可更大） |
| 下游总线 | CANFD 5Mbps | CAN |
| 寻址 | `CtrlChannel` + `can_id` | `passage` + `motor_id` |
| 执行器类 | `PowerFlowR/L`, `OmniPicker` | `EncosMotor` |
| MIT 编码 | PowerFlow 位域 | `soem_demo_cpp` 16+12bit 打包 |
| 使能方式 | `RequestState` 状态机 | MIT 零力矩保持 |

---

## 4. 架构融合设计

### 4.1 设计原则

1. **转发层隔离**：两种协议编解码各自封装在 `Actuator` 子类中，互不影响；
2. **节点自治**：`Dcu` 与 `EncosSlave` 各自管理 PDO 缓冲，仅实现 `PullSendData` / `PushRecvData`；
3. **统一调度**：`EthercatManager` 单一 1kHz 循环收发所有从站，不假设固定 PDO 长度；
4. **统一 API**：`XyberController` 对上层屏蔽协议差异，`SetMitCmd` / `GetPosition` 等按执行器名路由；
5. **配置驱动**：YAML `type: dcu / Encos` 决定走哪条转发路径，无需改代码即可增减从站。

### 4.2 双协议转发数据流

**下行（控制指令转发）**：

```text
SetMitCmd(name, pos, vel, effort, kp, kd)
        │
        ├─[DCU 路径]──────────────────────────────────────┐
        │  XyberController → Dcu::SetMitCmd               │
        │  → PowerFlow::SetMitCmd()                       │
        │  → 写入 canfd[ch].data[can_id 偏移]              │
        │  → DcuSendPacket (240B)                         │
        │                                                  │
        └─[Encos 路径]──────────────────────────────────────┤
           XyberController → EncosSlave::SetMitCmd          │
           → EncosMotor::SetMitCmd()                        │
           → 编码 8B CAN 数据到 motor[passage-1]         │
           → EncosSlavePacket (86B)                         │
                                                          │
        EthercatManager::WorkLoop()                       │
        PullSendData() → ec_slave[id].outputs             │
        → ec_send_processdata() ──────────────────────────┘
                          │
                          ▼
                   从站硬件 → CANFD / CAN → 电机
```

**上行（状态反馈转发）**：

```text
ec_receive_processdata()
        │
        ▼
PushRecvData() → DcuRecvPacket / EncosSlavePacket
        │
        ├─[DCU]  PowerFlow 解包 canfd[ch] → position/velocity/effort
        │
        └─[Encos]  EncosMotor::UpdateFeedback() → position/velocity/effort
        │
        ▼
GetPosition / GetVelocity / GetEffort(name)
        │
        ▼
DcuDriverModule::PublishLoop() → /joint_states
```

### 4.3 软件分层

```text
DcuDriverModule（配置解析 + 发布循环）
          │
          ▼
XyberController（统一执行器 API，双路径路由）
   │                              │
   ▼                              ▼
Dcu                          EncosSlave
PowerFlow Actuator           EncosMotor Actuator
   │                              │
   └──── EthercatManager + SOEM ──┘
                  │
                  ▼
         EtherCAT 总线（1kHz DC）
```

### 4.4 关键实现要点

| 要点 | 实现位置 | 说明 |
|------|----------|------|
| 异构 PDO 收发 | `ethercat_manager.cpp` WorkLoop | 按各从站 `Obytes/Ibytes` 调用 Pull/Push |
| Encos PDO 截断 | `encos_slave.cpp` | 仅读写 86B，防止大 PDO 越界 |
| SDOCA 修复 | `ethercat_manager.cpp` Start() | 清除 `ECT_COEDET_SDOCA`，encos 从站可进 OP |
| API 路由 | `xyber_controller.cpp` | `actuator_dcu_map_` / `actuator_encos_map_` 分发 |
| 配置分流 | `dcu_driver_module.cc` InitDcu() | `type: dcu` / `type: Encos` 分支创建 |
| DC 同步 | `dcu_x1.yaml` ethercat 段 | `enable_dc: true`, `cycle_time_ns: 1000000` |

---

## 5. 已实现代码说明

本节描述当前仓库中已落地的双协议转发实现（对应融合方案 §5，以实际代码为准）。

### 5.1 `xyber_controller`（核心转发层）

**DCU 路径（保留）**：

- `CreateDcu(name, ecat_id)` — 注册 DCU 从站节点；
- `AttachActuator(dcu_name, ch, type, name, can_id)` — 绑定 PowerFlow 执行器至 CANFD 通道；
- `Dcu::RegisterActuator()` — 将执行器数据区绑定到 `canfd[ch].data` 偏移。

**Encos 路径（新增）**：

- `CreateEncosSlave(name, ecat_id)` — 注册 Encos 从站节点；
- `AttachEncosMotor(slave_name, passage, type, name, motor_id)` — 绑定 EncosMotor 至 passage 槽位；
- `SetEncosCanIde(slave_name, can_ide)` — 设置 CAN 帧类型（MIT 用 `can_ide=1`）；
- `EncosSlave::RegisterActuator(actr, passage)` — passage 1~6 映射 `motor[0..5]`；
- `EncosMotor::SetMitCmd()` / `UpdateFeedback()` — Encos CAN 协议编解码。

**统一 API（双路径自动路由）**：

```cpp
SetMitCmd(name, pos, vel, effort, kp, kd);  // → Dcu 或 EncosSlave
GetPosition(name);  GetVelocity(name);  GetEffort(name);
EnableActuator(name);  ClearError(name);  DisableActuator(name);
```

### 5.2 `dcu_driver_module`（配置与发布）

- `InitDcu()`：`dcu_network[].type` 为 `dcu` 或 `Encos` 时分别调用对应创建接口；
- `config_parse.h`：`DcuConfig` 扩展 `type`、`passage`、`motor_id`、`can_ide` 字段；
- `PublishLoop()`：统一调用 `GetPosition/Velocity/Effort`，不区分协议来源；
- Encos 从站跳过 IMU 初始化（`imu_enable` 仅对 DCU 生效）。

### 5.3 YAML 配置（`cfg/dcu_x1.yaml`）

**当前启用配置**：

```yaml
dcu_network:
  - name: body          # ecat_id=1, type: dcu
    channel_1:
      - name: left_shoulder_yaw_actuator
        type: POWER_FLOW_R52
        can_id: 1

  - name: waist         # ecat_id=2, type: Encos
    can_ide: 1
    motors:
      - name: lumbar_yaw_actuator
        type: Encos_EC_A4315
        passage: 1
        motor_id: 1
```

**待启用**（配置已写好，注释状态）：`left_leg`（ecat_id=3）、`right_leg`（ecat_id=4），各 6 路电机。

---

## 6. IMU 处理

Encos 开发板无 IMU PDO。原 X1 方案中 IMU 经 `hip` DCU 的 UART 接入并打包进 EtherCAT 上行数据。

迁移后不能继续依赖 `hip` DCU 的 IMU。当前 `dcu_x1.yaml` 中 `imu_dcu_name: hip` 需在 IMU 独立方案确定后更新。底层链路打通阶段，IMU 可暂时不影响双协议转发验证。

---

## 7. 返回值结构与上层影响评估

两种协议底层帧格式不同，但适配层统一输出：

| 接口 | 单位 | 说明 |
|------|------|------|
| `GetPosition()` | rad | 两类执行器均已换算 |
| `GetVelocity()` | rad/s | Encos 内部已处理 rpm 换算 |
| `GetEffort()` | Nm | 按各自反馈帧解析 |

上层 `DcuDriverModule` 的 transmission 层与 `/joint_cmd` 接口**无需修改**。联调时需按电机分别校准：

- MIT 参数限幅（`PF_R86_MIT_MODE_DEFAULT_PARAM` vs `Encos_MIT_MODE_DEFAULT_PARAM`）；
- 正方向、零点、饱和限幅。

---

## 8. 分阶段打通计划

本节面向**底层链路通信打通**任务，与融合方案整机联调阶段对应但范围更聚焦。

### 阶段 0：环境确认

- PREEMPT_RT 内核、网卡 `ifname`、CPU 绑核与线程优先级；
- 确认 `body` DCU 与 `waist` Encos 板 `ecat_id`、电机 `can_id`/`motor_id`；
- 准备 Wireshark（EtherCAT TAP 旁路）与 CAN 分析仪，用于协议级诊断。

### 阶段 1：单 Encos 从站链路（当前阶段）

- 仅接 `waist` Encos 板 + 1 个电机（`lumbar_yaw_actuator`）；
- 验证：从站进 OP、PDO 收发、`SetMitCmd` 下发、`GetPosition/Velocity/Effort` 有连续反馈；
- 验收：1kHz 下 WKC 稳定，Encos MIT 帧可在 CAN 分析仪上看到。

### 阶段 2：DCU + Encos 异构合网

- 同时接入 `body` DCU 与 `waist` Encos 板；
- 验证：两类从站同周期收发，互不影响；DCU 的 PowerFlow 与 Encos 的 EncosMotor 可同时 `SetMitCmd`；
- 验收：周期抖动可接受，两路径执行器均响应控制指令。

### 阶段 3：单腿 6 电机

- 启用 `left_leg`（ecat_id=3），接入 6 路 Encos 电机；
- 验证：6 个 `passage` 槽位无串扰，`actuator_list` 与 `transmission` 映射正确；
- 通过 `DcuDriverModule` 发布 `/joint_states`，检查数据连续性。

### 阶段 4：全下肢 + 腰 + 上肢合网

- 启用 `right_leg`（ecat_id=4），四从站（1 DCU + 3 Encos）全部在线；
- 验证：长时间 OP 稳定、WKC 无异常、全部注册执行器可读写；
- 里程碑：**双协议转发链路全面打通**。

---

## 9. 风险与规避

| 风险 | 规避 |
|------|------|
| encos 从站无法进 OP（AL 0x001E） | 已修复：清除 SDOCA 位（`ethercat_sdoca_issue.md`） |
| Encos PDO 大于 86B 导致缓冲区越界 | `EncosSlave` 仅读写 86B |
| DCU PDO 非 240B 导致 Init 失败 | 确认 DCU 固件 PDO 映射为 240B |
| 6 路电机单板超限 | 左右腿分板，各 6 路 |
| 两类 MIT 参数范围不同 | 下发前按 `MitParam` 限幅 |
| 异构从站导致周期抖动 | PREEMPT_RT + 绑核 + DC 同步 + WKC 监控 |
| passage 槽位冲突 | `RegisterActuator` 有重复检测与日志 |

---

## 10. 验收标准

### 10.1 链路层

1. 所有注册从站稳定进入并保持 EtherCAT OP；
2. 1kHz DC 同步下 WKC 无持续异常；
3. `EthercatManager` 日志中各从站 `Obytes/Ibytes` 与预期一致，`Init()` 全部通过。

### 10.2 协议转发

4. **DCU 路径**：对 PowerFlow 执行器 `SetMitCmd` 后，电机运动；`GetPosition/Velocity/Effort` 与实测一致；
5. **Encos 路径**：对 EncosMotor 执行器 `SetMitCmd` 后，电机运动；CAN 分析仪可看到对应 `motor_id` 的 MIT 帧；
6. **异构共存**：DCU 与 Encos 从站在同一周期循环中同时收发，互不影响。

### 10.3 上层接口

7. `DcuDriverModule` 正常发布 `/joint_states`，量纲为 rad / rad/s / Nm；
8. `EnableActuator` / `ClearError` / `DisableAllActuator` 对两类执行器均有效；
9. 进程退出时执行器可被安全禁用。

---

## 11. 备注

1. 本文档描述**底层双协议转发链路的实现与验证**，架构总体规划见 [hybrid_dcu_demo_integration.zh_CN.md](hybrid_dcu_demo_integration.zh_CN.md)。
2. 代码中类名为 `EncosSlave` / `EncosMotor`，YAML 配置字段为 `type: Encos`；协议对齐 `soem_demo_cpp`。
3. 若 Encos 板固件变更 PDO 布局或 CAN 协议，需同步更新 `encos_slave.h` / `encos_motor.cpp`。
4. 调试建议：EtherCAT 流量用 Wireshark + TAP 旁路抓取；CAN 报文用 CAN 分析仪并联监测，两种工具可分别覆盖两条转发路径。
