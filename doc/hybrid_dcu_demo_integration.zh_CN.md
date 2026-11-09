# F1 人形机器人混合 EtherCAT 架构融合方案

> **任务来源**：研发类项目《F1具身智能技术研究》  
> **工作令**：YF24251JY-004  
> **关联文档**：《F1人形机器人控制系统研发与整机验证》《F1人形机器人自研DCU技术方案》  
> **软件基线**：`agibot_x1_infer`（智元 X1 开源代码）

---

## 1. 背景与目标

### 1.1 项目背景

F1 人形机器人以智元 X1 为主要技术参考与硬件基础，目标是在运动性能上实现显著升级：复杂地形下的稳健行走、小跑至中速连续奔跑、以及跳跃等高阶动态运动。上述目标对底层通信提出了**强实时、低抖动、高可靠**的硬性要求，控制指令传输与执行器/传感器数据回传是整个控制链路的物理基础。

在 X1 阶段，项目已在 `agibot_x1_infer` 上完成 1kHz EtherCAT 主干通信搭建：

- 主控 + 双 DCU 分层架构，DCU 作为协议转换节点，将 EtherCAT 指令转换为 CAN-FD 并分发至各执行单元；
- 主控制循环周期 1ms，实际运行时间 870–920μs，周期抖动控制在 ±50μs 以内；
- 分布式时钟（DC）同步配置完成，实现多关节协同的统一时间基准；
- 31 自由度电机标定、双 IMU 标定、零位/站立位整机控制验证均已打通。

当前 `agibot_x1_infer` 实机控制链路基于 `DcuDriverModule + XyberController + SOEM`，默认拓扑为：

- `body` DCU（上肢 + 腰部左右推杆）
- `hip` DCU（双腿 + 腰部 yaw + IMU）

### 1.2 架构升级动因

原版 DCU 在通信协议、传感器接口、可观测性等方面存在黑盒限制（详见自研 DCU 技术方案），且 X1 平台无法实现连续奔跑。F1 硬件构型升级（髋部电机布局重构、关节电机更换等）要求通信架构同步演进：

| 维度 | X1 原版方案 | F1 目标方案 |
|------|-------------|-------------|
| 上肢执行器 | 智元 PowerFlow（CANFD） | 保留，继续经 DCU 驱动 |
| 下肢/腰部执行器 | 智元 PowerFlow（CANFD） | 更换为因克斯（Encos/encos）电机 |
| 下肢/腰部协议转换 | `hip` DCU | Encos EtherCAT 开发板（SSC-Device） |
| IMU | `hip` DCU 内置 | 需独立方案（Encos 板无 IMU PDO） |
| 主站协议栈 | SOEM | SOEM（后续可评估 IGH Master） |

### 1.3 融合目标

在**同一条 EtherCAT 总线**上实现异构从站共存，完成双协议转发链路打通：

1. **保留** `body` DCU（上肢继续使用原 PowerFlow 路径）；
2. **替换** `hip` DCU：下肢与腰部改用 Encos EtherCAT 开发板（对齐 `soem_demo_cpp` 协议栈）；
3. **替换** 下肢与腰部电机为 Encos 电机（EC-A8112 / EC-A4314 等）；
4. **统一** 上层控制语义：`/joint_cmd`、`/joint_states` 接口不变，执行器层统一暴露 `SetMitCmd()` / `GetPosition()` / `GetVelocity()` / `GetEffort()`；
5. **满足** F1 实时指标：1kHz 控制频率、DC 同步、周期抖动 ≤ ±50μs、WKC 稳定。

---

## 2. 结论概览

### 2.1 可行性

该方案可行，已在代码层完成核心适配（`EncosSlave` + `EncosMotor`）。SOEM 允许同一总线挂载不同 PDO 大小的从站，前提是：

1. 每个从站 PDO 映射正确（含 encos 从站 SDOCA 规避，见 `doc/ethercat_sdoca_issue.md`）；
2. 主站软件对不同从站类型使用不同数据结构（`DcuSendPacket` 240B vs `EncosSlavePacket` 86B）；
3. 上层控制语义（`position/velocity/effort`）经适配层统一；
4. 实时环境满足 PREEMPT_RT 内核 + CPU 绑核 + DC 同步要求。

### 2.2 拓扑建议

推荐 EtherCAT 从站排列（分步替换策略：先替换一个 DCU，保留另一个原版 DCU 确保整机功能不中断）：

| ecat_id | 从站名 | 类型 | 挂载内容 | 说明 |
|---------|--------|------|----------|------|
| 1 | `body` | DCU（智元） | 上肢 PowerFlow 执行器 | 保留原版，3 路 CANFD |
| 2 | `waist` | Encos 开发板 | 腰部 yaw 等 1 路电机 | 替换原 `hip` DCU 腰部通道 |
| 3 | `left_leg` | Encos 开发板 | 左腿 6 路电机 | CAN1(1~3) + CAN2(4~6) |
| 4 | `right_leg` | Encos 开发板 | 右腿 6 路电机 | 左右腿分板，规避单板容量风险 |

> **替换策略**：与自研 DCU 方案一致，采用**分步替换**——第一阶段仅将 `hip` DCU 替换为 Encos 开发板组，保留 `body` DCU；验证通过后再推进自研 DCU 对 `body` DCU 的逐步替代。

---

## 3. 两套从站协议差异

### 3.1 DCU 路径（智元 PowerFlow，现有）

DCU 作为域控制器（Domain Control Unit），以 1kHz 频率经 EtherCAT 与主站通信，通过 3 路 CANFD（5Mbps）转发至下游 PowerFlow 执行器。

**EtherCAT PDO 结构**：

- 下发包：`DcuSendPacket`（240B）
  - `canfd[3].{ctrl, data[64]}` — 每路 CANFD 广播 1 帧，Payload 64B，各设备以 CAN ID 为 offset 读取 8B 控制指令
  - `imu_cmd` — IMU 校准命令
- 反馈包：`DcuRecvPacket`（240B）
  - `canfd[3][64]` — 各设备独立回传状态帧
  - `imu{acc, gyro, quat}` — IMU 传感器数据

**执行器协议**：PowerFlow MIT 打包（R86/R52/L28/夹爪）。

**原版协议特征**（自研 DCU 改进方向参考）：

- 广播模式（Cmd=0xFF），单帧携带全部设备指令，无从站掉线感知；
- 上下行 PDO 固定 240B，不支持手动映射；
- 上行无帧序列号/时间戳，主站无法判断数据新鲜度。

### 3.2 Encos 开发板路径（因克斯 encos，目标）

Encos EtherCAT 开发板基于 SSC（EtherCAT Slave Stack Code）+ ESC 芯片（AX58100 / LAN9252），协议对齐 `soem_demo_cpp`。

**EtherCAT PDO 结构**（`EncosSlavePacket`，有效 86B；PDO 可能更大，主站仅读写前 86B）：

- `motor_num` — 有效电机数量
- `can_ide` — CAN 帧类型（MIT 力位混合需扩展帧，`can_ide=1`）
- `motor[6]` — 每路 `Motor_Msg{id(4B), rtr(1B), dlc(1B), data[8](8B)}`

**板端转发逻辑**：

- ESC（FMMU/SM）取 PDO → MCU 转发至 CAN1（passage 1~3）/ CAN2（passage 4~6）；
- 每板最多 6 路电机，左右腿分板。

**执行器协议**：Encos CAN 协议（位置/速度/电流/力位混合 MIT），编解码参考 `soem_demo_cpp/app_cpp/motor_control.c`。

**电机型号**：

| 型号 | 用途示例 |
|------|----------|
| `ENCOS_EC_A8112`（EC-A8112-P1-18H） | 髋、膝等大关节 |
| `ENCOS_EC_A4314`（EC-A4314-P2-36） | 踝、腰等小关节 |

### 3.3 双协议对比

| 维度 | DCU / PowerFlow | Encos 开发板 / Encos 电机 |
|------|-----------------|----------------------|
| PDO 大小 | 固定 240B | 有效 86B（PDO 可能 186/192B） |
| 下游总线 | CANFD（5Mbps） | CAN |
| 通道模型 | 3 路 × 8 执行器 | 6 槽位（passage 1~6） |
| 寻址 | `can_id` + `CtrlChannel` | `passage` + `motor_id` |
| 控制帧 | PowerFlow MIT 位域 | Encos MIT（16bit pos + 12bit vel/cur） |
| IMU | 板载 PDO | 无 |
| 使能流程 | PowerFlow `RequestState` | MIT 零力矩保持 |

---

## 4. 架构融合设计

### 4.1 设计原则

1. **上层接口不变**：`/joint_cmd`、`/joint_states`、`/imu/data` 话题语义保持力位混合控制；
2. **transmission 层复用**：关节空间 ↔ 执行器空间转换逻辑不变，按新执行器名重绑定；
3. **后端按从站类型分流**：YAML 配置 `type: dcu` 与 `type: encos` 决定创建路径；
4. **执行器语义统一**：所有执行器统一对外提供 `SetMitCmd()`、`GetPosition()`、`GetVelocity()`、`GetEffort()`；
5. **分步替换、兼容优先**：先合网验证，再逐步扩大 Encos 覆盖范围，避免主从站同时调试引入干扰；
6. **可观测性延续**：继承 X1 阶段 TAP 旁路监听、Wireshark 协议解析、CAN 分析仪等调试能力，覆盖异构从站场景。

### 4.2 推荐软件分层

```text
JoyStickModule / ControlModule
          |
          |  /joint_cmd
          v
DcuDriverModule + transmission（关节空间 ↔ 执行器空间）
          |
          v
XyberController（统一执行器 API，按名称路由）
   |                              |
   v                              v
Dcu（EthercatNode）            EncosSlave（EthercatNode）
DcuSendPacket 240B             EncosSlavePacket 86B
PowerFlow Actuator             EncosMotor Actuator
   |                              |
   +-------- EthercatManager + SOEM（1kHz DC 同步）--------+
                              |
                              v
                    EtherCAT 物理总线（异构从站共存）
```

### 4.3 关键点

- `Dcu` 与 `EncosSlave` 均继承 `EthercatNode`，各自实现 `PullSendData` / `PushRecvData`；
- `EthercatManager` 保持统一 1kHz 收发循环，**不做固定 PDO 长度假设**，按各从站实际 `Obytes`/`Ibytes` 调用；
- `EncosSlave` 内部仅读写 `sizeof(EncosSlavePacket)=86B`，PDO 超出部分忽略，避免缓冲区越界；
- `ec_config_map()` 前清除 `ECT_COEDET_SDOCA` 位，规避 encos 从站 SAFE_OP 失败（AL 0x001E）；
- `DcuDriverModule::InitDcu()` 通过执行器名称映射到底层不同节点，`PublishLoop` 统一调用 `GetPosition/Velocity/Effort`；
- `XyberController` 对 `EnableActuator`、`SetMitCmd`、`ClearError` 等 API 按 `actuator_dcu_map_` / `actuator_encos_map_` 自动路由。

### 4.4 实时性要求（对齐 F1 整机验证指标）

| 指标 | 目标值 | 实现手段 |
|------|--------|----------|
| 控制周期 | 1ms（1kHz） | `cycle_time_ns: 1000000` |
| 循环执行时间 | 870–920μs | PREEMPT_RT + CPU 绑核 |
| 周期抖动 | ≤ ±50μs | DC 同步 + PI 调节 |
| WKC | 无持续异常 | `EthercatManager` 错误处理线程监控 |
| 观测侵入性 | 零延迟、零干扰 | EtherCAT TAP 旁路 + Wireshark |

---

## 5. 需要修改的代码范围

### 5.1 `xyber_controller`（核心，已实现）

**新增**（代码中类名为 `EncosSlave` / `EncosMotor`，对应原方案中的 Demo 从站概念）：

1. **`EncosSlave`**（`EthercatNode` 子类，`encos_slave.h/cpp`）
   - 发送/接收缓存：`EncosSlavePacket send_buf_` / `recv_buf_`
   - `Init()` 校验 PDO ≥ 86B，超出部分在 `Pull/PushRecvData` 中截断
   - `RegisterActuator(actr, passage)` — passage 1~6 映射至 `motor[0..5]` 槽位
   - `SetCanIde()` — 配置 CAN 扩展帧/标准帧

2. **`EncosMotor`**（`Actuator` 子类，`encos_motor.h/cpp`）
   - 绑定字段：`passage(1~6)`、`motor_id`
   - `SetMitCmd()` — 映射至 Encos 力位混合 CAN 帧编码
   - `UpdateFeedback()` — 从 `Motor_Msg` 解包 pos/vel/effort
   - 使能/禁用独立逻辑，不复用 PowerFlow `RequestState`

3. **`XyberController` API 扩展**（`xyber_controller.h/cpp`）
   - `CreateEncosSlave(name, ecat_id)`
   - `AttachEncosMotor(slave_name, passage, type, actuator_name, motor_id)`
   - `SetEncosCanIde(slave_name, can_ide)`
   - 统一 API 路由：`SetMitCmd` / `GetPosition` / `EnableActuator` / `ClearError` 等按执行器名自动分发至 DCU 或 Encos 路径

**保留**：

- 现有 `CreateDcu` / `AttachActuator` 全链路不变（用于 `body` DCU）

**`EthercatManager` 修复**：

- `ec_config_init()` 后清除所有从站 `ECT_COEDET_SDOCA` 位（`ethercat_manager.cpp`）

### 5.2 `dcu_driver_module`（已实现）

1. `InitDcu()` 支持两类从站配置：
   - `type: dcu`（默认）→ `CreateDcu()` + `AttachActuator()`，读取 `channel_1/2/3`
   - `type: encos` → `CreateEncosSlave()` + `AttachEncosMotor()`，读取 `motors[]`
2. `config_parse.h` 扩展 `DcuConfig` 结构，支持 `type`、`passage`、`motor_id`、`can_ide` 字段；
3. 发布循环保持统一读取 `GetPosition/Velocity/Effort`，发布 `/joint_states`；
4. 使能/清错流程对 Encos 执行器走独立路径；
5. Encos 从站跳过 IMU 初始化（`imu_enable` 仅对 DCU 生效）。

### 5.3 YAML 配置（`cfg/dcu_x1.yaml`）

建议变更：

1. **删除** `hip` DCU（下肢 + IMU 不再经此路径）；
2. **`body` 保留**上肢执行器，腰部并联推杆执行器迁出；
3. **新增**三个 Encos 从站：`waist`、`left_leg`、`right_leg`；
4. **新增**每路 Encos 电机映射：`passage`、`motor_id`、`can_ide`；
5. **`actuator_list`** 按实际启用执行器更新；
6. **`transmission`** 按新执行器名重绑定；
7. **`imu_dcu_name`** 待 IMU 独立方案确定后更新。

**DCU 从站配置示例**：

```yaml
- name: body
  ecat_id: 1
  enable: true
  type: dcu
  imu_enable: false
  channel_1:
    - name: left_shoulder_yaw_actuator
      type: POWER_FLOW_R52
      can_id: 1
```

**Encos 从站配置示例**：

```yaml
- name: waist
  ecat_id: 2
  enable: true
  type: encos
  can_ide: 1
  imu_enable: false
  motors:
    - name: lumbar_yaw_actuator
      type: ENCOS_EC_A4314
      passage: 1
      motor_id: 1
```

---

## 6. IMU 处理

X1 阶段下肢 IMU 经 `hip` DCU 的 UART（921600bps）接入，数据打包进 EtherCAT 上行 PDO 发布至 `/imu/data`。Encos 开发板当前**不包含 IMU PDO**，迁移后不能继续依赖 `hip` DCU 的 IMU。

可选方案：

| 方案 | 说明 | 优先级 |
|------|------|--------|
| 独立 IMU 模块 | 新增 AimRT 模块，独立采集并发布 `/imu/data` | **推荐** |
| 扩展 Encos 板固件 PDO | 在 `EncosSlavePacket` 中增加 IMU 字段，主站同步扩展解析 | 中期 |
| 临时降级 | 算法允许无 IMU 条件下运行，仅做静态/慢速验证 | 临时 |

> F1 整机验证阶段已完成双 IMU 安装姿态标定、采样率优化与滤波算法调优，IMU 独立方案应复用已有标定流程与算法成果。

---

## 7. 返回值结构与上层影响评估

底层协议结构不同（PowerFlow 位域 vs Encos MIT 反馈帧），但适配层统一输出 `position(rad)` / `velocity(rad/s)` / `effort(Nm)`，上层 `ControlModule` 与 transmission 层**基本不受影响**。

需要重点校准：

| 校准项 | DCU / PowerFlow | Encos 电机 | 说明 |
|--------|-----------------|----------|------|
| 位置范围 | ±2π rad | ±12.5 rad | `ENCOS_MIT_MODE_DEFAULT_PARAM` |
| 速度范围 | ±4π rad/s | ±18 rad/s | Encos 内部 rpm↔rad/s 已转换 |
| 力矩范围 | ±100 Nm（R86） | ±30 Nm | 需按实际电机限幅 |
| Kp / Kd | 0~500 / 0~8 | 0~500 / 0~5 | 控制器下发前限幅 |
| 正方向与零点 | 按 X1 标定 | 需重新标零 | 31 DOF 个体化标零流程可复用 |
| 刷新时序 | 1kHz 周期一致 | 1kHz 周期一致 | 同一 `WorkLoop` 内收发 |

---

## 8. 分阶段实施计划

本计划与自研 DCU 技术方案的阶段一（能力建设）及控制系统整机验证流程对齐，采用**隔离变量、分步替换**策略。

### 阶段 0：硬件与协议确认

- 确认 Encos 开发板接线、`ecat_id` 排列与 SSC 固件版本；
- 确认 Encos 电机协议、反馈模式（MIT ack_status）、CAN 波特率；
- 明确 IMU 独立方案选型；
- 搭建实时观测环境：EtherCAT TAP + Wireshark + CAN 分析仪。

### 阶段 1：最小链路打通（单 Encos 板 + 单电机）

- 仅接 1 块 Encos 开发板 + 1 个电机；
- 验证 `EncosSlave` PDO 收发、`SetMitCmd` 下发与 `GetPosition/Velocity/Effort` 反馈；
- 验证 encos 从站进 OP（SDOCA 修复生效）；
- 里程碑：SOEM 连接 Encos 从站，1kHz DC 同步稳定运行。

### 阶段 2：单腿打通（6 电机）

- 接入左腿 6 电机，验证 6 路 `passage` 无串扰；
- 联调 transmission 与关节映射（含踝并联传动）；
- 验证 `/joint_states` 连续性与噪声水平；
- 完成 Encos 电机电流/速度/位置各控制环参数标定。

### 阶段 3：全下肢 + 腰（3 块 Encos 板合网）

- 接入 `waist + left_leg + right_leg`；
- 联调 enable/stop/fault 流程；
- 做全身静态动作与小幅步态测试；
- 31 自由度个体化标零与正反转全行程验证。

### 阶段 4：与 body DCU 异构合网

- 接回 `body` DCU，四从站（1 DCU + 3 Encos）同网运行；
- 验证异构从站 WKC、周期抖动（目标 ≤ ±50μs）、丢包率；
- 复现 X1 零位/站立位控制验证，确认上下肢协同；
- 里程碑：上肢（DCU）与下肢（Encos）可同时稳定 1kHz MIT 控制。

### 阶段 5：整机运动验证（对齐 F1 控制目标）

- 手柄控制零位/站立位平滑切换；
- 引入 MPC/WBC 算法，验证抗扰动动态行走；
- 单腿跳跃台架实验，摸索滞空姿态估计与落地力控；
- 为奔跑姿态预研积累数据。

---

## 9. 风险与规避

| 风险 | 影响 | 规避措施 |
|------|------|----------|
| **容量风险**：双腿单板超限 | 6 路以上电机无法单板上齐 | 左右腿分板（各 6 路），腰部独立单板 |
| **协议风险**：MIT 参数语义不一致 | 控制发散或饱和 | 建立统一参数标定表与限幅（§7） |
| **时序风险**：异构 PDO 导致抖动增大 | 影响高动态运动 | PREEMPT_RT + 绑核 + DC 同步 + WKC 监控 |
| **观测风险**：IMU 缺失 | 姿态估计退化 | 优先补齐独立 IMU 链路（§6） |
| **兼容性风险**：encos PDO 映射异常 | 从站无法进 OP | SDOCA 清除 + 普通 SDO 读映射 |
| **替换风险**：合网期间功能中断 | 影响整机调试进度 | 分步替换，保留 `body` DCU 直至验证通过 |
| **黑盒风险**：原版 DCU 无从站掉线感知 | 安全隐患 | 中期推进自研 DCU（帧序列号 + emergency 上报） |

---

## 10. 验收标准（建议）

### 10.1 底层通信

1. 全部注册从站长时间保持 EtherCAT OP，无掉站；
2. 1kHz DC 同步下 WKC 无持续异常；
3. 主控制循环执行时间 870–920μs，周期抖动 ≤ ±50μs；
4. Encos 从站 6 路 `passage` 无串扰，DCU 3 路 CANFD 通道正常。

### 10.2 控制与状态

5. `/joint_states` 与实际动作一致，量纲正确（rad / rad/s / Nm）；
6. `SetMitCmd` 对 DCU 与 Encos 执行器均有效，力位混合控制闭环稳定；
7. 停止/急停/失联保护可触发，`DisableAllActuator` 正常；
8. 上肢（DCU）与下肢（Encos）可同时稳定运行。

### 10.3 整机验证

9. 31 自由度标零完成，机械零位与电气零位对齐；
10. 零位/站立位手柄控制平滑切换，无抖动；
11. IMU 数据正常发布（独立方案上线后）；
12. Wireshark / CAN 分析仪可非侵入式抓取并解析双协议报文。

---

## 11. 备注

1. 本方案描述 `agibot_x1_infer` 侧软件改造，与 F1 自研 DCU 硬件方案（SSC 从站 + ESC 芯片 + CANFD 协议改进）形成**软硬件协同演进**关系：当前 Encos 开发板验证异构合网能力，后续自研 DCU 可逐步替换 `body` DCU。
2. 若 Encos 板固件 PDO 或 CAN 协议变更，需同步更新 `EncosSlave` / `EncosMotor` 适配层。
3. 代码中 `EncosSlave`/`EncosMotor` 对应本方案早期文档中的 `DemoSlave`/`DemoMotorActuator` 命名，YAML 配置字段为 `type: encos`。
4. 自研 DCU 改进项（帧序列号、emergency 上报、设备级状态字、调试接口）在 `body` DCU 替换阶段一并落地，本方案当前阶段聚焦**双协议异构合网**与**底层链路打通**。
