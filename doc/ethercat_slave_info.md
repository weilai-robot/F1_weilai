# EtherCAT 从站信息汇总

> 拓扑：master → slave[1] → slave[2]（slave[1] 离主机网卡最近）。
> 物理对应：**slave[1] = body（DCU，EasyCAT-AX58100 开发板）**，**slave[2] = waist（encos 板，SSC-Device 开发板）**。
> 数据来源：`EthercatManager::InitSlaveNodes()` 中对 `ec_slave[]` 的扫描 + SDO 读 `0x1C12/0x1C13`。

> 判定依据（代码侧）：
> - `Dcu::Init()`（`dcu.cpp:37`）对 `send_size_/recv_size_` 做**严格 `== sizeof(DcuSendPacket/DcuRecvPacket)=240`** 校验，只有 PDO 恰好 240/240 的从站才能通过 → 即 slave[1]（AX58100，240/240）。`example/main.cpp:26-28` 也以 `ethercat_id=1` 创建 DCU 并命名 "body"。
> - `EncosSlave::Init()`（`encos_slave.cpp:37`）做宽松 `>= sizeof(EncosSlavePacket)=86` 校验，`Pull/Push` 截断到 86B，只要 PDO ≥ 86 即可 → 即 slave[2]（SSC-Device，187/193）。
> - `encos_slave.h:36` 注释明确 "PDO layout matches encos board firmware (86B)"，`EncosSlave` 即 encos 板驱动。

---

## 1. Host 端结构（基准）

| 驱动结构 | 字节数 | 含义 | 绑定从站 | static_assert |
|---|---|---|---|---|
| `DcuSendPacket`  | **240 B** | body 发送：`3×(1+64) canfd + 1 imu_cmd + 44 reserved` | slave[1] (AX58100) | — |
| `DcuRecvPacket`  | **240 B** | body 接收：`3×64 canfd + (3+3+4)×4 imu + 8 reserved` | slave[1] (AX58100) | — |
| `EncosSlavePacket` | **86 B**  | waist：`motor_num(1) + can_ide(1) + 6×Motor_Msg(14)` | slave[2] (SSC-Device) | ✅ `==86` |

> `Motor_Msg`（14 B）= `id(4) + rtr(1) + dlc(1) + data[8]`，见 `encos_slave.h:29`。`motor[0..2]` 自动路由到 CAN1，`motor[3..5]` 到 CAN2（`encos_slave.h:37`）。

> host 侧 `Dcu` / `EncosSlave` 均未覆写 `GetPreOpConfigFunc()`（返回 `nullptr`），不做 PRE_OP→SAFE_OP 的 PDO 重映射，两块板的 PDO 完全由各自 EEPROM/固件决定。

---

## 2. slave[1] —— body（DCU / EasyCAT-AX58100）

### 2.1 身份
| 字段 | 值 |
|---|---|
| name | `EasyCAT-AX58100` |
| configadr | 0x1001 |
| vendor (eep_man) | 0x00000001 |
| product (eep_id) | 0x00058100 |
| rev (eep_rev) | 0x00020101 |
| Itype / Dtype | 1541 / 0 |

### 2.2 板端上报的 PDO 尺寸
| | bits | bytes |
|---|---|---|
| Output | 1920 | **240** |
| Input  | 1920 | **240** |

SM/FMMU：
- SM[2] @0x1100 len=240 type=Outputs；SM[3] @0x1400 len=240 type=Inputs。
- FMMU 把 SM[2]/SM[3] 映射进 process data。
- 邮箱：wr/rd 各 128B @0x1000/0x1080，proto=0x0004（CoE）。DC：hasdc=1，DCactive=0。

> 板端 EEPROM 的 `0x1600` 默认列出的是 EasyCAT 通用模板（若干 112-bit 通道），并非生效映射；生效的 240B PDO 由 `0x1C12/0x1C13` 选定，恰好等于 `DcuSendPacket/DcuRecvPacket`。

### 2.3 生效的内存结构（240B 全部有效，严格匹配）

`Dcu::Init()` 要求 `send_size_==sizeof(DcuSendPacket)==240` 且 `recv_size_==sizeof(DcuRecvPacket)==240`（`dcu.cpp:39-48`）。slave[1] 上报 240/240，**精确相等**，无需任何补丁，整段 PDO 即 DCU 协议帧。

**DcuSendPacket（host→slave，240B）**：

| 偏移 | 字节 | 内容 |
|---|---|---|
| 0..64    | 65 | `canfd[0]`：`ctrl(1)+data[64]`（CAN-FD 通道 1） |
| 65..129  | 65 | `canfd[1]`：CAN-FD 通道 2 |
| 130..194 | 65 | `canfd[2]`：CAN-FD 通道 3 |
| 195       | 1  | `imu_cmd` |
| 196..239 | 44 | `reserved[44]` |

**DcuRecvPacket（slave→host，240B）**：

| 偏移 | 字节 | 内容 |
|---|---|---|
| 0..63    | 64 | `canfd[0][64]` |
| 64..127  | 64 | `canfd[1][64]` |
| 128..191 | 64 | `canfd[2][64]` |
| 192..231 | 40 | `imu`：`acc[3]×4 + gyro[3]×4 + quat[4]×4` |
| 232..239 | 8  | `reserved[8]` |

> DCU 板完整消费 240B 输入/输出，是两块板中布局最规整的一个。

---

## 3. slave[2] —— waist（encos 板 / SSC-Device）

### 3.1 身份
| 字段 | 值 |
|---|---|
| name | `SSC-Device` |
| configadr | 0x1002 |
| vendor (eep_man) | 0x00000009 |
| product (eep_id) | 0x26483052 |
| rev (eep_rev) | 0x00020111 |
| Itype / Dtype | 8 / 0 |

### 3.2 板端上报的 PDO 尺寸
| | bits | bytes |
|---|---|---|
| Output | 1492 | **187** |
| Input  | 1540 | **193** |

SM/FMMU：
- SM[2] @0x1100 len=187 type=Outputs；SM[3] @0x1400 len=193 type=Inputs。
- parent=1（接在 slave[1] 之后）。

### 3.3 真正生效的内存结构（187/193B 中只有前 86B 有意义）

encos 板的 PDO 槽宽 187/193B，但**板载固件实际只消费 PDO 偏移 0..85**，把它解释为 `EncosSlavePacket`（与 `soem_demo_cpp` 的 `EtherCAT_Msg` 逐字节一致）。`EncosSlave::Init()` 的宽松校验 + 截断拷贝即迁就这一现状。

`EncosSlavePacket`（86B，host 与板端共享的前 86B 语义）：

| 偏移 | 字节 | 内容 | 来源 |
|---|---|---|---|
| 0 | 1 | `motor_num` | 写入：`send_buf_.motor_num` |
| 1 | 1 | `can_ide`（0=标准帧，1=扩展帧） | `send_buf_.can_ide` |
| 2..15  | 14 | `motor[0]`：`id(4)+rtr(1)+dlc(1)+data[8]` | CAN1 通道 1 |
| 16..29 | 14 | `motor[1]` | CAN1 通道 2 |
| 30..43 | 14 | `motor[2]` | CAN1 通道 3 |
| 44..57 | 14 | `motor[3]` | CAN2 通道 4 |
| 58..71 | 14 | `motor[4]` | CAN2 通道 5 |
| 72..85 | 14 | `motor[5]` | CAN2 通道 6 |
| 86..186 (Out) / 86..192 (In) | 101 / 107 | **板端 PDO 填充，固件不读** | 忽略 |

### 3.4 encos PDO 补丁（host 侧）

由于板端 PDO(187/193B) > 结构(86B)，原 `EncosSlave::Init()` 若用 `sizeof(EncosSlavePacket) != send_size_` 严格相等校验会误判失败，且基类 `PullSendData/PushRecvData` 按 `Obytes=187` 拷贝会越界读写 86B 的 `send_buf_/recv_buf_`。补丁对齐 `soem_demo_cpp` 的"只用前 86B"做法：

- **`encos_slave.h`**：新增 `PullSendData`/`PushRecvData` override 声明。
- **`encos_slave.cpp` `Init()`**：`!=` 改为 `>=`——只要 `send_size_/recv_size_ >= sizeof(EncosSlavePacket)` 即通过；大于部分记一条 INFO 并忽略。
- **`encos_slave.cpp` `PullSendData/PushRecvData`**：拷贝字节数取 `min(len, sizeof(EncosSlavePacket))`，即只搬前 86B，避免越界。等价于 demo 的 `memcpy(outputs, &Tx, sizeof(EtherCAT_Msg))`，但带下界防护（板子若报 <86B 直接 `Init()` 失败，不会盲写）。

补丁定位：
- `src/module/dcu_driver_module/xyber_controller/xyber_api/include/internal/encos_slave.h`（override 声明）
- `src/module/dcu_driver_module/xyber_controller/xyber_api/src/encos_slave.cpp`（`Init()` 放宽 + `Pull/Push` 截断）

> 合规性：PDO 取子集在 EtherCAT 协议上合法；板载固件已定义偏移 0..85 的语义，demo 已验证可行。正道仍是把板端 PDO 配成 86B 使 `Obytes==86`，当前补丁为迁就板端 187/193B 默认 PDO 的容错措施。

---

## 4. 相关代码与参考

| 位置 | 内容 |
|---|---|
| `src/.../xyber_api/src/dcu.cpp:37` | `Dcu::Init()` 严格 `==240` 校验（对应 slave[1]=AX58100） |
| `src/.../xyber_api/include/internal/dcu.h:23-40` | `DcuSendPacket/DcuRecvPacket`(240B) 结构 |
| `src/.../xyber_api/src/encos_slave.cpp:37` | `EncosSlave::Init()` 放宽 `>=86` 校验 + `Pull/Push` 截断（对应 slave[2]=SSC-Device） |
| `src/.../xyber_api/include/internal/encos_slave.h:38-42` | `EncosSlavePacket`(86B) 结构、override 声明 |
| `src/.../xyber_api/src/ethercat_manager.cpp` | `DumpSlaveDetail()` / `InitSlaveNodes()` 诊断 |
| `src/.../xyber_controller/example/main.cpp:26-28` | DCU 以 `ethercat_id=1` 创建并命名 "body" |
| `soem_demo_cpp/app_cpp/config.h:18-32` | `EtherCAT_Msg`(86B)，与 `EncosSlavePacket` 一致 |
| `soem_demo_cpp/app_cpp/transmit.cpp:282/340/387` | demo 只读写前 86B，佐证 encos 板端语义 |
