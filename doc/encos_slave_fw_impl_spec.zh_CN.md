# ENCOS EtherCAT 从站固件实现需求规格（指导代码生成）

> **定位**：本文每条需求都可直接转成代码/配置，含时间预算、中断清单、EtherCAT→CAN 转换算法、字节级 PDO 布局、对象字典表。需求编号 **R-xxx**，括号内追溯原始固件需求 FW-xxx。
> **硬件**：EC-EtherCAT-IMU 板（HW-ELEC-0006）：STM32F407ZET6 + AX58100 + 24LC64 + 2×TJA1050（经典 CAN）。
> **主站**：本仓库 SOEM，1 kHz、DC SYNC0 1 ms 相位 0、只读写 PDO 前 86 B。
> **范围**：不含 IMU（PDO 布局已预留）。已拍板：D-1 PDO 86 B 前缀冻结+扩展块；D-2 can_ide 每槽位；D-5 安全默认 MIT 零力矩；D-8 TxPDO 尾部预留 40 B IMU 区。**PDO 尺寸冻结：RxPDO(SM2)=90 B、TxPDO(SM3)=150 B（§6）。**
> **本版固化的实现假设**（如不同意请指出，影响对应条目）：A1 裸机无 RTOS；A2 PDI 用 FSMC 16 位并行（原理图已布线，SPI2 为备选）；A3 下行为逐周期透传（demo 兼容）。

---

## 1. 运行环境与硬件资源分配

**R-101 时钟树**：HSE 8 MHz → PLL → SYSCLK 168 MHz；APB1 = 42 MHz（CAN1/CAN2、TIM2-7）；APB2 = 84 MHz（USART1、TIM10）。TIM 时钟×2（APB 定时器倍频规则）。

**R-102 ESC 访问**：AX58100 映射为 FSMC 16 位存储器窗口，基址 `FSMC_NE4 = 0x6C000000`（NE4 区，STM32F4 存储映射）。引脚级映射**已按 HW-ELEC-0006 03/04 引脚分册逐线核对完毕**：

| 信号 | STM32 侧（FSMC 复用） | ESC 侧（本地总线功能） | 备注 |
|---|---|---|---|
| 片选 SCS_ESC | PG12 = FSMC_NE4 | U2-79 | 与 PB12(SPI2_NSS) 双布线，只允许一侧输出使能 |
| 读 LRDn | PD4 = FSMC_NOE | U2-64 | 与 PB13(SPI2_SCK) 双布线 |
| 写 LWRn | PD5 = FSMC_NWE | U2-78 | 与 PB15(SPI2_MOSI) 双布线 |
| 就绪 LRDY | PB14（GPIO，**无 FSMC 复用**） | U2-6 (LAT_IN/LRDY) | F407 硬件 NWAIT=PD6 且未连接 → 只能 GPIO 轮询或不用，靠时序余量；语义按 AX58100 手册冻结 |
| 数据 LDA[15:0] | FSMC_D[15:0]：PD14/15、PD0/1、PE7~15、PD8/9/10 | IO[0..15] | **按位对齐 D[n]↔LDA[n]**，16 位全核对 |
| 地址 LA[13:1] | FSMC_A[0..12]：PF0~5、PF12~15、PG0~2 | IO[8..21] | **错一位是刻意的**：16 位模式下 FSMC 外部 A0 = 字节地址 bit1 |
| LA[0] | 不接 MCU，经 R40 电阻固定 | U2-71 | 字节选固定 0，16 位全字访问 |
| LBHE | 不接 MCU，经 R41 电阻固定 | U2-68 | 高字节使能固定，全字访问 |
| （PD11/12/13 = FSMC_A16~18 未连接） | — | — | 地址线只需 A0~A12 |

访问模型：ESC 寄存器（16 位对齐）字节偏移 X ↔ `*(volatile uint16_t*)(0x6C000000 + X)`。FSMC 时序参数初值：地址建立 3 HCLK、数据保持 4 HCLK（**建议初值**，联调按信号质量收紧；若不用 LRDY 需覆盖 ESC DPRAM 访问时间并留余量）。单次 16 位访问 ≤ 60 ns 量级（估算），读 90 B PDO ≈ 45 次访问 ≤ 5 µs。

**R-103 ESC 中断接入**：ESC `LINT`（U2-1）→ **PC0 / EXTI0**，边沿触发；`SYNC_LATCH[0/1]`（U2-2/3 → PC3/PC1）可作交叉校验输入（本期不用）。

**R-104 CAN 接入**：CAN1 = PA11/PA12（槽位 1~3），CAN2 = PB5/PB6（槽位 4~6）；TJA1050 高速模式（S 脚接地，无需软件控制）；各路 120 Ω 已端接。F407 CAN2 为 CAN1 从属：**CAN1 必须先使能**，滤波器组默认 CAN1 用 0~13、CAN2 用 14~27（CAN1_FMR 配置）。

**R-105 其他外设分配**：USART1（PA9/PA10，CH340N，日志，DMA 发送）；IWDG（LSI 32 kHz）；ADC 待定（D-7：电压/板温采样通道硬件未定）；LED1/LED2 = PF7/PF8；PF6=IMU 加热（本期不用，初始化为输入浮空）；SWD = PA13/PA14。

**R-106 Flash 分区**（512 KB，按 F407 扇区边界）：

| 区 | 扇区 | 地址 | 大小 | 用途 |
|---|---|---|---|---|
| Boot | 0,1 | 0x0800_0000 | 32 KB | bootloader（FoE 接收+校验+跳转+回退） |
| Param | 2 | 0x0800_8000 | 16 KB | 参数镜像 + CRC32 + 版本号 |
| AppA（运行区） | 3,4,5 | 0x0800_C000 | 208 KB | 当前固件 |
| AppB（升级区） | 6,7 | 0x0801_4000 | 256 KB | FoE 写入区；App 二进制 ≤ 208 KB |

**R-107 RAM 预算**（192 KB）：SSC 栈+缓冲+mailbox ≈ 16 KB；应用数据/队列/统计 ≈ 8 KB；日志环形缓冲 4 KB；主栈 4 KB；中断栈共用主栈。合计 < 64 KB，余量给后续 IMU。

---

## 2. 实时指标与时间预算（回答"1 kHz 下代码运行时间应是多少"）

**R-201 周期时间轴**（全部锚定 SYNC0，主站已配 `ec_dcsync0(id,TRUE,1ms,0)`）：

| 阶段 | 预算 | 追溯 | 说明 |
|---|---|---|---|
| SYNC0 事件 → EXTI ISR 入口 | ≤ 2 µs | 设计 | 最高抢占优先级保证 |
| ISR 内：读 SM2(90 B) + 解析 + 6 帧全部入队 | **≤ 100 µs 硬上限，典型 ≤ 20 µs** | FW-CAN-01 | R-601 算法 |
| 入队 → CAN 首帧上总线 | ≤ 150 µs | 推导 | 首帧直接装邮箱启动 |
| CAN 指令突发（每总线 3×扩展帧 8 B @1 Mbps ≈ 131~160 µs/帧 + 2×50 µs 帧间隔） | ≤ 580 µs | 板规格 | TIM 节奏出队，CPU 不忙等 |
| 电机应答 | 最快 ~0.4 ms（问答往返） | HW-SPEC-0005 §7 | 与后续指令流水重叠 |
| 反馈全部锁存进 TxPDO | ≤ SYNC0 + 900 µs | 设计 | 10% 余量 |
| SYNC0 → 入队 峰峰抖动 | **≤ 20 µs** | FW-ECT-02 | 由 R-301 中断优先级与 R-202 临界区规则保证 |

**R-202 关中断临界区规则**：全系统任何 `__disable_irq()` 临界区 ≤ 20 µs；CAN RX 槽写入用"先关 CAN RX 中断（仅该源）后拷贝"或 16 位原子写，禁止长关全局中断。日志格式化、SDO 段处理、flash 操作一律不在临界区内。

**R-203 每周期 CPU 占用上限 ≤ 30%（300 µs @168 MHz）**，核算表：

| 项 | 次数/周期 | 单次上限 | 小计 |
|---|---|---|---|
| SYNC0/PDI ISR | 1 | 20 µs | 20 µs |
| CAN RX ISR（2 路） | 6 | 10 µs | 60 µs |
| CAN TX 完成 ISR | 6 | 5 µs | 30 µs |
| TIM 节奏/软定时 ISR | ~12 | 2 µs | 24 µs |
| 主循环单圈（SSC 栈轮询+诊断+日志切片） | 1 | 150 µs | 150 µs |
| **合计** | | | **≈ 284 µs ≈ 28%** |

**R-204 CAN 总线负载上限**：正常周期单总线负载 ≤ 90%；进 OP 前按 R-603 公式预检，超限拒绝。公式：`T_bus = Σ(命令帧时间) + (n-1)×frame_gap_us + t_motor_proc + Σ(反馈帧时间) ≤ 900 µs`。

**R-205 联调实测项**（bring-up 首日必须测，用 R-1401 打点）：① 主站 LRW 写入 SM2 相对 SYNC0 的实际相位（决定反馈锁存窗口，不达标调整主站 shift）；② SYNC0→首帧上总线实测；③ 六电机满配时反馈完成时刻。设计保证不依赖具体相位（R-604 连续锁存 + 新鲜度计数）。

---

## 3. 中断架构（回答"应有哪些中断"）

**R-301 NVIC 优先级表**（4 位抢占优先级，数字越小越高；同号内 CAN1>CAN2）：

| # | 中断源 | 抢占优先级 | 触发 | ISR 内容（上限） | 追溯 |
|---|---|---|---|---|---|
| 1 | **EXTI0（ESC LINT，PC0）** | 0 | ESC 事件：SYNC0 / AL 状态 / 邮箱 | 读中断源寄存器分派：SYNC0→R-601 周期入口（≤20 µs）；邮箱/AL→仅置标志，主循环处理（≤5 µs） | FW-ECT-02 |
| 2 | **CAN1_RX0** | 1 | FIFO0 收到 1 帧 | R-604：取帧→DC 时间戳→路由→写槽（≤10 µs） | FW-CAN-02 |
| 3 | **CAN2_RX0** | 1 | 同上 | 同上 | FW-CAN-02 |
| 4 | **CAN1_TX / CAN2_TX** | 2 | 邮箱发送完成 | 打点翻转；启动 TIM 一次性 50 µs 节奏（≤5 µs） | FW-CAN-01 |
| 5 | **TIM（软定时器，1 个 TIM 复用）** | 3 | 50 µs 一次性（帧间隔节奏）；100 µs 周期（看门狗/同步丢失计数） | 出队下一帧装邮箱（≤5 µs）；计数比较置标志（≤2 µs） | FW-CAN-01/03 |
| 6 | **USART1 + DMA** | 4 | DMA 发送完成 | 取下一段日志启动 DMA（≤5 µs） | FW-SVC-05 |
| 7 | ADC/DMA（待 D-7） | 4 | 周期采样完成 | 写缓冲+置标志 | FW-DIAG-03 |

**R-302 ISR 编码规则**：所有 ISR 无动态内存、无格式化输出、无阻塞调用、无 flash 操作；ISR→主循环只经单生产者单消费者环形队列或 `volatile` 标志；返回值携带经 `volatile` 结构。

**R-303 ESC 中断使能**：ESC 寄存器开 SYNC0 事件 + AL 状态变迁 + 邮箱事件（具体寄存器/位按 AX58100 手册在 BSP 冻结，逐位注释）；看门狗事件不开中断（轮询）。

---

## 4. 软件架构与模块划分

**R-401 架构形态**：裸机、主循环轮询 + 中断驱动（假设 A1）。主循环一圈目标 ≤ 1 ms，无阻塞；空闲进 WFI。

**R-402 主循环骨架**：

```c
for (;;) {
  ec_poll();          /* SSC: 状态机/邮箱/SDO 分片处理, 单次 ≤150 µs */
  iwdg_feed();        /* IWDG 100 ms, 每圈必喂 */
  coe_apply_pending();/* SDO 写参数 → 生效（看门狗时间/安全模式等） */
  diag_pump();        /* 事件入 0x10F3、统计、状态字刷新 */
  log_flush_slice();  /* 日志切片 ≤100 µs/圈, DMA 送 USART1 */
  safety_supervise(); /* 若 SYNC0 停跳, TIM 100 µs 计数已置标志 → 切安全节拍 */
  __WFI();
}
```

**R-403 模块文件划分**（生成代码按此组织）：

| 文件 | 职责 | 关键接口 |
|---|---|---|
| `main.c` | 启动序列 + 主循环 | — |
| `esc_bsp.c` | FSMC 读写、EXTI、ESC 中断使能、DC 时间读取 | `esc_read16/32`、`esc_dc_time()` |
| `ecat_app.c` | SSC 应用钩子（R-1101 表） | `APPL_*` 全套 |
| `coe_app.c` | 对象字典（§10 表）+ 读写回调 | OD 表 + `OBJ_Download` 钩子 |
| `can_gw.c` | 下行/上行转换、路由表、0x7FF 特例 | `sync0_cycle()`、`can_rx_isr()` |
| `can_bsp.c` | bxCAN 初始化、位时序、错误恢复、统计 | `can_init/send/teardown` |
| `safety.c` | 安全状态机（§7） | `safety_enter/exit/tick` |
| `diag.c` | 0x10F3、复位原因、统计、状态字 | `diag_event(type)` |
| `log.c` | 分级日志 + 环形缓冲 + DMA | `LOGE/LOGW/LOGI/LOGD` |
| `persist.c` | 参数 flash 镜像 + CRC + 恢复出厂 | `param_load/save` |
| `timebase.c` | DC 时间、打点脚、节拍统计 | `mark_toggle()` |
| `foe_boot.c`（bootloader 内） | FoE 接收、CRC、双区跳转 | — |

---

## 5. EtherCAT→CAN 帧转换规格（核心数据面）

**总原则（R-501）**：正常控制流中从站是**透明网关**——不解析 `data[8]` 内容（MIT 打包在主站 `encos_motor.cpp` 完成），只按 `{id, rtr, dlc, data}` 原样组帧转发；从站唯一需要自主编码 CAN 帧的场景是安全状态（R-703）。

**R-601 下行转换（SYNC0 ISR 内执行）**：

```c
void sync0_cycle(void) {          /* EXTI0 ISR, SYNC0 源 */
  cycle_counter++;
  proc_start = esc_dc_time();     /* 0x0910, 读 6~8 B, ≤2 µs */
  if (al_state != OP || !output_fresh) { safety_tick(); return; }
  esc_read_sm2(&rx);              /* FSMC 读 90 B 到栈缓存, ≤5 µs */
  route_build(rx);                /* 路由表: slot_id[i]=rx.motor[i].id (含 0x7FF 特例标记) */
  for (i = 0; i < 6; i++) {                        /* 全槽扫描, 不以 motor_num 为上界 */
    if (rx.motor[i].dlc == 0) continue;             /* 空槽跳过, 判据 dlc==0 (demo 板一致) */
    bus = (i < 3) ? CAN1 : CAN2;                    /* 槽位→总线, 固定 */
    ide = rx.slot_ide ? ((rx.slot_ide >> i) & 1) : rx.can_ide;  /* D-2: 每槽位, 0x00 回退全局(兼容旧主站) */
    can_tx_enqueue(bus, rx.motor[i].id, rx.motor[i].rtr,
                   rx.motor[i].dlc, rx.motor[i].data, ide);
  }
  can_tx_kick();                  /* 每路首帧入邮箱启动; 后续帧: TX 完成中断 → TIM 50 µs 一次性 → 出队下一帧 (帧间隔可配 0x2005) */
  mark_pin_toggle(MARK_SYNC0);
}
```

规则明细：
- **R-601a 槽位路由固定**：槽 0~2→CAN1，槽 3~5→CAN2，发送顺序=槽序，不可重排（FW-CAN-01）。
- **R-601b 空槽/无效输入**：扫描范围为**全部 6 槽**，不以 `motor_num` 为上界（demo 板固件与厂商功能规格均为全槽扫描；demo 主站从不写 `motor_num` 也能正常转发）；**`dlc==0` 的槽跳过**（两 CAN 组电机数不等时中间可出现空槽，如 CAN1×2+CAN2×3 → 槽 2 空；主站未用槽恒为全零，两项判据等价）；`dlc>8` 按 8 截断；全零 PDO → 本周期无指令帧（≠安全状态）。`motor_num` 仅为诊断信息（电机数量）。
- **R-601c 帧间隔**：同总线上两帧最小间隔 `frame_gap_us`（默认 50 µs，OD 0x2005 可配 0=连续），从 TX 完成中断起由 TIM 一次性中断保证。
- **R-601d 全零 PDO**：OP 进入瞬间主站只发过一次全零 PDO（`motor_num=0`）→ 无帧下发，视为无指令（R-601b 覆盖）。
- **R-601e 逐周期透传**（假设 A3）：槽内容每 SYNC0 周期重发，不做变化检测；0x7FF/配置帧的重发节奏由主站负责（ENCOS 建议配置指令间隔 ≥500 ms）。

**R-604 上行反馈（CAN RX ISR 内执行，连续锁存模式）**：

```c
void can_rx_isr(can_bus_t bus) {          /* 优先级 1 */
  frame = fifo0_get();                    /* 取 1 帧 */
  ts = esc_dc_time();                     /* 反馈到达 DC 时间戳, ≤2 µs */
  slot = route_lookup(frame.id);          /* 查 R-601 建的表; 0x7FF 走 R-605 */
  if (slot < 0) { drop_cnt[bus]++; return; }
  recv_slot[slot].id  = frame.id;         /* 整槽 14 B 写入, 期间仅屏蔽本 CAN RX 中断, ≤5 µs */
  recv_slot[slot].rtr = 0;
  recv_slot[slot].dlc = frame.dlc;
  memcpy(recv_slot[slot].data, frame.data, frame.dlc);
  freshness[slot]++;                      /* u8 回绕 */
  motor_error[slot] = frame.data[0] & 0x1F;  /* 电机错误码镜像, HW-SPEC-0005 §11.1 */
  ts_rx_last[bus] = ts;
  mark_pin_toggle(MARK_CANRX);
}
```

- **R-604a 连续锁存**：反馈到达即写 `recv_slot`；SSC 输入映射（主循环每圈）把 `recv_slot` 快照 + 状态块拷入 SM3 映射区。不设"锁存截止时刻"——新鲜度由计数保证，主站晚一周期读到旧值也能识别。
- **R-604b 从未收到反馈的槽**：`dlc=0` 初始化（兼容现有主站 `dlc==0` 判空逻辑）；收到过之后保持最新帧不归零，新鲜度看计数（R-604c 修正旧文档"无反馈槽 dlc 归零"的表述——归零只发生在从未收到时）。
- **R-604c 新鲜度语义**：`freshness[i]` 每收到一帧该槽反馈 +1（u8 回绕）；主站比较相邻周期计数判断新旧。连续 `online_n`（默认 5，OD 0x2020）周期计数不变 → 电机离线（FW-CAN-06）。
- **R-604d 非法帧**：`dlc` 与该反馈类型不符的帧不覆盖槽（丢弃+计数）；不影响已锁存数据。
- **R-604e 未知 ID**：路由表查不到 → 丢弃 + `drop_cnt` 可读（正常不应出现，出现即路由表或电机 ID 配错）。

**R-605 0x7FF 应答路由（默认规则，待 D-3 实测 demo 板后冻结）**：本周期槽 i 命令 `id==0x7FF` → 置 `pending_7ff = i`；此后 50 ms 内（OD 可配）到达、且普通路由表匹配不上的应答帧 → 送入槽 `pending_7ff`；超时清除。零点设置应答数据 `data[2]==0x01 && data[3]==0x03` 表示成功（HW-SPEC-0005 §8.1，主站已按此判）。

---

## 6. PDO 布局 v2（字节级，D-1/D-8 已拍板）

> **总线尺寸冻结**：SM2（RxPDO，Outputs）= **90 B**，SM3（TxPDO，Inputs）= **150 B**。两尺寸一次定终身：只允许其下**字段语义**按 §6.4 规则演进，结构体长度不得再改——这是主站 `Dcu::Init()` 严格 `==` 校验风格（doc/ethercat_slave_info.md §2.3）的对称要求，也是"新从站上电=旧主站照跑、新主站=旧从站照跑"的前向/后向兼容基线。
> **布局核对基线**：前缀 86 B 与主站 [encos_slave.h:38-42](../src/module/dcu_driver_module/xyber_controller/xyber_api/include/internal/encos_slave.h#L38-L42) `EncosSlavePacket` 逐字节一致（`motor_num(1)+can_ide(1)+6×Motor_Msg(14)`；`Motor_Msg = id u32 LE + rtr u8 + dlc u8 + data u8[8]`，`static_assert(sizeof==86/14)` 已在主站代码）。`id` 为**小端 u32**，其标识符语义按该槽帧类型解释：扩展帧=29 位 EID，标准帧=低 11 位有效、高 21 位忽略。

### 6.0 PDO 构成总览

| PDO | SM | 尺寸 | 构成 | 溯源 |
|---|---|---|---|---|
| RxPDO（M→S 输出） | SM2 @0x1100 | **90 B** | 86 B 冻结前缀 + v2 控制块 4 B（+0x56..0x59） | D-1 |
| TxPDO（S→M 输入） | SM3 @0x1400 | **150 B** | 86 B 同布局前缀 + v2 状态块 24 B（+0x56..0x6D）+ IMU 预留 40 B（+0x6E..0x95） | D-1/D-8 |

### 6.1 RxPDO（主站→从站，90 B，`#pragma pack(1)`）—— R-502

```c
/* ---- 偏移 0..85: 与现网 86 B 逐字节冻结一致, 即主站 EncosSlavePacket ---- */
typedef struct {
  uint32_t id;        /* +0x00  CAN ID, LE. 语义按该槽帧类型: 扩展=29b EID, 标准=低11位有效 */
  uint8_t  rtr;       /* +0x04  远程帧标志, 透传 */
  uint8_t  dlc;       /* +0x05  0..8 */
  uint8_t  data[8];   /* +0x06..0x0D  数据域原样透传, 从站不解析内容 (R-501) */
} Motor_Msg_V2;       /* == 14 B, 与主站 Motor_Msg 逐字节一致 */

typedef struct {
  /* ---- 前缀 0..85 (86 B 冻结) ---- */
  uint8_t   motor_num;   /* +0x00  电机数量, 仅诊断信息: 固件全槽扫描不以其为上界 (demo 主站恒写 0) */
  uint8_t   can_ide;     /* +0x01  全局帧类型(legacy): 1=扩展 0=标准 */
  Motor_Msg_V2 motor[6]; /* +0x02..0x55 */
  /* ---- v2 控制块 +0x56..0x59 (4 B) ---- */
  uint8_t   slot_ide;    /* +0x56  bit0..5=槽位帧类型(1=扩展); 0x00→回退 can_ide (旧主站兼容, D-2) */
  uint8_t   cmd_seq;     /* +0x57  主站指令序列号, 每周期+1 回绕; 0xFF=不校验 (0x2008 使能) */
  uint8_t   ctrl_word;   /* +0x58  bit0=主站请求进入安全状态(→R-701 S4), 其余=0 保留 */
  uint8_t   reserved;    /* +0x59  保留, 主站写 0; 从站读但不校验 */
} EncosRxPdoV2;          /* sizeof == 90 */
```

### 6.2 TxPDO（从站→主站，150 B）—— R-503

```c
typedef struct {
  /* ---- 前缀 0..85 (86 B 冻结, 字段布局同 RxPDO) ---- */
  uint8_t   motor_num;   /* +0x00  回显 (诊断信息, 电机数量) */
  uint8_t   can_ide;     /* +0x01  回显 */
  Motor_Msg_V2 motor[6]; /* +0x02..0x55  每槽最新反馈 (R-604a 连续锁存, R-604b 从未收到=dlc 0) */
  /* ---- v2 状态块 +0x56..0x69 (20 B) ---- */
  uint8_t   freshness[6];   /* +0x56..0x5B  每槽反馈计数, 收一帧+1, u8 回绕 (R-604c) */
  uint16_t  status_word;    /* +0x5C..0x5D  位定义见 R-504, LE */
  uint16_t  cycle_counter;  /* +0x5E..0x5F  SYNC0 计数, 每周期+1 回绕, LE */
  uint8_t   motor_error[6]; /* +0x60..0x65  每槽最近反馈 data[0]&0x1F (HW-SPEC-0005 §11.1) */
  uint16_t  can1_err;       /* +0x66..0x67  [7:0]=状态位 [15:8]=TEC 快照, LE (R-505) */
  uint16_t  can2_err;       /* +0x68..0x69  同上, CAN2 */
  /* ---- v2 状态块续 +0x6A..0x6D (4 B) ---- */
  uint8_t   diag_flag;      /* +0x6A  bit0=有未读诊断事件(status_word b5 的独立镜像, 免位域解包); bit1..7 保留 */
  uint8_t   reserved[3];    /* +0x6B..0x6D  保留, 从站写 0 */
  /* ---- IMU 预留区 +0x6E..0x95 (40 B, D-8) ---- */
  uint8_t   imu_reserved[40];  /* 本期固件恒写 0; 内容布局延后到 IMU 阶段冻结 */
} EncosTxPdoV2;             /* sizeof == 150 */
```

### 6.3 偏移总表（双 PDO 对照，十六进制字节偏移）

| 偏移 | RxPDO (SM2, 90 B) | TxPDO (SM3, 150 B) |
|---|---|---|
| 0x00 | motor_num | motor_num（回显） |
| 0x01 | can_ide | can_ide（回显） |
| 0x02..0x55 | motor[0..5] × 14 B（**命令**） | motor[0..5] × 14 B（**反馈锁存**） |
| 0x56 | slot_ide | freshness[0] |
| 0x57 | cmd_seq | freshness[1] |
| 0x58 | ctrl_word | freshness[2] |
| 0x59 | reserved（RxPDO 至此结束） | freshness[3] |
| 0x5A | — | freshness[4] |
| 0x5B | — | freshness[5] |
| 0x5C..0x5D | — | status_word（LE） |
| 0x5E..0x5F | — | cycle_counter（LE） |
| 0x60..0x65 | — | motor_error[0..5] |
| 0x66..0x67 | — | can1_err（LE） |
| 0x68..0x69 | — | can2_err（LE） |
| 0x6A | — | diag_flag |
| 0x6B..0x6D | — | reserved[3]（写 0） |
| 0x6E..0x95 | — | **IMU 预留 40 B（D-8：本期恒 0，布局延后冻结）** |
| 0x96 | — | （TxPDO 至此结束，总长 150 B） |

### 6.4 布局修订规则（冻结）

1. **字段上限已到顶**：RxPDO 控制块止于 0x59、TxPDO 状态块止于 0x6D——v2 之内除 reserved/IMU 区外再无空闲字节，防"顺手加字段"。
2. **扩展只能加尾部**：TxPDO 任何 v2 内新增状态字段，一律放 **0x96 之后（IMU 区之后）**；RxPDO 同理只能放 0x5A 之后。凡此一律属 **PDO v3**，须整版冻结（SII/ESI + 从站 + 主站三方同步）。
3. **字段语义修订仅限**：状态字/错误码**枚举值补充**、保留位**启用**（如 status_word b13..b15、ctrl_word bit1..7）——不得移动现有字段位置/宽度，不得动 IMU 区。
4. **静态断言防线**：从站代码 `static_assert(sizeof(EncosRxPdoV2)==90)`、`static_assert(sizeof(EncosTxPdoV2)==150)`、`static_assert(sizeof(Motor_Msg_V2)==14)`；SII SM2 长度 90 / SM3 长度 150 与 ESI、固件三方一致（R-505）。
5. **字节序**：PDO 多字节字段一律**小端**（与 86 B 前缀现状、主站 `std::copy` 语义一致）。CAN 帧内字段序仍按 HW-SPEC-0005（MIT 位域大端拼接），与 PDO 字节序互不相干。
6. **旧主站兼容（前缀 86 B）**：旧主站读/写截断 86 B（`Pull/Push` min(len,86)），从站把缺省的 slot_ide=0 回退 can_ide（D-2）、全零 PDO 视为无指令（R-601b/d）；新主站按 90/150 B 全量读写。
7. **IMU 预留区纪律（D-8）**：本期从站固件每周期主动清零该 40 B；主站本期不解读。启用 IMU 时先冻结区内布局再发布——区内布局属 IMU 阶段决策，不改 PDO 总长。

**R-504 状态字位定义**（FW-DIAG-04，与主站同步冻结）：
`b0 sync_ok`（连续 SYNC0 正常）｜`b1 can1_ok`｜`b2 can2_ok`（无 bus-off 且 TEC<96）｜`b3 imu_ready`（本期恒 0）｜`b4 safe_active`｜`b5 diag_pending`｜`b6 watchdog_tripped（本周期内发生过）`｜`b7..b12 motor_online[0..5]`｜`b13..b15 保留`。

**R-505 尺寸自洽**：SM2=90 B、SM3=150 B 精确配置进 SII/ESI（消除主站 86 B 截断补丁土壤）；`can_err[7:0]` 状态位：`b0 busoff_active｜b1 rx_overrun_ever｜b2 tx_fail_ever｜b3 tec_warn`，`[15:8]`=TEC 快照。

---

## 7. 安全状态机与安全帧编码（D-5 已拍板）

**R-701 状态表**：

| 当前态 | 条件（OR 触发） | 动作 | 次态 |
|---|---|---|---|
| NORMAL | S1: SM2 输出新鲜度超时 > `watchdog_ms`（默认 5000，OD 0x2002） | 记诊断事件、置 `safe_active` | SAFE |
| NORMAL | S2: AL 离开 OP（SSC `APPL_StopInputHandler` 钩子） | 同上 | SAFE |
| NORMAL | S3: 连续 `sync_loss_n`（默认 10）周期无 SYNC0（TIM 100 µs 节拍计数） | 同上 | SAFE |
| NORMAL | S4: `ctrl_word.bit0==1`（主站显式请求） | 同上 | SAFE |
| SAFE | 触发源全部消失 **且** 连续 `recovery_k`（默认 5）个周期收到有效输出 **且** AL=OP | 清 `safe_active`、记恢复事件 | NORMAL |
| SAFE | 保持 | 每 1 ms（SYNC0 或 TIM 兜底节拍）向全部使能槽下发安全帧 | SAFE |

**R-702 SAFE 期间行为**：忽略 RxPDO 槽内容；反馈照常接收锁存（主站可见电机状态）；周期计数继续。

**R-703 安全帧编码**（从站唯一自主 MIT 编码场景）：按槽 `safe_mode`（OD 0x2010，默认 0）：

| 模式 | 帧 | 编码 |
|---|---|---|
| 0 MIT 零力矩（默认） | 扩展帧 8 B，id=该槽电机 ID | `kp=kd=tor=0`，`pos=vel=量程中点`。以默认量程（POS ±12.5、SPD ±18、TOR ±30）为例字节为 `00 00 00 7F FF 7F F7 FF` |
| 1 阻尼 | 同上 | `kd=quantize(safe_kd,0..5,9bit)`，其余同零力矩 |
| 2 停发 | 不发帧 | 依赖电机 500 ms 自身超时停转（HW-SPEC-0005 §7） |

量化公式（与主站 `FloatToUint` 一致，截断取整）：`raw = (x - x_min) × (2^n - 1) / (x_max - x_min)`。量程按槽电机型号（OD 0x2012：0=A8116 → TOR ±90/CUR ±60；1=A4315 → ±70/±30；2=自定义走 0x2013 量程块）。

---

## 8. 看门狗、同步监测与复位原因

**R-801 双层看门狗**：① ESC SM watchdog（硬件，SII/寄存器配置，监督 PDI 访问，兜底固件挂死）；② 应用层输出新鲜度——SM2 新数据事件（EXTI0 的 AL/SM 事件或每周期读后比对）记 DC 时间戳，超 `watchdog_ms` 触发 S1（R-701）。默认 5000 µs，容忍主站 4 帧丢失（FW-ECT-03）。

**R-802 SYNC0 丢失**：TIM 100 µs 节拍累计无 SYNC0 周期数 ≥ `sync_loss_n` → S3；恢复后清零。计数器 `sync_lost_cnt` 可读可清（FW-DIAG-07）。

**R-803 IWDG**：LSI 32 kHz / 32 分频 / 重载 100 → 超时 100 ms；主循环喂狗；超时复位后按复位原因进入安全启动（FW-SVC-02）。

**R-804 复位原因映射**（FW-DIAG-02，RCC_CSR 读后清）：bit0 `PORRSTF` 上电｜bit1 `BORRSTF` 欠压｜bit2 `PINRSTF` 引脚｜bit3 `IWDGRSTF` 独立看门狗｜bit4 `SFTRSTF` 软件；上电第一条：读 CSR → 存 RAM+参数区镜像 → 清标志；异常复位（IWDG/BOR/POR 之外）直接以 SAFE 态启动。

---

## 9. CAN 位时序与错误管理

**R-901 位时序**（1 Mbps，PCLK1 42 MHz）：prescaler=2 → 21 tq/位；BS1=15、BS2=5、SJW=3 → 采样点 (1+15)/21 ≈ 76.2%（目标 76.5%±1%，HW-SPEC-0005 §7）。

**R-902 接收配置**：两路均 FIFO0、全接收（掩码 0，软件路由）；FIFO 溢出（OVR 位）→ `rx_overrun_cnt`++。

**R-903 Bus-off 恢复流程**（确定性，FW-CAN-04）：检测 ESR BOFF → ① 立即：记诊断事件、置状态字 `canX_ok=0`、该路进入 SAFE 节拍；② 延时 `busoff_recover_ms`（OD 0x2006，默认 5 ms，TJA1050 无需额外恢复，总线隐性 129×11 位≈1.42 ms）→ bxCAN 进 init → 重配位时序 → 退出 init（TEC/REC 清零）→ 重发安全帧；③ 无限重试但每次计数 `busoff_cnt`；④ 诊断事件含次数。

**R-904 错误统计来源**：TEC/REC ← ESR `TEC/REC` 字段（周期快照入 PDO 摘要+SDO 全量）；发送失败 ← 邮箱发送超时（TIM 5 ms 看门狗，TXOK 未置位）计数；接收溢出 ← FIFO OVR；bus-off 次数 ← R-903。全部 SDO 可读（0x2030）、可清（写 0）。

**R-905 带宽预检**（FW-CAN-05，进 OP 时执行 R-204 公式）：按两路各自**非空槽数**与各槽 dlc 计算 `T_bus`（`motor_num` 仅为电机数量参考，含空槽布局下不能当每路帧数），任一 > 900 µs → 拒绝进 OP：AL status code 返回 `0x001D` 系列（具体码按 ETG.1000.4 AL 状态码表冻结，候选 0x001D Invalid output configuration）+ 诊断对象写入原因码"CAN 带宽超限（含计算值）"，禁止笼统失败。

---

## 10. 对象字典（CoE OD，代码直接照抄此表）

**R-1001 标准区**：

| Index | Sub | 名称 | 类型 | 访问 | 值 |
|---|---|---|---|---|---|
| 0x1008 | 0 | DeviceName | string | RO | "F1-EC-EcatCanGW" |
| 0x1009 | 0 | HwVersion | string | RO | "V1.0" |
| 0x100A | 0 | SwVersion | string | RO | "x.y.z" |
| 0x1018 | 1/2/3/4 | Identity: vendor/product/serial/revision | u32 | RO | vendor/product 待定（建议 F1 专用，主站不校验）|
| 0x10F3 | 1..N | DiagHistory（条目含 DC 时间戳） | 标准 | RO | 深度默认 16，SII 配置 |

**R-1002 自定义区 0x2xxx**（除标注 RO 外 RW；`保存` 列=掉电保存进 Param 区）：

| Index:Sub | 名称 | 类型 | 默认 | 保存 | 追溯 |
|---|---|---|---|---|---|
| 0x2002:1 | watchdog_ms | u16 | 5000 | ✓ | FW-ECT-03 |
| 0x2005:1 | frame_gap_us | u16 | 50 | ✓ | FW-CAN-01 |
| 0x2006:1 | busoff_recover_ms | u16 | 5 | ✓ | FW-CAN-04 |
| 0x2008:1 | cmd_seq_en | u8 | 0(禁用) | ✓ | R-502 |
| 0x2010:1..6 | safe_mode[槽] | u8 | 0 | ✓ | FW-CAN-03/D-5 |
| 0x2011:1..6 | safe_kd[槽]×100 | u16 | 0 | ✓ | R-703 |
| 0x2012:1..6 | motor_model[槽] | u8 | 1(A4315) | ✓ | R-703 |
| 0x2013:1..20 | 自定义 MIT 量程块（min/max×10 组） | i16×20 | 型号默认 | ✓ | R-703 |
| 0x2020:1 | online_n | u8 | 5 | ✓ | FW-CAN-06 |
| 0x2021:1 | recovery_k | u8 | 5 | ✓ | R-701 |
| 0x2022:1 | sync_loss_n | u8 | 10 | ✓ | R-802 |
| 0x2030:1..5 | CAN1: tec/rec/tx_fail/ovr/busoff_cnt | u8/u8/u32/u32/u32 | — | ✗ RO | FW-CAN-04 |
| 0x2030:6..10 | CAN2 同上 | 同上 | — | ✗ RO | |
| 0x2031:1 | drop_cnt[2] | u32×2 | — | ✗ RO | R-604e |
| 0x2040:1 | reset_cause | u8 | — | ✗ RO | FW-DIAG-02 |
| 0x2041:1 | uptime_s | u32 | — | ✗ RO | |
| 0x2042:1/2/3 | v5/v33/v12 ×100 mV | u16 | — | ✗ RO | FW-DIAG-03（待 D-7 通道） |
| 0x2043:1 | board_temp ×10 ℃ | i16 | — | ✗ RO | 同上 |
| 0x2044:1..6 | voltage/temp 越限阈值 | u16/i16 | 4750/3150/1100(±10%)、60.0 | ✓ | FW-DIAG-01 |
| 0x2050:1..4 | ts_can_tx/rx [2 路] | u64 | — | ✗ RO | FW-DIAG-06 |
| 0x2051:1 | sync_lost_cnt | u32 | — | RO/清 | FW-DIAG-07 |
| 0x2052:1/2 | proc_time_max_us / avg | u16 | — | RO/清 | FW-DIAG-07 |
| 0x2053:1 | wdcnt_tripped | u32 | — | RO/清 | |
| 0x2060:1 | param_ctrl（1=保存 2=出厂） | u8 | 0 | ✓ | FW-SVC-06 |
| 0x2061:1 | param_version | u32 | — | ✗ RO | |
| 0x2062:1 | selftest_result 位域 | u32 | — | ✗ RO | FW-SVC-04 |
| 0x2070:1 | log_level（0..3） | u8 | 1(INFO) | ✓ | FW-SVC-05 |
| 0x2080:1 | build_time | string | 编译期 | ✗ RO | FW-SVC-01 |
| 0x2081:1 | serial_number | u32 | 每板唯一 | ✗ RO | FW-SVC-01 |

**R-1003 写校验**：所有 RW 对象带范围检查，越界返回 SDO abort 0x0609 30（值超上限）；`watchdog_ms` 生效即改 ESC/应用比较基准；参数保存写入 Param 区带 CRC32 + `param_version`+1。

---

## 11. 启动流程与 SSC 钩子

**R-1101 SSC 应用钩子职责表**：

| 钩子 | 时机 | 本固件职责 |
|---|---|---|
| `APPL_StartMailboxHandler` | →PRE_OP | 诊断/统计对象初始化；清自检结果标志 |
| `APPL_StopMailboxHandler` | 离开 | — |
| `APPL_StartInputHandler` | →SAFE_OP | 启动 TxPDO 映射（`recv_slot` 快照+状态块）；此刻起反馈照收 |
| `APPL_StopInputHandler` | 离开 SAFE_OP/OP | 触发 S2 进 SAFE；停周期发送 |
| `APPL_Application` | 主循环 | 输入/输出映射搬移（`ecat_app.c`） |
| SYNC0 ISR 钩子 | 每周期 | `sync0_cycle()`（R-601） |
| `APPL_ApplicationInputValidation` | OP 前 | R-905 带宽预检（失败返回具体 AL 码） |

**R-1102 上电序列**（`main.c`）：
1. 早期安全初始化：两路 bxCAN 保持不发送（init 模式），加热 PF6 浮空，LED 全灭。
2. R-804 读复位原因。
3. 自检（FW-SVC-04，结果写 0x2062 位域，任一失败→拒绝进 OP+具体位）：bit0 ESC PDI（FSMC 读 ESC 版本寄存器+DC 时间 100 ms 内走动）；bit1 SII 加载（ESC EEPROM 加载状态位，位号按 AX58100 手册冻结）；bit2/3 CAN1/CAN2 回环（bxCAN Silent+LoopBack 自发自收各 1 帧）；bit4 参数区 CRC（失败用默认值继续，不阻断）。
4. `param_load()` → 生效配置。
5. SSC 栈初始化（`ECAT_Init`）→ 等主站带状态机。
6. 启 IWDG（R-803）→ 进入主循环（R-402）。

**R-1103 FoE 升级流程**（FW-SVC-03）：主站经 FoE 写 AppB → 每块 CRC32 增量校验 → 完成后置 Param 区"启动选择=A/B+镜像 CRC"标志 → 复位 → bootloader 校验 AppA CRC 失败或标志选 B 且 B CRC 通过 → 跳转对应区；App 内每周期喂狗+检测升级请求。**约束：flash 擦写仅在 SAFE/维护模式执行（擦写期间 flash 停运影响取指，周期抖动指标在该模式豁免，主站须知）**。

---

## 12. 维护功能其余规格

**R-1201 日志**（FW-SVC-05）：`LOGE/W/I/D` 四级宏；环形缓冲 4 KB（ISR 投递只拷贝定长记录，格式化在主循环切片完成）；DMA 发送；默认 INFO；`log_level` SDO 运行时改。
**R-1202 参数持久化**（FW-SVC-06）：R-1003；恢复出厂=写默认镜像+复位。
**R-1203 LED**（FW-SVC-07）：LED4=RUN/LED5=ERR 由 AX58100 硬件驱动（SII 正确即遵循 EtherCAT 指示灯规范）；LED1/LED2（PF7/PF8）应用状态：LED1 心跳 1 Hz（主循环），LED2 安全状态/总线错误快闪 5 Hz——**待 §3.8 原文后修订（D-7②）**。
**R-1204 打点脚**（FW-DIAG-08）：复用 PF8（编译开关 `MARK_ENABLE`，默认关；开着时与 LED2 冲突已知）；事件：SYNC0 ISR 翻转、CAN TX 完成翻转、可选 CAN RX。
**R-1205 串口参数**：USART1 115200-8-N-1。

---

## 13. 主站兼容硬约束（浓缩，代码侧必须内建）

1. 主站无 PRE_OP 重映射 → **SII 的 SM/PDO 配置即生效配置**，SM2=90/SM3=150 必须与 ESI、固件三方一致（R-505）。
2. 主站清 SDOCA 位 → 固件不依赖 CA 读，PDO 映射普通 SDO 读必须正确。
3. 主站 `ec_dcsync0(1ms,0)` → SYNC0 相位 0；DC 必须在 SII 使能。
4. 主站 1 ms WorkLoop、停止即无输出 → 看门狗 5 ms 默认合理（R-801）。
5. **偏移 0..85 语义冻结**（R-502/503），无反馈槽 `dlc=0` 语义保留（R-604b）。
6. 主站按拓扑 ecat_id 识别 → vendor/product 可自由改（建议 F1 专用）。
7. OP 进入瞬间可能收到全零 PDO → 视为无指令（R-601b/d）。
8. 主站 ErrorHandler 依赖标准 AL ACK 语义 + 具体状态码（R-905）。

**主站侧联动**（不在本固件范围，另行排期）：`EncosSlavePacket` 扩展至 90/150 B（§6 结构，含 slot_ide/cmd_seq/ctrl_word 与状态块+IMU 预留）、状态字/新鲜度消费、`>=86` 校验回归 `==`、MIT 量程对齐（主站默认 ±30 与 A8116/A4315 出厂量程不一致）。

---

## 14. 待确认清单（阻塞对应条目的编码细节，不阻塞框架）

| # | 事项 | 阻塞 |
|---|---|---|
| D-3 | demo 板 0x7FF 应答实际路由行为（实测抓包） | R-605 |
| D-7① | 三路电压/板温 ADC 通道（V1.0 原理图未见分压网络） | R-105、0x2042/0x2043 |
| D-7② | 硬件需求 §3.8 LED 定义原文 | R-1203 |
| D-9 | 电机心跳 500 ms 与安全链路快慢匹配（联调定） | R-703 模式 2 |
| A1/A2/A3 | 裸机、FSMC、逐周期透传三个实现假设确认 | 全局 |
| — | AX58100 手册待冻结项：①ESC 中断使能寄存器/位号；②SII EEPROM 加载状态位；③U2-1 (WD_TRIG/LINT/SINT) 在 16 位本地总线模式下确为 PDI 中断（LINT）；④U2-6 (LAT_IN/LRDY) 时序语义 | R-303、R-1102、R-102 |
| — | AL 状态码具体值（查 ETG.1000.4 表冻结） | R-905 |
