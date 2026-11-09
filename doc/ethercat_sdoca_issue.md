# EtherCAT SDO Complete Access (SDOCA) 导致 SAFE_OP 失败

> 现象：encos（SSC-Device）从站进 SAFE_OP 失败，AL status `0x001E "Invalid input configuration"`。
> 根因：从站 CoE details 带 `ECT_COEDET_SDOCA` 位，SOEM 用 Complete Access 读 PDO 映射，但该从站 CA 实现有 bug，拿到错误映射，被从站应用层拒绝。
> 修复：`ec_config_map()` 之前清除所有从站的 `ECT_COEDET_SDOCA` 位，强制走普通 SDO 读。与 `soem_demo_cpp` 做法一致。

---

## 1. 背景：SDO 与两种读法

EtherCAT 从站内部有**对象字典（OD, Object Dictionary）**，每项用 `0x索引:子索引` 寻址（如 `0x1C12:01`）。主站在 PRE_OP 阶段通过 **SDO（Service Data Object）** 经邮箱（mailbox）读写从站配置（含 PDO 映射）。读一个对象有两种方式：

| 方式 | 过程 | 特点 |
|---|---|---|
| **普通读（Normal SDO Read）** | 先读 `:00` 得子索引数 n，再逐个读 `:01..:0n`，共 n+1 次请求 | 所有 CoE 从站必须支持；基础方式 |
| **Complete Access 读（CA）** | 一次请求读回整张表 `:00..:0n` | 更省往返、更原子；**但需从站固件正确实现 CA** |

两者拿到的数据理论上相同，只是 CA 更高效。前提是从站正确实现 CA——很多从站（SSC/EasyCAT/自研板等）CA 实现有 bug 或不支持，CA 读会返回残缺/错位数据。

## 2. SDOCA 位的作用

从站扫描时 `ec_slave[i].CoEdetails` 带一个 `ECT_COEDET_SDOCA (0x20)` 位（定义见 `third_party/soem/soem/ethercatmain.h:99`），表示"该站声明支持 Complete Access"。

SOEM 的 `ec_config_map()` 读 PDO 配置（`0x1C12/0x1C13` 及其指向的 `0x1600/0x1A00` 映射对象）时，**只要这一位是 1，就用 CA 方式读**。

## 3. 本系统的故障

- 拓扑：slave[1] = body（DCU，EasyCAT-AX58100，240/240B）；slave[2] = waist（encos，SSC-Device，186/192B）。
- 故障：只有 slave[2] 进 SAFE_OP 失败：

  ```
  [ERROR] Slave waist (2) set EC_STATE_SAFE_OP failed.
          State=0x12 StatusCode=0x001e : Invalid input configuration
  ```
  `0x12` = SAFE_OP + ERROR；`0x001E` = 输入(TxPDO)配置非法。
- 流程：主站读 PDO → 配置 SM → 请求 SAFE_OP → **从站应用层自检 PDO 配置** → 发现 Input PDO 与固件期望不符 → 拒绝。

### 为什么只有 encos 失败、AX58100 正常
encos（SSC-Device）虽带 SDOCA 位，但 CA 读实现有问题 → 拿到错误 Input PDO 映射 → 被拒。AX58100（DCU）CA 没问题 / PDO 简单，同套代码能过。

> 注意：之前 host 侧那个"PDO 186/192 > 结构 86B，截断到 86B"的补丁（`encos_slave.cpp`）只保护 host 缓冲，**不改变从站 PDO 配置**，救不了这一步。从站自己拿到的映射还是错的。

## 4. 修复

在 `ec_config_init()` 之后、`ec_config_map()` 之前，对所有从站清除 SDOCA 位：

```cpp
// ethercat_manager.cpp, EthercatManager::Start()
for (int32_t i = 1; i <= ec_slavecount; i++) {
  ec_slave[i].CoEdetails &= ~ECT_COEDET_SDOCA;
}
```

清位后 SOEM 退回普通 SDO 读（逐子索引），encos 用普通读能正确返回 PDO 表，映射正确，SAFE_OP 通过。AX58100 清位后改用普通读，结果与 CA 一致，不受影响（普通读是所有 CoE 从站的基础能力）。

## 5. 与 soem_demo_cpp 的对照

`soem_demo_cpp/app_cpp/transmit.cpp:68-69` 在 `ec_config_map()` 之前做了同样的事：

```c
for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
    ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;
```

soem_demo 不写任何显式 CA 调用（无 `ec_SDOread(..., TRUE)`），PDO 配置完全交给 `ec_config_map` 自动完成；这两行清位就是把读法从 CA 拨回普通读。该 demo 的 encos 能正常进 OP，正是靠这一步。本仓库此前少了这一步，现补齐。

## 6. 关键定位线索

- `0x001E "Invalid input configuration"` 专门指向 **Input(TxPDO) 配置非法**，而非 DC/watchdog/输出问题——直接锁定 PDO 映射被从站判错。
- 对照 soem_demo（同款 encos 板，能进 OP）的初始化序列，发现多了一行 `CoEdetails &= ~ECT_COEDET_SDOCA`，差异即在此。
- 进一步佐证"encos 的 CA 实现有问题"：AX58100 不清位也能过，说明不是 SOEM 全局 bug，而是从站 CA 支持差异。

## 7. 相关代码

| 位置 | 内容 |
|---|---|
| `src/.../xyber_api/src/ethercat_manager.cpp`（`Start()`，`ec_config_init` 之后） | 本次修复：清 `ECT_COEDET_SDOCA` |
| `third_party/soem/soem/ethercatmain.h:99` | `#define ECT_COEDET_SDOCA 0x20` |
| `soem_demo_cpp/app_cpp/transmit.cpp:68-69` | 参考做法 |
| `src/.../xyber_api/src/ethercat_manager.cpp`（SAFE_OP 失败处） | 已补 AL status/ALstatuscode 打印，用于定位本类问题 |
