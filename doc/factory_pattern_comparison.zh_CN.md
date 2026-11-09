# 工厂模式三种形态对比示例（简单工厂 / 工厂方法 / 注册式工厂）

> **关联文档**：[Slave 工厂模式重构方案](slave_factory_refactor.zh_CN.md)（本示例是其 §2.2 选型结论的展开讲解）
> **示例场景**：取自本项目——按类型字符串创建 EtherCAT 从站（`Dcu` / `EncosSlave`，未来 `SelfDevDcu`）
> **读者**：想搞清楚三种工厂写法区别、以及本项目为什么选注册式工厂的同学

---

## 0. 公共部分：产品族（三种形态都先有这个）

无论哪种工厂形态，前提都是**同一组产品继承自统一基类**：

```cpp
class SlaveBase {              // 产品基类
 public:
  virtual ~SlaveBase() = default;
  virtual bool Init() = 0;
};

class Dcu        : public SlaveBase { /* 240B PDO ... */ };
class EncosSlave : public SlaveBase { /* 86B PDO ...  */ };

// 调用方的心愿：只报类型字符串，拿到基类指针，不认识具体类
// SlaveBase* s = ???("encos");
```

三种形态的差别只在：**工厂长什么样、加新类型时改哪里**。

---

## 1. 简单工厂 —— 把 if/else 从调用方"搬进"工厂

```cpp
SlaveBase* CreateSlave(const std::string& type) {
  if (type == "dcu")   return new Dcu("body", 1);
  if (type == "encos") return new EncosSlave("waist", 2);
  return nullptr;                       // 未知类型
}

// 调用方：确实不认识具体类了 ✅
SlaveBase* s = CreateSlave("encos");
```

**加自研 DCU 时**：必须回来**改这个函数**，加一行 `if (type == "selfdcu") ...`。

判断没有消失，只是集中了。调用方闭合了，**工厂本身不闭合**——开闭原则只做一半。

> **本质：判断的搬家。**

---

## 2. 工厂方法 —— 用多态取代 if/else，每个产品配一个工厂类

```cpp
class SlaveFactoryBase {                        // 抽象工厂
 public:
  virtual ~SlaveFactoryBase() = default;
  virtual SlaveBase* CreateSlave() = 0;         // "工厂方法"
};

class DcuFactory : public SlaveFactoryBase {
  SlaveBase* CreateSlave() override { return new Dcu("body", 1); }
};
class EncosSlaveFactory : public SlaveFactoryBase {
  SlaveBase* CreateSlave() override { return new EncosSlave("waist", 2); }
};
```

```cpp
// 调用方：依赖注入，注入哪个工厂就造哪种从站
void SetupRobot(SlaveFactoryBase& factory) {
  SlaveBase* s = factory.CreateSlave();
  s->Init();
}

SetupRobot(EncosSlaveFactory{});   // 想换 dcu？装配处换成 DcuFactory{}
```

**加自研 DCU 时**：新增**两个类**（`SelfDevDcu` + `SelfDevDcuFactory`），旧代码一行不改 ✅ 完全开闭。

代价看得见：**产品类和工厂类 1:1 成对，类数量翻倍**；而且"字符串 → 选哪个工厂"的判断只是被推到了装配处——总得有人写 `if (type == "encos") SetupRobot(EncosSlaveFactory{})`。类型少的时候，仪式感大于收益。

> **本质：判断交给"注入的是哪个工厂对象"（虚函数多态）。**

---

## 3. 注册式工厂 —— 把"类型→构造"做成一张表，而不是一串 if

```cpp
class SlaveFactory {
 public:
  using Creator = std::function<SlaveBase*(std::string name, int32_t id)>;

  static bool Register(const std::string& type, Creator c) {      // 往表里登记
    return Registry().emplace(type, c).second;
  }
  static SlaveBase* Create(const std::string& type, std::string name, int32_t id) {
    auto it = Registry().find(type);                             // 查表
    return it == Registry().end() ? nullptr : it->second(name, id);
  }
 private:
  static std::unordered_map<std::string, Creator>& Registry() {
    static std::unordered_map<std::string, Creator> r;
    return r;
  }
};

// 各类型登记（集中注册或自注册均可）：
SlaveFactory::Register("dcu",   [](std::string n, int32_t i){ return (SlaveBase*)new Dcu(n, i); });
SlaveFactory::Register("encos", [](std::string n, int32_t i){ return (SlaveBase*)new EncosSlave(n, i); });
```

```cpp
// 调用方：和简单工厂一样简洁
SlaveBase* s = SlaveFactory::Create("encos", "waist", 2);
```

**加自研 DCU 时**：写 `SelfDevDcu` 类 + **一行 `Register("selfdcu", ...)`**——不改任何现有函数，工厂类不随类型增长。查表逻辑是通用的，永远不会因为"类型变多"而需要修改。

> **本质：类型到构造的映射从"代码"（if 链）变成"数据"（map）——新增类型 = 往表里加一条数据。**

---

## 4. 三者一表看全

| 维度 | 简单工厂 | 工厂方法 | 注册式工厂 |
|---|---|---|---|
| 类型判断在哪 | 工厂函数里的 if/else | 虚函数（注入哪个工厂） | map 查表 |
| 加新类型改什么 | **改工厂函数**（+1 分支） | **+2 个类**（产品+工厂），不改旧码 | **+1 类 +1 行注册** |
| 类数量 | 1 个工厂函数 | 产品数 × 2（成对出现） | 1 个工厂 + 产品 |
| 开闭原则 | 只对调用方成立 | 完全成立 | 完全成立 |
| 适合场景 | 类型极少且基本不变 | 产品要**成族配套创建**（复杂依赖） | 类型多、**按字符串配置驱动**、会持续增长 |

**一个类比帮助记忆**：

- **简单工厂**：一个什么都懂的前台，新增业务得培训它（改它代码）；
- **工厂方法**：每个业务设自己的前台，业务多了前台也成排（类翻倍）；
- **注册式**：一面公告牌（map），新业务自己把"名字→窗口号"贴上去，谁来找都只是查牌（查表逻辑永不变）。

---

## 5. 本项目的选型结论

对照仓库的实际约束，三条都指向注册式工厂：

1. 从站类型来自 **YAML 字符串**（`type: dcu` / `type: encos`，运行期才确定，天然适合字符串查表）；
2. 类型**会增长**：自研 DCU 在路上，Encos 板按部位还会加；
3. 不接受工厂方法那种"产品×工厂"类数翻倍。

唯一要注意的工程坑：**静态库下"各产品 .cpp 文件自注册"可能被链接器丢弃**（目标文件无外部符号引用时整文件不链入 → 注册丢失 → 工厂返回空指针）。因此本仓库把注册集中放在 `SlaveFactory::RegisterBuiltinSlaves()`（首次 `Create()` 时惰性执行），详见[重构方案 §2.3](slave_factory_refactor.zh_CN.md)。

最终落地形态（与方案文档一致）：

```cpp
// InitDcu() 里——对 type 无感知：
xyber_ctrl_->CreateSlave({.type = slave_cfg.type, .name = slave_cfg.name,
                          .ecat_id = (int32_t)slave_cfg.ecat_id, .can_ide = slave_cfg.can_ide});
```
