# 新下肢 F1 踝关节传动

本工程只保留新硬件实现。旧位置逆解、展开式雅可比和旧正解 YAML 已移除。

## 文件与来源

| 文件 | 用途 |
|---|---|
| `src/module/dcu_driver_module/src/ankle_transmission.cc` | 新表读取、四角双线性插值、左右脚适配及数据空间转换 |
| `src/module/dcu_driver_module/src/ankle_kinematics.cc` | 从独立验证项目移植的位置逆解、隐式雅可比及速度/扭矩映射 |
| `src/module/dcu_driver_module/cfg/ankle_f1.yaml` | 左右脚几何、曲柄方向、有效网格设置和回代检查阈值 |
| `src/module/dcu_driver_module/cfg/ankle_trans_f1_new.yaml` | 已生成的新硬件正解表 |

几何来源为独立项目 `ankle_parallel_validation/config/new_hardware.json`，表来源为该项目的 `lookup_tables/`。配置 `provenance` 保存来源哈希。表加载时逐节点核对左右脚的连杆长度约束，以及当前 C++ 逆解是否返回表项对应的原电机角；任一节点不一致即拒绝加载。

生成阶段在旧表候选网格内，选择从原网格下界开始、包含电机零位、所有节点均通过当前逆解检查的最大矩形。异常边缘节点不写入发布 YAML，不再保留异常角度后用掩码屏蔽。为保持矩形格式，部分本身通过检查的边缘节点也随外边界一并移除；详情见独立项目的 `lookup_tables/excluded_candidate_nodes.json`。这不等于证明被删除区域不存在其他分支一致的正解。

`dcu_x1.yaml` 的左右踝 `param_path` 均指向 `cfg/dcu_driver_module/ankle_f1.yaml`，表文件路径相对该配置文件所在目录解析。原有 CMake 的 `src/*.cc` 收集和 `cfg/` 安装规则会包含新增文件；部署时必须重新配置、构建并同步整个配置目录。

## 坐标、编号和方向

物理坐标为 X 向前、Y 向机器人左侧、Z 向上。pitch 正值为绷脚；roll 正值为后视左边高、右边低，即左脚外翻、右脚内翻。

两脚 5 号均为上部长杆电机，6 号均为下部短杆电机。从机器人背后向前看，四个原始 CAN 电机正向均为逆时针。零位曲柄：左脚 5 朝左、6 朝右；右脚相反。编码器机械零位须与此零位对应。

现有绑定保留：左脚 5=`left_ankle_right_actuator`、6=`left_ankle_left_actuator`；右脚 5=`right_ankle_left_actuator`、6=`right_ankle_right_actuator`。当前配置的 actuator direction 均为正；数学几何中曲柄的 motor_direction 已包含实际轴向，不要再次统一反号。

单张正解表以右脚为基准：键 `qm5qm6_i_j` 中 i 对应5号、j对应6号；q5为pitch、q6为roll，单位均为弧度。左脚查表前两电机角取反，查表后仅roll取反。这是共用表的镜像换算，不是额外改变 CAN 指令方向。

## 转换链路

定义 $J=\partial q_m/\partial q$：

$$
\dot q_m=J\dot q,\qquad \dot q=J^{-1}\dot q_m
$$

$$
\tau_m=J^{-T}\tau_q,\qquad \tau_q=J^T\tau_m
$$

上行使用实测电机位置查表获得关节位置，再以同一状态计算雅可比、关节速度和反馈effort。下行对目标位置做逆解；速度与effort映射使用实测电机位置及其查表姿态计算的雅可比。

MIT位置、速度、effort和增益接口保持原有结构。RL行走分支将电机端增益设为零，因此有效控制项为effort；站立等PD分支仍保留原有增益透传规则。

反馈effort沿用原驱动数值，不加入电流到扭矩的换算；也不声称其物理单位已验证为Nm。两路反馈均读取各自的state.effort。电机协议层没有在本次修改。

## 边界与失败处理

读取完整有效矩形网格，使用四角双线性插值，不外推，也不将越界输入强行夹紧到边界。旧候选网格中被删除的外边缘现在属于查表范围之外；不再有“表内存在异常节点、运行时通过掩码拒绝”的情况。网格范围不是硬件电机限位，不能据此认定无碰撞或整个区域可在真机运行。

不合法数据、不可达目标、奇异雅可比、约束不一致或不支持的查表区域会报告转换失败。驱动捕获异常后锁存故障，向所有执行器发送零effort、零kp/kd的MIT指令，并停止发布不完整的关节状态。后续指令不能自动恢复运动；须排除原因后重启模块。零effort不等同于机械制动或支撑保持。

完整反馈快照、正逆转换及命令提交使用同一互斥锁，避免混用不同反馈更新过程中的电机状态。

本次未改变RL策略、控制参数或电机通信协议。既有独立项目验证不等于这份部署工程已完成真机验收。
