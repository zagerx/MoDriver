# MoDrive - STM32G4 电机驱动项目

## 项目概述

**MoDrive** 是基于 **STM32G473RCTx** 的完整 FOC 伺服电机驱动固件，采用模块化 CMake 构建，集成 CANopen CiA 402 协议栈，可通过 CAN 总线进行参数配置和运动控制。

## 已完成功能

### 1. 硬件抽象层 (Hardware)
- **MCU 外设初始化**：时钟、GPIO、ADC、DMA、TIM、FDCAN、SPI、UART
- **三相 PWM 驱动**：TIM1 高级定时器，支持互补输出和死区控制
- **编码器接口**：AS5047P 磁编码器 SPI 驱动
- **Flash 参数存储**：使用 Flash 最后两页（4KB），带擦除/写入/读取/校验接口

### 2. 电机控制核心 (motorlib)
- **FOC 算法**：Clark/Park 变换、电流解耦、SVPWM 空间矢量调制
- **三环控制**：
  - 电流环（d/q 轴 PID）
  - 速度环 PID
  - 位置环 PID
- **开环控制**：强制对齐、开环旋转
- **轨迹规划器**：S 曲线加减速规划
- **校准系统**：
  - 电流采样偏移校准
  - 编码器零点偏移校准（含小数偏移）
  - 极对数自动识别
  - 方向自动检测
  - RL 参数测量（电阻/电感）
- **保护系统**：过压、欠压、堵转保护

### 3. CANopen 通信与运动控制
- **CANopen 协议栈**：完整集成 CANopenNode，节点 ID = 21
- **SDO 服务**：支持通过 CAN 读写所有 OD 参数
- **PDO 支持**：RPDO/TPDO、SYNC、Heartbeat、NMT、EMCY
- **CiA 402 驱动协议**：
  - 完整 PDS 状态机（Not Ready → Switch On → Operation Enabled）
  - 支持控制模式：
    - Profile Position (pp, 模式 1)
    - Profile Velocity (pv, 模式 3)
    - Homing (hm, 模式 6)
    - Cyclic Synchronous Position (csp, 模式 8)
    - Cyclic Synchronous Velocity (csv, 模式 9)
    - Cyclic Synchronous Torque (cst, 模式 10)
  - OD 对象映射：0x6040 控制字、0x6041 状态字、0x6060 模式、0x6064 实际位置、0x606C 实际速度 等

### 4. 参数存储与管理
- **OD 0x2009 应用参数**：包含电机全部 34 个可调参数
  - 反馈参数（轮半径、减速比、极对数、方向、编码器分辨率/偏移）
  - 电流采样参数（三相偏移、增益）
  - 轨迹规划参数（最大加速度、最大速度）
  - FOC PID 参数（d/q 电流环、速度环、位置环）
  - 电机电气参数（Rs、Ls）
- **参数持久化**：
  - 支持 CANopen 标准 `0x1010`（Store Parameters）保存到 Flash
  - 支持 `0x1011`（Restore Default Parameters）恢复默认值
  - **上电自动从 Flash 加载参数**，覆盖 OD 默认值

### 5. 主机端 Python 工具链
- **`read_motorparams.py`**：通过 SDO 读取并打印 OD 0x2009 全部电机参数
- **`write_motorlibparams.py`**：通过 SDO 下发参数
  - 直接执行：下发所有默认值
  - 带参数执行：如 `wheel_radius=16.5` 仅更新指定参数

## 项目信息

- **项目维护者**：zager
- **项目版本**：v0.1.0
- **更新日期**：2026-04-14

---

*本项目仍在积极开发中*
