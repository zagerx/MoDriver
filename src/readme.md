# MoDrive

基于 **STM32G473** 的 FOC 伺服电机驱动固件，支持 CANopen CiA 402 协议，可通过 CAN 总线进行参数配置和运动控制。

## 主要功能

**电机控制**
- FOC 三环控制：电流环（20kHz）/ 速度环（4kHz）/ 位置环（100Hz）
- SVPWM 空间矢量调制，电流解耦
- 开环控制（强制对齐、开环旋转）
- S 曲线轨迹规划
- 齿槽效应补偿（360 点/圈 anticogging 校准与实时补偿）

**校准系统**
- 编码器零点偏移校准（含小数偏移）
- 极对数自动识别、方向自动检测
- 电流采样偏移校准
- RL 参数测量（电阻/电感）

**通信协议**
- 完整 CANopenNode 协议栈，节点 ID = 21
- 支持 CiA 402 模式：Profile Position / Profile Velocity / Homing / CSP / CSV / CST
- SDO 参数读写，PDO 实时通信
- 支持 SYNC、Heartbeat、NMT、EMCY

**参数管理**
- 34 个电机可调参数（PID 增益、编码器参数、电气参数等）
- 对象字典 0x2009 统一访问
- Flash 持久化存储（支持 0x1010 保存 / 0x1011 恢复）
- 上电自动加载

**主机工具**
- `read_motorparams.py`：通过 SDO 读取全部参数
- `write_motorlibparams.py`：通过 SDO 下发参数（支持单参数更新）
- PyQt6 上位机（`canopen/` 目录）

## 信息

- **作者**：zager
- **版本**：v0.1.0
- **更新**：2026-04-14
- **状态**：积极开发中
