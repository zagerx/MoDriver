# Test 工具说明

本目录包含用于与 MoDrive 设备进行 CANopen 通信的 Python 调试工具。

> **前提条件**：运行前请确保系统已安装 `python-can`，且 CAN 接口（如 `can0`）已正确配置。
>
> ```bash
> sudo ip link set can0 up type can bitrate 1000000
> ```

---

## 1. `read_motorparams.py` — 读取电机参数

**功能**：通过 SDO 读取 OD 索引 `0x2009` 下的全部电机参数，并按分组格式化输出。

**用法**：
```bash
python3 test/read_motorparams.py
```

**输出示例**：
```
[feedback_param]
  wheel_radius         = 17.5
  gear_ratio           = 1.0
  ...

[foc_d]
  D_kp                 = 0.1
  D_ki                 = 800.0
  ...
```

---

## 2. `write_motorlibparams.py` — 下发电机参数

**功能**：通过 SDO 将参数写入设备 OD `0x2009`。参数来源可以是文件内定义的默认值，也可以是命令行传入的指定值。

### 2.1 下发所有默认值
```bash
python3 test/write_motorlibparams.py
```

### 2.2 仅更新指定参数
支持同时更新多个参数，格式为 `name=value`：
```bash
python3 test/write_motorlibparams.py wheel_radius=16.5
python3 test/write_motorlibparams.py wheel_radius=16.5 gear_ratio=2.0 D_kp=0.5
python3 test/write_motorlibparams.py is_calibrated=1
```

> **注意**：此脚本仅将参数写入设备的 RAM（OD_RAM）。若设备断电重启，参数会丢失，除非执行后续的 **保存到 Flash** 操作。

---

## 3. 保存参数到 Flash（持久化）

参数写入 RAM 后，如需掉电保存，必须通过 CANopen 标准 `Store parameters` 命令将其写入 Flash。

### 使用 `cansend` 发送保存指令
```bash
cansend can0 615#2310100373617665
```

**指令说明**：
- `615` = SDO Client->Server ID (`0x600 + 21`)
- `23` = Expedited download, 4 bytes
- `10 10` = OD Index `0x1010`（小端序，低字节在前）
- `03` = Sub-index `3`（应用参数）
- `73 61 76 65` = `"save"` 的 ASCII 小端序 (`0x65766173`)

如果保存成功，设备会立即回复：
```bash
can0  595   [8]  60 10 10 03 00 00 00 00
```

> **提示**：`0x1010:03` 对应 OD 中 `subIndexOD = 3` 的存储条目（即 `0x2009` 应用参数）。保存成功后，设备下次上电会自动从 Flash 加载这些参数。

---

## 4. 恢复默认参数

如需清除已保存的 Flash 数据并恢复默认值，可发送 `Restore default parameters` 命令：

```bash
cansend can0 615#231110036C6F6164
```

- `11 10` = OD Index `0x1011`（小端序，低字节在前）
- `03` = Sub-index `3`
- `6C 6F 61 64` = `"load"` 的 ASCII 小端序 (`0x64616F6C`)

执行后 Flash 中的持久化数据会被擦除，但**当前 RAM 中的值不会立即改变**。重启设备后，参数将恢复为编译时的默认值。

---

## 参数列表速查

| 参数名 | 子索引 | 类型 | 说明 |
|--------|--------|------|------|
| `wheel_radius` | 0x01 | float | 轮子半径 (mm) |
| `gear_ratio` | 0x02 | float | 减速比 |
| `pole_pairs` | 0x03 | float | 极对数 |
| `direction` | 0x04 | float | 旋转方向 |
| `encoder_resolution` | 0x05 | uint16 | 编码器分辨率 (CPR) |
| `encoder_offset` | 0x06 | uint16 | 编码器零位偏移（整数） |
| `encoder_offset_frac` | 0x07 | float | 编码器零位小数偏移 |
| `A_chn_offset` | 0x08 | uint16 | A 相电流采样偏移 |
| `B_chn_offset` | 0x09 | uint16 | B 相电流采样偏移 |
| `C_chn_offset` | 0x0A | uint16 | C 相电流采样偏移 |
| `gain_phase` | 0x0B | float | 相电流增益 |
| `gain_i_bus` | 0x0C | float | 母线电流增益 |
| `gain_v_bus` | 0x0D | float | 母线电压增益 |
| `acc_max` | 0x0E | float | 最大加速度 |
| `vmax` | 0x0F | float | 最大速度 |
| `D_kp` ~ `D_limit` | 0x10~0x13 | float | d 轴电流环 PID |
| `Q_kp` ~ `Q_limit` | 0x14~0x17 | float | q 轴电流环 PID |
| `vel_kp` ~ `vel_limit` | 0x18~0x1B | float | 速度环 PID |
| `pos_kp` ~ `pos_limit` | 0x1C~0x1F | float | 位置环 PID |
| `rs` | 0x20 | float | 定子电阻 |
| `ls` | 0x21 | float | 定子电感 |
| `is_calibrated` | 0x22 | uint8 | 校准成功标志 (0/1) |
| `crc16` | 0x23 | uint16 | 参数校验码 |
