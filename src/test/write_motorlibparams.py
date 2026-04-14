#!/usr/bin/env python3
"""
write_motorlibparams.py - 电机参数下发工具

用法:
    # 下发所有参数默认值
    python3 write_motorlibparams.py

    # 仅更新指定参数
    python3 write_motorlibparams.py wheel_radius=16.5
    python3 write_motorlibparams.py wheel_radius=16.5 gear_ratio=2.0 D_kp=0.5
"""

import struct
import sys
import can

bus = can.interface.Bus(channel="can0", interface="socketcan")
node_id = 21

# 参数定义: (index, subindex, name, dtype, default_value)
params = [
    (0x2009, 0x01, "wheel_radius", "float", 17.5),
    (0x2009, 0x02, "gear_ratio", "float", 1.0),
    (0x2009, 0x03, "pole_pairs", "float", 0.0),
    (0x2009, 0x04, "direction", "float", 0.0),
    (0x2009, 0x05, "encoder_resolution", "uint16", 0x0000),
    (0x2009, 0x06, "encoder_offset", "uint16", 0x0000),
    (0x2009, 0x07, "encoder_offset_frac", "float", 0.0),
    (0x2009, 0x08, "A_chn_offset", "uint16", 0x0000),
    (0x2009, 0x09, "B_chn_offset", "uint16", 0x0000),
    (0x2009, 0x0A, "C_chn_offset", "uint16", 0x0000),
    (0x2009, 0x0B, "gain_phase", "float", 0.006855),
    (0x2009, 0x0C, "gain_i_bus", "float", 0.0),
    (0x2009, 0x0D, "gain_v_bus", "float", 0.01059),
    (0x2009, 0x0E, "acc_max", "float", 1800.0),
    (0x2009, 0x0F, "vmax", "float", 15000.0),
    (0x2009, 0x10, "D_kp", "float", 0.1),
    (0x2009, 0x11, "D_ki", "float", 800.0),
    (0x2009, 0x12, "D_kd", "float", 0.0),
    (0x2009, 0x13, "D_limit", "float", 13.0),
    (0x2009, 0x14, "Q_kp", "float", 0.1),
    (0x2009, 0x15, "Q_ki", "float", 800.0),
    (0x2009, 0x16, "Q_kd", "float", 0.0),
    (0x2009, 0x17, "Q_limit", "float", 13.0),
    (0x2009, 0x18, "vel_kp", "float", 0.002),
    (0x2009, 0x19, "vel_ki", "float", 0.2),
    (0x2009, 0x1A, "vel_kd", "float", 0.0),
    (0x2009, 0x1B, "vel_limit", "float", 10.0),
    (0x2009, 0x1C, "pos_kp", "float", 400.0),
    (0x2009, 0x1D, "pos_ki", "float", 4000.0),
    (0x2009, 0x1E, "pos_kd", "float", 0.0),
    (0x2009, 0x1F, "pos_limit", "float", 15000.0),
    (0x2009, 0x20, "rs", "float", 0.0),
    (0x2009, 0x21, "ls", "float", 0.0),
    (0x2009, 0x22, "is_calibrated", "uint8", 0x00),
    (0x2009, 0x23, "crc16", "uint16", 0x0000),
]


def build_sdo_download_request(idx, sub, dtype, value):
    """构建 SDO 下载请求报文 (expedited transfer)"""
    if dtype == "float":
        data = struct.pack('<f', float(value))
        cmd = 0x23  # 4 bytes write
    elif dtype == "uint16":
        data = struct.pack('<H', int(value))
        cmd = 0x2B  # 2 bytes write
    elif dtype == "uint8":
        data = struct.pack('<B', int(value))
        cmd = 0x2F  # 1 byte write
    else:
        raise ValueError(f"不支持的数据类型: {dtype}")

    # 补零到 4 字节，避免访问 data[2]/data[3] 越界
    data = data.ljust(4, b'\x00')

    return can.Message(
        arbitration_id=0x600 + node_id,
        data=[cmd, idx & 0xFF, (idx >> 8) & 0xFF, sub, data[0], data[1], data[2], data[3]],
        is_extended_id=False
    )


def send_sdo_and_check(req, name):
    """发送 SDO 请求并检查响应"""
    bus.send(req)
    resp = bus.recv(timeout=1.0)

    if resp is None or resp.arbitration_id != (0x580 + node_id):
        print(f"错误: 写入 {name} (0x{req.data[1]:02X}{req.data[2]:02X}:{req.data[3]:02X}) 超时")
        bus.shutdown()
        sys.exit(1)

    scs = (resp.data[0] >> 5) & 0x07
    if scs != 3:
        print(f"错误: 写入 {name} 返回异常 scs={scs}, data={list(resp.data)}")
        bus.shutdown()
        sys.exit(1)

    print(f"  已写入 {name}")


def main():
    # 构建参数名到参数信息的映射
    param_map = {p[2]: p for p in params}

    # 决定要下发的参数列表
    if len(sys.argv) == 1:
        # 无命令行参数: 下发所有默认值
        print("下发所有参数默认值...")
        targets = [(p[2], p[4]) for p in params]
    else:
        # 有命令行参数: 解析 name=value
        print("下发指定参数...")
        targets = []
        for arg in sys.argv[1:]:
            if '=' not in arg:
                print(f"错误: 参数格式不正确 '{arg}'，应为 name=value")
                bus.shutdown()
                sys.exit(1)
            name, value_str = arg.split('=', 1)
            if name not in param_map:
                print(f"错误: 未知参数名 '{name}'")
                bus.shutdown()
                sys.exit(1)
            targets.append((name, value_str))

    # 逐个下发
    for name, value in targets:
        idx, sub, _, dtype, _ = param_map[name]
        req = build_sdo_download_request(idx, sub, dtype, value)
        send_sdo_and_check(req, name)

    bus.shutdown()
    print("完成")


if __name__ == '__main__':
    main()
