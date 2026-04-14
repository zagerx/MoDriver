#!/usr/bin/env python3
import struct
import can

bus = can.interface.Bus(channel="can0", interface="socketcan")
node_id = 21

params = [
    (0x2009, 0x01, "wheel_radius", "float"),
    (0x2009, 0x02, "gear_ratio", "float"),
    (0x2009, 0x03, "pole_pairs", "float"),
    (0x2009, 0x04, "direction", "float"),
    (0x2009, 0x05, "encoder_resolution", "uint16"),
    (0x2009, 0x06, "encoder_offset", "uint16"),
    (0x2009, 0x07, "encoder_offset_frac", "float"),
    (0x2009, 0x08, "A_chn_offset", "uint16"),
    (0x2009, 0x09, "B_chn_offset", "uint16"),
    (0x2009, 0x0A, "C_chn_offset", "uint16"),
    (0x2009, 0x0B, "gain_phase", "float"),
    (0x2009, 0x0C, "gain_i_bus", "float"),
    (0x2009, 0x0D, "gain_v_bus", "float"),
    (0x2009, 0x0E, "acc_max", "float"),
    (0x2009, 0x0F, "vmax", "float"),
    (0x2009, 0x10, "D_kp", "float"),
    (0x2009, 0x11, "D_ki", "float"),
    (0x2009, 0x12, "D_kd", "float"),
    (0x2009, 0x13, "D_limit", "float"),
    (0x2009, 0x14, "Q_kp", "float"),
    (0x2009, 0x15, "Q_ki", "float"),
    (0x2009, 0x16, "Q_kd", "float"),
    (0x2009, 0x17, "Q_limit", "float"),
    (0x2009, 0x18, "vel_kp", "float"),
    (0x2009, 0x19, "vel_ki", "float"),
    (0x2009, 0x1A, "vel_kd", "float"),
    (0x2009, 0x1B, "vel_limit", "float"),
    (0x2009, 0x1C, "pos_kp", "float"),
    (0x2009, 0x1D, "pos_ki", "float"),
    (0x2009, 0x1E, "pos_kd", "float"),
    (0x2009, 0x1F, "pos_limit", "float"),
    (0x2009, 0x20, "rs", "float"),
    (0x2009, 0x21, "ls", "float"),
    (0x2009, 0x22, "is_calibrated", "uint8"),
    (0x2009, 0x23, "crc16", "uint16"),
]

group_map = {
    "feedback_param": ["wheel_radius", "gear_ratio", "pole_pairs", "direction", "encoder_resolution", "encoder_offset", "encoder_offset_frac"],
    "currsmp_param": ["A_chn_offset", "B_chn_offset", "C_chn_offset", "gain_phase", "gain_i_bus", "gain_v_bus"],
    "traj_param": ["acc_max", "vmax"],
    "foc_d": ["D_kp", "D_ki", "D_kd", "D_limit"],
    "foc_q": ["Q_kp", "Q_ki", "Q_kd", "Q_limit"],
    "foc_vel": ["vel_kp", "vel_ki", "vel_kd", "vel_limit"],
    "foc_pos": ["pos_kp", "pos_ki", "pos_kd", "pos_limit"],
    "electrical_param": ["rs", "ls"],
    "calibration": ["is_calibrated"],
    "crc": ["crc16"],
}

results = {}

for idx, sub, name, dtype in params:
    req = can.Message(
        arbitration_id=0x600 + node_id,
        data=[0x40, idx & 0xFF, (idx >> 8) & 0xFF, sub, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False
    )
    bus.send(req)
    resp = bus.recv(timeout=1.0)

    if resp is None or resp.arbitration_id != (0x580 + node_id):
        print(f"错误: 读取 {name} (0x{idx:04X}:{sub:02X}) 超时")
        bus.shutdown()
        exit(1)

    scs = (resp.data[0] >> 5) & 0x07
    if scs != 2:
        print(f"错误: 读取 {name} (0x{idx:04X}:{sub:02X}) 返回异常 scs={scs}")
        bus.shutdown()
        exit(1)

    if dtype == "float":
        value = struct.unpack('<f', resp.data[4:8])[0]
    elif dtype == "uint16":
        value = struct.unpack('<H', resp.data[4:6])[0]
    elif dtype == "uint8":
        value = resp.data[4]
    else:
        value = resp.data[4:8]

    results[name] = value

bus.shutdown()

for group_name, keys in group_map.items():
    print(f"\n[{group_name}]")
    for k in keys:
        v = results[k]
        if isinstance(v, float):
            print(f"  {k:20s} = {v}")
        else:
            print(f"  {k:20s} = {v}")
