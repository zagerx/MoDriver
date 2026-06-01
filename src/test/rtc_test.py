#!/usr/bin/env python3
"""
rtc_test.py - RTC 串口授时测试工具

用法:
    python3 rtc_test.py --days 15493 --ms 45000000
    python3 rtc_test.py --now              # 用当前系统时间自动计算 days+ms
"""

import sys
import time
import argparse

# 1984-01-01 的 Unix 时间戳（秒）
EPOCH_1984 = 441763200


def check_and_open_serial(port='/dev/ttyACM0', baudrate=115200):
    """检查并打开串口"""
    try:
        import serial
    except ImportError:
        raise Exception("未找到 pyserial 库，请安装: pip install pyserial")

    if not __import__('os').path.exists(port):
        raise Exception(f"串口设备 {port} 不存在")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        time.sleep(0.1)
        return ser
    except Exception as e:
        raise Exception(f"串口打开失败: {e}")


def send_command(ser, cmd):
    """发送命令并读取响应"""
    ser.write((cmd + '\n').encode('utf-8'))
    time.sleep(0.05)
    if ser.in_waiting:
        return ser.readline().decode('utf-8').strip()
    return ""


def current_time_to_canopen():
    """把当前系统时间转成 CANopen TIME 格式 (days, ms)"""
    now = int(time.time())
    delta = now - EPOCH_1984
    days = delta // 86400
    ms = (delta % 86400) * 1000
    return days, ms


def main():
    parser = argparse.ArgumentParser(description='RTC 串口授时测试')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0', help='串口设备路径')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='波特率')
    parser.add_argument('--days', type=int, help='自 1984-01-01 以来的天数')
    parser.add_argument('--ms', type=int, help='当天午夜以来的毫秒数 (0-86399999)')
    parser.add_argument('--now', action='store_true', help='使用当前系统时间')
    args = parser.parse_args()

    if args.now:
        days, ms = current_time_to_canopen()
    else:
        if args.days is None or args.ms is None:
            print("错误: 请指定 --days 和 --ms，或使用 --now")
            sys.exit(1)
        days, ms = args.days, args.ms

    print(f"测试时间: days={days}, ms={ms}")

    try:
        ser = check_and_open_serial(args.port, args.baudrate)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)

    # Step 1: 下发 rtc_set
    print(f"\n[1/3] 下发 rtc_set:{days},{ms}")
    resp = send_command(ser, f"rtc_set:{days},{ms}")
    print(f"      响应: {resp}")
    if resp != "OK":
        print("❌ 设置失败!")
        ser.close()
        sys.exit(1)

    # Step 2: 读取 rtc_get
    print(f"\n[2/3] 下发 rtc_get:")
    resp = send_command(ser, "rtc_get:")
    print(f"      响应: {resp}")

    # Step 3: 比对
    print(f"\n[3/3] 验证:")
    expected = f"{days},{ms}"
    if resp == expected:
        print(f"✅ 验证通过")
    else:
        print(f"❌ 验证失败")
        print(f"   期望: {expected}")
        print(f"   实际: {resp}")

    ser.close()


if __name__ == '__main__':
    main()
