#!/usr/bin/env python3
"""
test.py - 目标位置速度随机测试工具

该工具会在开始随机测试前自动发送初始化指令:
1. "state:1.0" - 设置状态为1.0
2. 等待20ms
3. "mode:1" - 设置模式为1
4. 等待20ms
5. 开始持续发送随机位置速度指令

用法:
    # 随机测试：先发送初始化指令，然后持续发送随机位置速度指令
    python3 test.py
    
    # 自定义串口
    python3 test.py --port /dev/ttyACM1 --baudrate 115200
    
    # 限制发送次数
    python3 test.py --count 10

随机指令格式: "tar:200.0,100.0" (位置mm,速度mm/s)
位置范围: -8000.0 到 8000.0 mm
速度范围: 50.0 到 5000.0 mm/s
间隔时间: 10ms 到 3s 随机
"""

import sys
import os
import time
import random
import argparse

# 默认配置
DEFAULT_PORT = '/dev/ttyACM0'
DEFAULT_BAUDRATE = 115200
MIN_POSITION = -8000.0
MAX_POSITION = 8000.0
MIN_VELOCITY = 50.0
MAX_VELOCITY = 5000.0
MIN_INTERVAL = 0.01  # 10ms
MAX_INTERVAL = 3.0   # 5s

def check_and_open_serial(port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
    """
    检查并打开串口
    
    Args:
        port: 串口设备路径
        baudrate: 波特率
        
    Returns:
        serial.Serial: 打开的串口对象
        
    Raises:
        Exception: 包含具体错误信息的异常
    """
    try:
        import serial
    except ImportError:
        raise Exception("未找到 pyserial 库，请安装: pip install pyserial")
    
    # 检查串口设备是否存在
    if not os.path.exists(port):
        raise Exception(f"串口设备 {port} 不存在\n"
                       "请检查:\n"
                       "1. 设备是否连接\n"
                       "2. 串口设备名称是否正确\n"
                       "3. 尝试使用: ls /dev/tty* 查看可用串口")
    
    # 检查是否有访问权限
    if not os.access(port, os.R_OK | os.W_OK):
        raise Exception(f"没有权限访问串口设备 {port}\n"
                       "请尝试:\n"
                       "1. 使用 sudo 运行脚本\n"
                       "2. 将用户添加到 dialout 组: sudo usermod -a -G dialout $USER")
    
    # 尝试打开串口
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        # 等待串口稳定
        time.sleep(0.1)
        return ser
        
    except serial.SerialException as e:
        # 分析常见的串口异常
        error_msg = f"串口打开失败: {e}\n"
        
        if "Permission denied" in str(e):
            error_msg += "权限被拒绝，请尝试:\n"
            error_msg += "1. 使用 sudo 运行脚本\n"
            error_msg += "2. 将用户添加到 dialout 组: sudo usermod -a -G dialout $USER\n"
            error_msg += "3. 然后重新登录或重启系统"
        elif "Device or resource busy" in str(e):
            error_msg += "串口设备正忙，可能被其他程序占用\n"
            error_msg += "请关闭可能使用该串口的其他程序"
        elif "No such file or directory" in str(e):
            error_msg += "串口设备不存在\n"
            error_msg += "请检查设备连接和设备名称"
        else:
            error_msg += "请检查:\n"
            error_msg += "1. 设备是否连接正确\n"
            error_msg += "2. 波特率设置是否正确\n"
            error_msg += "3. 串口线是否完好"
            
        raise Exception(error_msg)

def send_command(ser, command):
    """
    通过串口发送命令
    
    Args:
        ser: 已打开的串口对象
        command: 要发送的命令字符串
    """
    # 发送命令，添加换行符作为结束标志
    data_to_send = command + '\n'
    ser.write(data_to_send.encode('utf-8'))
    print(f"[{time.strftime('%H:%M:%S')}] 已发送: {command}")
    
    # 可选：读取响应
    time.sleep(0.05)  # 短暂等待设备响应
    if ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"[{time.strftime('%H:%M:%S')}] 设备响应: {response}")


def send_initialization_commands(ser):
    """
    发送初始化指令序列
    
    Args:
        ser: 已打开的串口对象
    """
    print("\n=== 发送初始化指令 ===")
    
    # 发送 state:1.0
    send_command(ser, "state:1.0")
    time.sleep(0.02)  # 等待20ms
    
    # 发送 mode:1
    send_command(ser, "mode:1")
    time.sleep(0.02)  # 等待20ms
    
    print("=== 初始化指令发送完成 ===")

def generate_random_command():
    """
    生成随机的位置速度指令
    
    Returns:
        str: 格式为 "tar:position,velocity" 的指令字符串
    """
    position = round(random.uniform(MIN_POSITION, MAX_POSITION), 1)
    velocity = round(random.uniform(MIN_VELOCITY, MAX_VELOCITY), 1)
    return f"tar:{position},{velocity}"

def random_test_mode(ser, count=None):
    """
    随机测试模式：先发送初始化指令，然后持续发送随机指令
    
    Args:
        ser: 已打开的串口对象
        count: 发送次数限制，None表示无限发送
    """
    print("进入随机测试模式...")
    
    # 首先发送初始化指令
    send_initialization_commands(ser)
    
    print("=== 开始随机测试 ===\n")
    
    print(f"位置范围: {MIN_POSITION} 到 {MAX_POSITION} mm")
    print(f"速度范围: {MIN_VELOCITY} 到 {MAX_VELOCITY} mm/s")
    print(f"间隔时间: {MIN_INTERVAL} 到 {MAX_INTERVAL} 秒")
    print("按 Ctrl+C 停止测试\n")
    
    sent_count = 0
    
    try:
        while True:
            if count is not None and sent_count >= count:
                print(f"已达到指定发送次数 {count}，测试结束")
                break
                
            # 生成随机指令
            command = generate_random_command()
            
            # 发送指令
            send_command(ser, command)
            sent_count += 1
            
            # 随机间隔
            interval = random.uniform(MIN_INTERVAL, MAX_INTERVAL)
            print(f"等待 {interval:.2f} 秒后发送下一条指令...\n")
            
            # 等待间隔时间，但允许中断
            wait_start = time.time()
            while time.time() - wait_start < interval:
                time.sleep(0.1)  # 每0.1秒检查一次，以便响应Ctrl+C
                
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        print(f"总共发送了 {sent_count} 条指令")



def main():
    parser = argparse.ArgumentParser(
        description='目标位置速度随机测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  %(prog)s
  %(prog)s --port /dev/ttyACM1 --baudrate 115200
  %(prog)s --count 10
        '''
    )
    
    # 参数
    parser.add_argument(
        '--port',
        default=DEFAULT_PORT,
        help=f'串口设备路径 (默认: {DEFAULT_PORT})'
    )
    parser.add_argument(
        '--baudrate', '-b',
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f'串口波特率 (默认: {DEFAULT_BAUDRATE})'
    )
    parser.add_argument(
        '--count', '-c',
        type=int,
        help='发送指令次数限制，默认无限发送'
    )
    
    args = parser.parse_args()
    
    # 尝试打开串口
    try:
        ser = check_and_open_serial(args.port, args.baudrate)
        print(f"串口已打开: {args.port}, 波特率: {args.baudrate}")
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)
    
    try:
        # 运行随机测试模式
        random_test_mode(ser, args.count)
            
    except Exception as e:
        print(f"测试过程中出错: {e}")
        sys.exit(1)
        
    finally:
        # 关闭串口
        ser.close()
        print("串口已关闭")

if __name__ == '__main__':
    main()

