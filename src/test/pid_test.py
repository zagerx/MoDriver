#!/usr/bin/env python3
"""
pi_debug.py - 调试参数下发工具

用法:
    python3 pi_debug.py "D_KpKiKd:1.2,23.3,43.5"
    python3 pi_debug.py "D_Kp:2.345"
    python3 pi_debug.py "Q_KpKi:1.5,2.8"

如果串口不可用，会打印具体原因并直接退出。
"""

import sys
import os
import time
import argparse

def check_and_open_serial(port='/dev/ttyACM0', baudrate=115200):
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
    print(f"已发送: {command}")
    
    # 可选：读取响应
    time.sleep(0.1)
    if ser.in_waiting:
        response = ser.readline().decode('utf-8').strip()
        print(f"设备响应: {response}")

def main():
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(
        description='向嵌入式设备发送调试参数',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  %(prog)s "D_KpKiKd:1.2,23.3,43.5"
  %(prog)s "D_Kp:2.345" --port /dev/ttyACM0
  %(prog)s "Q_KpKi:1.5,2.8" --baudrate 9600
        '''
    )
    
    parser.add_argument(
        'command',
        help='要发送的命令字符串，格式如: D_KpKiKd:1.2,23.3,43.5'
    )
    
    parser.add_argument(
        '--port', '-p',
        default='/dev/ttyACM0',
        help='串口设备路径 (默认: /dev/ttyACM0)'
    )
    
    parser.add_argument(
        '--baudrate', '-b',
        type=int,
        default=115200,
        help='串口波特率 (默认: 115200)'
    )
    
    args = parser.parse_args()
    
    # 基本格式检查
    if ':' not in args.command:
        print("错误: 命令格式不正确，应包含冒号分隔符")
        print("正确格式示例: D_KpKiKd:1.2,23.3,43.5")
        sys.exit(1)
    
    # 尝试打开串口
    try:
        ser = check_and_open_serial(args.port, args.baudrate)
        print(f"串口已打开: {args.port}, 波特率: {args.baudrate}")
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)
    
    # 发送命令
    try:
        send_command(ser, args.command)
        print("命令发送成功")
    except Exception as e:
        print(f"发送命令时出错: {e}")
        sys.exit(1)
    finally:
        # 关闭串口
        ser.close()
        print("串口已关闭")

if __name__ == '__main__':
    main()