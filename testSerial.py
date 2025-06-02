#!/usr/bin/env python3
import serial
import time
import sys

def test_serial():
    try:
        # 打开串口
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print(f"串口已打开: {ser.port}")
        print(f"波特率: {ser.baudrate}")
        print(f"超时: {ser.timeout}")
        
        # 等待串口稳定
        time.sleep(0.1)
        
        # 要发送的数据
        data = bytes([0x02, 0x01, 0x01])
        print(f"准备发送数据: {' '.join([f'0x{b:02X}' for b in data])}")
        
        # 发送数据
        bytes_written = ser.write(data)
        print(f"已发送 {bytes_written} 字节")
        
        # 强制刷新缓冲区
        ser.flush()
        print("缓冲区已刷新")
        
        # 等待一下，看是否有返回数据
        time.sleep(0.1)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"收到响应: {' '.join([f'0x{b:02X}' for b in response])}")
        else:
            print("没有收到响应数据")
            
        # 关闭串口
        ser.close()
        print("串口已关闭")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        return False
    except Exception as e:
        print(f"其他错误: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("=== 串口测试脚本 ===")
    print("测试端口: /dev/ttyACM0")
    print("发送数据: 0x02 0x01 0x00")
    print("==================")
    
    if test_serial():
        print("测试完成")
    else:
        print("测试失败")
        sys.exit(1)