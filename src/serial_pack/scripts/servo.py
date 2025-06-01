#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys
import threading

class SoftwarePWM:
    def __init__(self, pin, frequency):
        self.pin = pin
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.duty_cycle = 0
        self.running = False
        self.thread = None
        
    def start(self, duty_cycle):
        self.duty_cycle = duty_cycle
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
        
    def change_duty_cycle(self, duty_cycle):
        self.duty_cycle = duty_cycle
        
    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        GPIO.output(self.pin, GPIO.LOW)
        
    def _run(self):
        while self.running:
            if self.duty_cycle > 0:
                high_time = self.period * (self.duty_cycle / 100.0)
                low_time = self.period - high_time
                
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(high_time)
                
                if low_time > 0:
                    GPIO.output(self.pin, GPIO.LOW)
                    time.sleep(low_time)
            else:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(self.period)

def angle_to_duty_cycle(angle):
    """
    将角度(0-180度)转换为占空比
    脉宽范围: 0.5ms - 2.5ms
    周期: 20ms
    """
    pulse_width = 0.5 + (angle / 180.0) * (2.5 - 0.5)
    duty_cycle = (pulse_width / 20.0) * 100
    return duty_cycle

def main():
    if len(sys.argv) != 2:
        print("用法: python3 servo.py <角度>")
        print("角度范围: 0-180")
        sys.exit(1)
    
    try:
        angle = float(sys.argv[1])
        if angle < 0 or angle > 180:
            print("错误: 角度必须在0-180之间")
            sys.exit(1)
    except ValueError:
        print("错误: 请输入有效的数字")
        sys.exit(1)
    
    output_pin = 26 # 默认使用引脚
    
    print(f"检测到板子型号: {GPIO.model}")
    print(f"使用GPIO引脚: {output_pin} (BOARD编号)")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    
    # 使用软件PWM
    pwm = SoftwarePWM(output_pin, 50)  # 50Hz
    duty_cycle = angle_to_duty_cycle(angle)
    
    print(f"设置舵机角度: {angle}°")
    print(f"脉宽: {0.5 + (angle/180.0) * 2.0:.2f}ms")
    print(f"占空比: {duty_cycle:.2f}%")
    print("PWM输出中... 按CTRL+C退出")
    
    try:
        pwm.start(duty_cycle)
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n程序退出")
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()