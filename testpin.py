#!/usr/bin/env python3
import RPi.GPIO as GPIO

def check_pwm_pins():
    GPIO.setmode(GPIO.BCM)
    
    print(f"板子型号: {GPIO.model}")
    print("检查可用的PWM引脚:")
    
    # 测试常见的引脚
    
    available_pwm = []
    
    for pin in range(0, 28):
        try:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, 50)
            pwm.start(10)  # 启动PWM，初始占空比为0%
            available_pwm.append(pin)
            print(f"引脚 {pin}: ✓ 支持PWM")
            # 不启动PWM，只是测试是否可以创建
        except Exception as e:
            print(f"引脚 {pin}: ✗ 不支持PWM - {str(e)}")
        finally:
            try:
                GPIO.cleanup(pin)
            except:
                pass
    
    print(f"\n可用的PWM引脚: {available_pwm}")
    GPIO.cleanup()

if __name__ == '__main__':
    check_pwm_pins()