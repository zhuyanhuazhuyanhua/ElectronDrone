#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

# Pin Definitions
OUTPUT_PIN = 19  # BCM pin 

class GPIORosController:
    def __init__(self):
        rospy.init_node('gpio_controller', anonymous=True)
        rospy.loginfo("GPIO Controller Node Started")

        # Pin Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)

        # Create subscriber
        self.subscriber = rospy.Subscriber(
            '/gpio_control',
            Bool,
            self.gpio_callback,
            queue_size=10
        )

    def gpio_callback(self, msg):
        if msg.data:
            GPIO.output(OUTPUT_PIN, GPIO.HIGH)
            rospy.loginfo("Set GPIO HIGH")
        else:
            GPIO.output(OUTPUT_PIN, GPIO.LOW)
            rospy.loginfo("Set GPIO LOW")

    def cleanup(self):
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up and node destroyed.")


def main():
    node = GPIORosController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()


if __name__ == '__main__':
    main()
