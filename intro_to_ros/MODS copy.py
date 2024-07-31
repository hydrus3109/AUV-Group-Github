import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep


class PWM(Node):
    PWM_PIN = 18
    
    def __init__(self):
        super().__init__("electromagnet")
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.PWM_PIN, GPIO.OUT) #Setup as output in
        self.pwm = GPIO.PWM(self.PWM_PIN, 50) #SEtup as pwm pin
        
        self.timer = self.create_timer(0.0, self.timer_callback)

    def timer_callback(self):
        for i in range (19):
            self.pwm.start(5*i)
            sleep(0.2)
        input('Press return to stop:') 
        self.pwm.stop()


def main(args=None):
    rclpy.init(args=args)
    pwm_node = PWM()
    rclpy.spin(pwm_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()