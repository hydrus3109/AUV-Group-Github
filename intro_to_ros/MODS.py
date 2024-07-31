from gpiozero import Servo, LED
from time import sleep

PWM_PIN = 18
LED_PIN = 16

servo = Servo(PWM_PIN)
led = LED(LED_PIN)

servo.ensure_pin_factory()
led.ensure_pin_factory()

while True:
    servo.min()
    led.on()
    sleep(1)
    servo.mid()
    sleep(1)
    servo.max()
    led.off()
    sleep(1)
