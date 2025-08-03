
import RPi.GPIO as GPIO
import time

# Define GPIO pins for Motor 1
IN1 = 23  # Motor 1 direction
IN2 = 24  # Motor 1 direction
ENA = 25  # Motor 1 speed control

# Define GPIO pins for Motor 2
IN3 = 27  # Motor 2 direction
IN4 = 10  # Motor 2 direction
ENB = 17  # Motor 2 speed control

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Setup Motor 1
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Setup Motor 2
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Initialize PWM for speed control
pwm1 = GPIO.PWM(ENA, 100)  # Motor 1 PWM
pwm2 = GPIO.PWM(ENB, 100)  # Motor 2 PWM

pwm1.start(0)  # Start with 0% duty cycle (motor off)
pwm2.start(0)

def motor1_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)

def motor1_backward(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)

def motor1_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)

def motor2_forward(speed=50):
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm2.ChangeDutyCycle(speed)

def motor2_backward(speed=50):
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm2.ChangeDutyCycle(speed)

def motor2_stop():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm2.ChangeDutyCycle(0)

try:
    while True:
        print("Motors Moving Forward")
        motor1_forward(70)
        motor2_forward(70)
        time.sleep(2)

        print("Motors Moving Backward")
        motor1_backward(70)
        motor2_backward(70)
        time.sleep(2)

        print("Motors Stopped")
        motor1_stop()
        motor2_stop()
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting Program")
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
