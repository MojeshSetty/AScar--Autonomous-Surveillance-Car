import RPi.GPIO as GPIO
import time

# Define motor control pins
IN1 = 23  # Motor 1 control
IN2 = 24
IN3 = 27  # Motor 2 control
IN4 = 10
ENA = 25  # Enable pin for motor speed control
ENB = 16

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Set up PWM for motor speed control
pwm_A = GPIO.PWM(ENA, 1000)
pwm_B = GPIO.PWM(ENB, 1000)
pwm_A.start(50)  # 50% duty cycle
pwm_B.start(50)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)  # Keep right motor stopped

def turn_right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)  # Keep left motor stopped
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    turn_left()
    time.sleep(1)
    turn_left()
    time.sleep(1)
    turn_left()
    time.sleep(1)
    turn_left()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    turn_right()
    time.sleep(1)
    stop()
except KeyboardInterrupt:
    GPIO.cleanup()
finally:
    GPIO.cleanup()
