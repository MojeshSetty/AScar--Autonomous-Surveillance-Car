import RPi.GPIO as GPIO
import time

# Define motor control pins
IN1 = 17  # Motor 1
IN2 = 27
IN3 = 22  # Motor 2
IN4 = 23
ENA = 18  # Speed Control (PWM)
ENB = 25

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Setup PWM for speed control
pwm_a = GPIO.PWM(ENA, 100)  # 100 Hz frequency
pwm_b = GPIO.PWM(ENB, 100)
pwm_a.start(70)  # 70% duty cycle
pwm_b.start(70)

def move_forward(duration=2):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(duration)

def turn_right(duration=0.8):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    time.sleep(duration)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    for _ in range(4):  # Move in a square (4 sides)
        move_forward()
        turn_right()
    
    stop()
    
except KeyboardInterrupt:
    print("Stopping the car...")
    stop()
    GPIO.cleanup()
