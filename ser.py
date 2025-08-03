import RPi.GPIO as GPIO
import time

# Define GPIO pin for the servo
SERVO_PIN = 18

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Set up PWM for 50Hz frequency
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    duty_cycle = (angle / 18.0) + 2.5  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.05)  # Small delay to allow movement

try:
    while True:
        # Sweep from 0 to 180 degrees
        for angle in range(0, 181, 5):
            set_angle(angle)
        
        # Sweep from 180 to 0 degrees
        for angle in range(180, -1, -5):
            set_angle(angle)

except KeyboardInterrupt:
    print("Servo movement stopped.")
    pwm.stop()
    GPIO.cleanup()
