import RPi.GPIO as GPIO
import time

# GPIO setup
SERVO_PIN = 14  # GPIO pin connected to the servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# PWM setup
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency
pwm.start(0)  # Start with duty cycle 0

# Function to set servo angle
def set_angle(angle):
    duty_cycle = 2 + (angle / 18)  # Convert angle to duty cycle (0-180 degrees)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)  # Stop sending signal

try:
    while True:
        angle = int(input("Enter angle (0 to 180): "))
        if 0 <= angle <= 180:
            set_angle(angle)
        else:
            print("Please enter a valid angle between 0 and 180.")

except KeyboardInterrupt:
    print("\nExiting...")
    pwm.stop()
    GPIO.cleanup()
