import RPi.GPIO as GPIO
import time

SERVO_PIN = 14  # Change GPIO pin to 14 (Pin 8 on Raspberry Pi)

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency for servo motor
pwm.start(0)

def set_angle(angle):
    duty_cycle = (angle / 18) + 2  # Convert angle to duty cycle
    print(f"Angle: {angle}, Duty Cycle: {duty_cycle:.2f}")
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)
    pwm.ChangeDutyCycle(0)  # Stop sending signal to prevent jitter

try:
    while True:
        angle = int(input("Enter angle (0-180): "))
        if 0 <= angle <= 180:
            set_angle(angle)
        else:
            print("Invalid angle! Enter between 0 and 180.")

except KeyboardInterrupt:
    print("\nStopping...")
    pwm.stop()
    GPIO.cleanup()
