import RPi.GPIO as GPIO
import time
from gpiozero import Servo
from gpiozero import DistanceSensor
from time import sleep

# Define GPIO pins
TRIG_PIN = 17  # GPIO for Ultrasonic Trigger
ECHO_PIN = 18  # GPIO for Ultrasonic Echo
SERVO_PIN = 12  # GPIO for Servo Motor
IN1 = 23  # Motor 1 control
IN2 = 24
IN3 = 27  # Motor 2 control
IN4 = 10
ENA = 25  # Enable pin for motor speed control
ENB = 16

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# PWM setup for speed control
pwmA = GPIO.PWM(ENA, 1000)  # Frequency 1kHz
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)

# Servo setup
servo = Servo(SERVO_PIN)
servo.value = 0  # Center position

# Global variables
MAX_SPEED = 100  # Max speed for motors
BASE_SPEED = 50  # Base speed when moving
SAFE_DISTANCE = 20  # Minimum distance to avoid obstacle
turn_duration = 1.0  # Time to turn
HIGH_SPEED = 70


def read_distance():
    """Measure distance using ultrasonic sensor."""
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to cm
    return round(distance, 2)


def set_motor_speed(speed):
    """Set motor speed dynamically based on distance."""
    speed = min(max(speed, 0), MAX_SPEED)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)


def move_forward():
    """Move car forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed(HIGH_SPEED)


def move_backward():
    """Move car backward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_motor_speed(HIGH_SPEED)


def turn_right():
    """Pivot right by making left wheel move forward and right wheel move backward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)  # Left wheel moves forward
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)  # Right wheel moves backward
    set_motor_speed(BASE_SPEED)
    time.sleep(turn_duration)  # Allow time for turning
    stop()


def turn_left():
    """Pivot left by making left wheel move backward and right wheel move forward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)  # Left wheel moves backward
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)  # Right wheel moves forward
    set_motor_speed(BASE_SPEED)
    time.sleep(turn_duration)  # Allow time for turning
    stop()

def stop():
    """Stop all motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed(0)


def look_right():
    """Rotate servo to right and measure distance."""
    servo.value = -1  # Rotate to right
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0  # Reset to center
    return distance


def look_left():
    """Rotate servo to left and measure distance."""
    servo.value = 1  # Rotate to left
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0  # Reset to center
    return distance


def main():
    """Main loop to control the autonomous vehicle."""
    try:
        while True:
            distance = read_distance()
            print(f"Distance: {distance} cm")

            if distance <= SAFE_DISTANCE:
                stop()
                time.sleep(0.2)
                """move_backward()"""

                right_distance = look_right()
                left_distance = look_left()
                
                print(f"Right_distance:{right_distance} cm ")
                print(f"Left Distance:{left_distance} cm")
    
                if right_distance > left_distance:
                    turn_right()
                else:
                    turn_left()
                move_forward()

            else:
                speed = BASE_SPEED + (MAX_SPEED - BASE_SPEED) * (distance / 100)
                set_motor_speed(speed)
                move_forward()

            time.sleep(0.1)

    except KeyboardInterrupt:
        stop()
        GPIO.cleanup()
        print("Program stopped")


if __name__ == "__main__":
    main()
