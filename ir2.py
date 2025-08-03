import RPi.GPIO as GPIO
import time

# Define GPIO pins for IR sensors
IR_PINS = [4, 26, 19, 13, 6, 5]  # Adjust based on your actual wiring

# Define motor control pins (L298N)
LEFT_MOTOR_FORWARD = 23
LEFT_MOTOR_BACKWARD = 24
RIGHT_MOTOR_FORWARD = 27
RIGHT_MOTOR_BACKWARD = 10
LEFT_MOTOR_PWM = 18  # PWM pin for speed control
RIGHT_MOTOR_PWM = 12  # PWM pin for speed control

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PINS, GPIO.IN)
GPIO.setup([LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD], GPIO.OUT)
GPIO.setup([LEFT_MOTOR_PWM, RIGHT_MOTOR_PWM], GPIO.OUT)

# PWM setup (Set frequency to 100 Hz)
left_pwm = GPIO.PWM(LEFT_MOTOR_PWM, 100)
right_pwm = GPIO.PWM(RIGHT_MOTOR_PWM, 100)
left_pwm.start(0)  # Start with 0% duty cycle
right_pwm.start(0)

# Motor control functions with PWM
def move_forward(speed=50):
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    left_pwm.ChangeDutyCycle(speed)
    right_pwm.ChangeDutyCycle(speed)

def turn_left(speed=40):
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    left_pwm.ChangeDutyCycle(0)  # Stop left wheel
    right_pwm.ChangeDutyCycle(speed)  # Move right wheel

def turn_right(speed=40):
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    left_pwm.ChangeDutyCycle(speed)  # Move left wheel
    right_pwm.ChangeDutyCycle(0)  # Stop right wheel

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

# Main loop with debugging
try:
    while True:
        sensor_values = [GPIO.input(pin) for pin in IR_PINS]
        print("Sensor values:", sensor_values)  # Debugging

        left_sensors = sensor_values[:3]   # First 3 sensors (left side)
        right_sensors = sensor_values[-3:]  # Last 3 sensors (right side)
        center_sensors = sensor_values[2:4]  # Middle sensors

        if center_sensors == [1, 1]:  # Move forward if middle sensors detect the line
            move_forward(speed=60)
        elif left_sensors == [1, 1, 1]:  # Turn left if all left sensors detect the line
            turn_left(speed=50)
        elif right_sensors == [1, 1, 1]:  # Turn right if all right sensors detect the line
            turn_right(speed=50)
        else:
            stop()  # Stop if no line is detected

        time.sleep(0.1)

except KeyboardInterrupt:
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
