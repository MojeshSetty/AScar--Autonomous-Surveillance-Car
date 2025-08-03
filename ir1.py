import RPi.GPIO as GPIO
import time

# Define GPIO pins for IR sensors
IR_PINS = [4, 26, 19, 13, 6, 5]  # Adjust based on the actual wiring

# Define motor control pins
LEFT_MOTOR_FORWARD = 23
LEFT_MOTOR_BACKWARD = 24
RIGHT_MOTOR_FORWARD = 27
RIGHT_MOTOR_BACKWARD = 10

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PINS, GPIO.IN)
GPIO.setup([LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD], GPIO.OUT)

# Motor control functions
def move_forward():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def turn_left():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def turn_right():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

try:
    while True:
        sensor_values = [GPIO.input(pin) for pin in IR_PINS]
        print("Sensor values:", sensor_values)  # Debugging

        # Assuming 6 IR sensors
        left_sensors = sensor_values[:3]   # First 3 sensors (left side)
        right_sensors = sensor_values[-3:]  # Last 3 sensors (right side)
        center_sensors = sensor_values[2:4]  # Middle sensors

        if center_sensors == [1, 1]:  # Move forward if middle sensors detect the line
            move_forward()
        elif left_sensors == [1, 1, 1]:  # Turn left if all left sensors detect the line
            turn_left()
        elif right_sensors == [1, 1, 1]:  # Turn right if all right sensors detect the line
            turn_right()
        else:
            stop()  # Stop if no line is detected

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
