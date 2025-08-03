import RPi.GPIO as GPIO
import time

# Define GPIO pins for IR sensors
IR_PINS = [4, 26, 19, 13, 6, 5]  # Adjust based on the number of sensors

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

def turn_right():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

# Main loop
try:
    while True:
        sensor_values = [GPIO.input(pin) for pin in IR_PINS]

        print("Sensor values:", sensor_values)  # Debugging

        if sensor_values == [0, 0, 0, 0, 1, 1, 1, 1]:  # Straight
            move_forward()
        elif sensor_values[:4] == [1, 1, 1, 1]:  # Left turn detected
            turn_left()
        elif sensor_values[-4:] == [1, 1, 1, 1]:  # Right turn detected
            turn_right()
        else:
            stop()  # Stop if no line detected

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
