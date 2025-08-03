# Import required libraries
import RPi.GPIO as GPIO
import time

# GPIO Mode (BCM)
GPIO.setmode(GPIO.BCM)

# Set GPIO Pins for Ultrasonic Sensor
TRIG = 4
ECHO = 17

# Set GPIO Pins for Motor Driver (L298N)
ENA = 12  # PWM for Motor A
IN1 = 5
IN2 = 6
ENB = 13  # PWM for Motor B
IN3 = 16
IN4 = 26

# Set GPIO direction (IN / OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([ENA, ENB], GPIO.OUT)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)

# Setup PWM for Motor Speed Control
pwmA = GPIO.PWM(ENA, 1000)  # 1 kHz frequency
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(100)  # Start with full speed
pwmB.start(100)

# Function to measure distance
def measure_distance():
    # Trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIG, False)

    # Wait for the echo to start
    start_time = time.time()
    timeout = start_time + 0.02  # 20 ms timeout
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if start_time > timeout:
            print("Timeout: No echo received (Start)")
            return None
    
    # Wait for the echo to end
    stop_time = time.time()
    timeout = stop_time + 0.02  # 20 ms timeout
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
        if stop_time > timeout:
            print("Timeout: No echo received (End)")
            return None

    # Calculate the distance
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound (34300 cm/s)
    
    # Check for unrealistic values
    if distance > 400:
        print("Error: Distance out of range")
        return None

    return distance

# Function to set motor speed
def set_motor_speed(speedA, speedB):
    pwmA.ChangeDutyCycle(speedA)
    pwmB.ChangeDutyCycle(speedB)

# Function to move forward
def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

# Function to stop motors
def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# Main loop to measure distance and control motors
try:
    move_forward()
    
    while True:
        distance = measure_distance()
        
        if distance is not None:
            print(f"Distance: {distance:.2f} cm")

            # Speed Adjustment Based on Distance
            if distance > 30:
                set_motor_speed(100, 100)  # Full Speed
            elif 15 <= distance <= 30:
                set_motor_speed(50, 50)  # Half Speed
            else:
                set_motor_speed(0, 0)  # Stop Motors

        else:
            print("No valid measurement.")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
