import RPi.GPIO as GPIO
import time

# Set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# --- Define Pins ---

# Servo Motor (for panning a camera/sensor)
SERVO_PIN = 18

# Ultrasonic Sensor (HC‑SR04)
TRIG_PIN = 23
ECHO_PIN = 24  # Use a voltage divider to step down 5V to 3.3V

# Motor Driver (L298N) Motor Control Pins
LEFT_IN1 = 5
LEFT_IN2 = 6
RIGHT_IN3 = 13
RIGHT_IN4 = 19

# --- Setup GPIO ---

# Servo setup
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM for servo
servo.start(0)

# Ultrasonic sensor setup
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Motor control pins setup
motor_pins = [LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# --- Define Functions ---

# Motor control functions
def move_forward():
    GPIO.output(LEFT_IN1, True)
    GPIO.output(LEFT_IN2, False)
    GPIO.output(RIGHT_IN3, True)
    GPIO.output(RIGHT_IN4, False)
    print("Moving forward")

def move_backward():
    GPIO.output(LEFT_IN1, False)
    GPIO.output(LEFT_IN2, True)
    GPIO.output(RIGHT_IN3, False)
    GPIO.output(RIGHT_IN4, True)
    print("Moving backward")

def stop_motors():
    for pin in motor_pins:
        GPIO.output(pin, False)
    print("Motors stopped")

# Servo control function
def set_servo_angle(angle):
    duty_cycle = angle / 18 + 2
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for servo to move
    servo.ChangeDutyCycle(0)  # Stop sending signal to prevent jitter

# Ultrasonic sensor measurement function with timeout handling
def measure_distance():
    # Ensure trigger is low
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    # Send a 10µs pulse to trigger
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    # Wait for echo to start (timeout to prevent infinite loop)
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if pulse_start - start_time > 0.1:  # Timeout after 100ms
            print("Echo signal not received (Start timeout).")
            return None

    # Wait for echo to end (timeout to prevent infinite loop)
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if pulse_end - start_time > 0.1:  # Timeout after 100ms
            print("Echo signal stuck high (End timeout).")
            return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert time to distance (cm)
    return round(distance, 2)

# --- Main Loop ---

try:
    while True:
        # Measure distance using the ultrasonic sensor
        distance = measure_distance()
        if distance is None:
            print("Skipping measurement due to sensor timeout.")
            continue

        print(f"Measured Distance: {distance} cm")
        threshold = 20  # Threshold distance in centimeters

        # Motor control based on distance measured
        if distance > threshold:
            move_forward()
        else:
            stop_motors()
            print("Obstacle detected! Stopping.")

        # Perform a servo sweep for surveillance
        for angle in range(0, 181, 45):
            set_servo_angle(angle)
            time.sleep(0.2)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Cleanup all used resources
    servo.stop()
    stop_motors()
    GPIO.cleanup()
    print("GPIO cleaned up. Exiting.")
