import RPi.GPIO as GPIO
import time

# Set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# --- Define Pins ---

# Servo Motor (for panning a camera/sensor)
SERVO_PIN = 18

# Ultrasonic Sensor (HC‑SR04)
TRIG_PIN = 23
ECHO_PIN = 24  # Remember: use a voltage divider on Echo for 5V -> 3.3V conversion

# Motor Driver (L298N) Motor Control Pins (avoid GPIO 22)
# Left Motor
LEFT_IN1 = 5
LEFT_IN2 = 6
# Right Motor
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
    # Calculate duty cycle (adjust mapping for your servo if necessary)
    duty_cycle = angle / 18 + 2
    GPIO.output(SERVO_PIN, True)
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Allow time for servo to reach position
    GPIO.output(SERVO_PIN, False)
    servo.ChangeDutyCycle(0)

# Ultrasonic sensor measurement function
def measure_distance():
    # Ensure trigger is low
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)
    
    # Send a 10µs pulse to trigger
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    # Wait for echo to start and end
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    # Calculate distance: speed of sound ~34300 cm/s, hence distance = (time * 17150) in cm
    distance = pulse_duration * 17150
    return round(distance, 2)

# --- Main Loop ---

try:
    while True:
        # Measure distance using the ultrasonic sensor
        distance = measure_distance()
        print(f"Measured Distance: {distance} cm")
        threshold = 20  # Example threshold in centimeters
        
        # Motor control based on distance measured
        if distance > threshold:
            move_forward()
        else:
            stop_motors()
            # Optional: Uncomment below to add reverse or turning logic
            # move_backward()
            # time.sleep(1)
            # stop_motors()
        
        # Optionally, perform a servo sweep for surveillance
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
