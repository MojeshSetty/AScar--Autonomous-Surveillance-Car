import RPi.GPIO as GPIO
import time
import pigpio

# GPIO Pins (Change based on your wiring)
TRIG = 5
ECHO = 6
SERVO = 18
ENA = 25
ENB = 17
IN1 = 23
IN2 = 24
IN3 = 27 # Updated to avoid conflict
IN4 = 10  # Updated to avoid conflict

# Set up GPIO
GPIO.setwarnings(False)  # Disable warnings
GPIO.cleanup()           # Reset GPIO state
GPIO.setmode(GPIO.BCM)

# Setup Motor Driver Pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Setup Ultrasonic Sensor Pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Set up Servo
pi = pigpio.pi()
if not pi.connected:
    print("pigpio daemon is not running. Starting it...")
    exit()

# Motor Speed Control (PWM)
pwm_A = GPIO.PWM(ENA, 1000)  # Motor A PWM
pwm_B = GPIO.PWM(ENB, 1000)  # Motor B PWM
pwm_A.start(0)
pwm_B.start(0)

# Test 1: Blink an LED to confirm GPIO is working
print("Test 1: Blinking LED...")
test_pin = 17  # Example GPIO pin for LED
GPIO.setup(test_pin, GPIO.OUT)
GPIO.output(test_pin, True)
time.sleep(1)
GPIO.output(test_pin, False)
time.sleep(1)

# Test 2: Measure distance using Ultrasonic Sensor
def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to cm
    return round(distance, 2)

print("Test 2: Measuring Distance...")
distance = get_distance()
print(f"Distance measured: {distance} cm")

# Test 3: Move motors forward (check motor driver)
def move_forward(speed):
    pwm_A.ChangeDutyCycle(speed)
    pwm_B.ChangeDutyCycle(speed)
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    GPIO.output(IN3, True)
    GPIO.output(IN4, False)

print("Test 3: Moving motors forward...")
move_forward(50)  # 50% speed
time.sleep(2)  # Move for 2 seconds
GPIO.cleanup()  # Stop motors

# Test 4: Rotate Servo (check servo motor)
print("Test 4: Rotating Servo...")
pi.set_servo_pulsewidth(SERVO, 1500)  # Neutral position
time.sleep(1)
pi.set_servo_pulsewidth(SERVO, 2000)  # Rotate right (clockwise)
time.sleep(1)
pi.set_servo_pulsewidth(SERVO, 1000)  # Rotate left (counterclockwise)
time.sleep(1)
pi.set_servo_pulsewidth(SERVO, 1500)  # Reset to neutral position
time.sleep(1)

# Final cleanup
print("Test complete. Cleaning up GPIO...")
GPIO.cleanup()
pi.set_servo_pulsewidth(SERVO, 0)  # Stop servo
pi.stop()
