import RPi.GPIO as GPIO
import time

# Pin Definitions
TRIG = 23
ECHO = 24

ENA = 12  # PWM for Motor A
IN1 = 5
IN2 = 6
ENB = 13  # PWM for Motor B
IN3 = 16
IN4 = 26

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([ENA, ENB], GPIO.OUT)
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)

# PWM Setup
pwmA = GPIO.PWM(ENA, 1000)  # 1 kHz frequency
pwmB = GPIO.PWM(ENB, 1000)

pwmA.start(100)  # Start with full speed (100% duty cycle)
pwmB.start(100)

# Function to measure distance
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
    distance = (elapsed_time * 34300) / 2  # Speed of sound is 343m/s
    return distance

# Function to control motor speed
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

# Main Loop
try:
    move_forward()
    
    while True:
        distance = get_distance()
        
        print(f"Distance: {distance:.2f} cm")

        speedA, speedB = 100, 100  # Default Full Speed
        
        if distance < 60:
            speedA, speedB = 60, 60  # Reduce speed
        if distance < 50:
            speedA, speedB = 40, 40
        if distance < 40:
            speedA, speedB = 20, 20  
        if distance < 30:
            speedA, speedB = 0, 0  # Stop motors

        set_motor_speed(speedA, speedB)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
    stop_motors()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
