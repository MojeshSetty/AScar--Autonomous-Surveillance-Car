import RPi.GPIO as GPIO
import time

# Pin Definitions
TRIG = 23  # Trigger pin
ECHO = 24  # Echo pin

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # Send a 10µs pulse to trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  
    GPIO.output(TRIG, False)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo start
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    # Wait for echo return
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    # Calculate distance (speed of sound = 343m/s)
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to cm
    return distance

# Main Loop
try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        time.sleep(0.5)  # Delay between readings

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()
