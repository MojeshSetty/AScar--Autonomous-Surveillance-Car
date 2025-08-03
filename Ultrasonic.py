import RPi.GPIO as GPIO
import time

# Setup
GPIO.setmode(GPIO.BCM)
TRIG = 4
ECHO = 17

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def measure_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)  # Let sensor settle

    # Trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10µs pulse
    GPIO.output(TRIG, False)

    # Wait for echo to start
    start_time = time.time()
    timeout = start_time + 0.02  # 20ms timeout
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if start_time > timeout:
            print("❌ No Echo Received (Start)")
            return None

    # Wait for echo to end
    stop_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    # Calculate distance
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # cm

    return distance

try:
    while True:
        dist = measure_distance()
        if dist is not None:
            print(f"✅ Distance: {dist:.2f} cm")
        else:
            print("⚠️ No valid measurement.")
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()
