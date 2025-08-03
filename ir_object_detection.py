import RPi.GPIO as GPIO
import time

# Set GPIO Mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define sensor pins
sensor_pins = [17, 18, 27, 22, 23]

# Setup pins as input
for pin in sensor_pins:
    GPIO.setup(pin, GPIO.IN)

try:
    while True:
        # Read sensors and print status
        sensor_status = []
        for pin in sensor_pins:
            status = GPIO.input(pin)
            sensor_status.append(status)
        
        print(f"Sensor Status: {sensor_status}")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Program stopped by User")
    GPIO.cleanup()
