import RPi.GPIO as GPIO
import time

# Set up the GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme

# Set GPIO pin 17 as an output
GPIO.setup(17 , GPIO.OUT)

# Start the blinking loop
try:
    while True:
        GPIO.output(17, GPIO.HIGH)  # Turn the LED on
        time.sleep(0.2)  # Wait for 1 second
        GPIO.output(17, GPIO.LOW)   # Turn the LED off
        time.sleep(0.2)  # Wait for 1 second
except KeyboardInterrupt:
    pass

# Clean up the GPIO settings when the program ends
GPIO.cleanup()
