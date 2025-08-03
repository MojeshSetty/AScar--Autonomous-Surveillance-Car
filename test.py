import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

test_pin = 17
GPIO.setup(test_pin, GPIO.OUT)

print("Turning on LED for 3 seconds...")
GPIO.output(test_pin, True)
time.sleep(3)
GPIO.output(test_pin, False)

GPIO.cleanup()
print("Test complete.")
