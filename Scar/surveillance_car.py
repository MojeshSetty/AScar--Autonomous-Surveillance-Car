#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import threading
from gpiozero import Servo
from flask import Flask, render_template, Response, request
import cv2

# ================================
# Hardware Pin Definitions & Setup
# ================================
# Motor pins (using one set for both autonomous and manual control)
IN1 = 23
IN2 = 24
IN3 = 27
IN4 = 10
ENA = 25  # PWM enable for left motor
ENB = 16  # PWM enable for right motor

# Ultrasonic sensor pins
TRIG_PIN = 17
ECHO_PIN = 18

# Servo pin (for sensor scanning)
SERVO_PIN = 12

# Set GPIO mode and warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins
motor_pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Setup ultrasonic sensor pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Setup PWM for motor speed control
pwmA = GPIO.PWM(ENA, 1000)  # 1kHz frequency
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)

# Setup servo for scanning
servo = Servo(SERVO_PIN)
servo.value = 0  # Center position

# =======================
# Global Variables & Mode
# =======================
MAX_SPEED = 100        # Maximum PWM duty cycle
BASE_SPEED = 50        # Base speed when moving forward
HIGH_SPEED = 70        # Speed used during turns or reverse moves
SAFE_DISTANCE = 20     # Minimum safe distance in centimeters
turn_duration = 1.0    # Duration to allow for a turn

# Mode variables: "autonomous" or "manual"
mode = 'autonomous'
manual_override = False

# =======================
# Video Capture Setup
# =======================
video_capture = cv2.VideoCapture(0)

# =======================
# Function Definitions
# =======================

def read_distance():
    """Measure distance using the ultrasonic sensor."""
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Convert to centimeters
    return round(distance, 2)

def set_motor_speed(speed):
    """Set both motors to a given speed (PWM duty cycle)."""
    speed = max(0, min(speed, MAX_SPEED))
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)

def move_forward():
    """Move the car forward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed(HIGH_SPEED)

def move_backward():
    """Move the car backward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    set_motor_speed(HIGH_SPEED)

def turn_right():
    """Pivot right: left wheel forward, right wheel backward."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)  # Left wheel forward
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH) # Right wheel backward
    set_motor_speed(BASE_SPEED)
    time.sleep(turn_duration)
    stop()

def turn_left():
    """Pivot left: left wheel backward, right wheel forward."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH) # Left wheel backward
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)  # Right wheel forward
    set_motor_speed(BASE_SPEED)
    time.sleep(turn_duration)
    stop()

def stop():
    """Stop all motor activity."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    set_motor_speed(0)

def look_right():
    """Rotate servo to right, read distance, then reset servo."""
    servo.value = -1  # Rotate to right extreme
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0   # Reset to center
    return distance

def look_left():
    """Rotate servo to left, read distance, then reset servo."""
    servo.value = 1   # Rotate to left extreme
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0   # Reset to center
    return distance

# =====================================
# Autonomous Navigation (Background Thread)
# =====================================
def autonomous_navigation():
    global manual_override, mode
    while True:
        # Only run autonomous routine if in autonomous mode and no manual override
        if mode != 'autonomous' or manual_override:
            time.sleep(0.1)
            continue

        distance = read_distance()
        print(f"[Autonomous] Distance: {distance} cm")

        if distance <= SAFE_DISTANCE:
            stop()
            time.sleep(0.2)
            # Look for a clearer path by scanning right and left
            right_distance = look_right()
            left_distance = look_left()
            print(f"[Autonomous] Right: {right_distance} cm, Left: {left_distance} cm")
            if right_distance > left_distance:
                turn_right()
            else:
                turn_left()
            move_forward()
        else:
            # Dynamically adjust speed based on distance (optional)
            speed = BASE_SPEED + (MAX_SPEED - BASE_SPEED) * (distance / 100)
            set_motor_speed(speed)
            move_forward()

        time.sleep(0.1)

# =====================================
# Flask Web Server for Manual Control
# =====================================
app = Flask(__name__)

def generate_frames():
    """Video streaming generator function."""
    while True:
        success, frame = video_capture.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Home page: display video feed and control options."""
    return render_template("index.html", mode=mode)

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ---------- Manual control endpoints ----------
def manual_control(action_function, action_name):
    """Helper to run a manual action and trigger override."""
    global manual_override
    manual_override = True
    print(f"[Manual] {action_name} command received")
    action_function()
    time.sleep(0.5)  # Action duration; adjust as needed
    stop()
    manual_override = False
    return f"{action_name} executed."

@app.route('/forward', methods=['POST'])
def forward():
    return manual_control(move_forward, "Forward")

@app.route('/backward', methods=['POST'])
def backward():
    return manual_control(move_backward, "Backward")

@app.route('/left', methods=['POST'])
def left():
    return manual_control(turn_left, "Left")

@app.route('/right', methods=['POST'])
def right():
    return manual_control(turn_right, "Right")

@app.route('/stop', methods=['POST'])
def stop_route():
    return manual_control(stop, "Stop")

@app.route('/set_mode', methods=['POST'])
def set_mode():
    """Switch between autonomous and manual modes."""
    global mode
    new_mode = request.form.get('mode')
    if new_mode in ['autonomous', 'manual']:
        mode = new_mode
        print(f"[Mode] Mode changed to: {mode}")
        return f"Mode set to {mode}"
    return "Invalid mode", 400

# =====================================
# Main Program: Start Threads & Server
# =====================================
if __name__ == '__main__':
    try:
        # Start the autonomous navigation in a background thread.
        auto_thread = threading.Thread(target=autonomous_navigation, daemon=True)
        auto_thread.start()

        # Run the Flask web server (accessible on your network)
        app.run(host='0.0.0.0', port=8080, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        GPIO.cleanup()
        video_capture.release()


