import cv2
import numpy as np
import time
import os
import threading
import RPi.GPIO as GPIO
from gpiozero import Servo
from flask import Flask, render_template, Response, request
import tflite_runtime.interpreter as tflite  # Use TFLite runtime for inference

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
if not video_capture.isOpened():
    print("Error: Cannot open video capture")
    exit()

# ==============================
# Load the Driving Model (TFLite) & Mapping
# ==============================
# The driving model should output four classes:
# 0 -> forward, 1 -> left, 2 -> right, 3 -> stop
commands = ["forward", "left", "right", "stop"]

MODEL_PATH = "driving_model.tflite"
if os.path.exists(MODEL_PATH):
    print("Loading driving model (TFLite)...")
    interpreter = tflite.Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
else:
    print("Driving model not found. Please train and convert the model to TFLite first.")
    exit()

# ==============================
# Motor Control Functions
# ==============================
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
    distance = (elapsed_time * 34300) / 2  # centimeters
    return round(distance, 2)

def look_right():
    """Rotate servo to right, read distance, then reset servo."""
    servo.value = -1  # right extreme
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0
    return distance

def look_left():
    """Rotate servo to left, read distance, then reset servo."""
    servo.value = 1  # left extreme
    time.sleep(0.5)
    distance = read_distance()
    servo.value = 0
    return distance

# =====================================
# Autonomous Navigation (Background Thread)
# =====================================
def autonomous_navigation():
    global manual_override, mode
    while True:
        # Run autonomous routine only if in autonomous mode and no manual override
        if mode != 'autonomous' or manual_override:
            time.sleep(0.1)
            continue

        # Capture a frame from the video stream for driving prediction
        ret, frame = video_capture.read()
        if not ret:
            continue

        # Preprocess the frame: resize to (64, 64) and normalize pixel values
        frame_resized = cv2.resize(frame, (64, 64))
        frame_normalized = frame_resized.astype("float32") / 255.0
        frame_input = np.expand_dims(frame_normalized, axis=0)  # Shape: (1, 64, 64, 3)

        # Use TFLite interpreter for prediction
        interpreter.set_tensor(input_details[0]['index'], frame_input)
        interpreter.invoke()
        output_data = interpreter.get_tensor(output_details[0]['index'])
        command_idx = np.argmax(output_data, axis=1)[0]
        predicted_command = commands[command_idx]
        print(f"[Autonomous] Predicted Command: {predicted_command}")

        # Execute the corresponding motor control function
        if predicted_command == "forward":
            move_forward()
        elif predicted_command == "left":
            turn_left()
        elif predicted_command == "right":
            turn_right()
        elif predicted_command == "stop":
            stop()

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
    """Video streaming route."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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
        # Start the autonomous navigation thread (which now uses the TFLite driving model)
        auto_thread = threading.Thread(target=autonomous_navigation, daemon=True)
        auto_thread.start()

        # Run the Flask web server (for manual control and video streaming)
        app.run(host='0.0.0.0', port=8080, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        GPIO.cleanup()
        video_capture.release()





How to Run This Code on Raspberry Pi 4B with TFLite Runtime
Convert Your Model to TFLite:
First, convert your trained Keras model (originally saved as an H5 file) to a TFLite model. For example, you can do this on your development machine:
python


import tensorflow as tf
model = tf.keras.models.load_model("driving_model.h5")
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()
with open("driving_model.tflite", "wb") as f:
    f.write(tflite_model)

