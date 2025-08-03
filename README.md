# 🚘 Autonomous Surveillance Car using Raspberry Pi 4B

A complete robotics project demonstrating **basic to advanced automation** using Raspberry Pi 4B. This surveillance car is equipped with multiple sensors (IR, Ultrasonic, Servo, Motors) and programmed to perform tasks ranging from simple sensor control to full autonomous navigation and surveillance monitoring.

---



---

## 📦 Repository Structure

This repo contains modular and integrated Raspberry Pi programs:

```bash
AScar/
├── basic_components/
│   ├── ir_sensor.py
│   ├── ultrasonic_sensor.py
│   ├── motor_control.py
│   └── servo_control.py
│
├── autonomous/
│   ├── lane_detection.py
│   ├── obstacle_avoidance.py
│   ├── camera_streaming.py
│   └── full_autonomous_surveillance.py
│
├── wiring_diagrams/
│   └── circuit_schematics.png
│
├── requirements.txt
└── README.md
```

---

## 🧠 Features

| Module                   | Description                                              |
| ------------------------ | -------------------------------------------------------- |
| **IR Sensor**            | Detects black/white surface for line-following behavior  |
| **Ultrasonic Sensor**    | Measures distance and avoids collisions                  |
| **Motor Driver (L298N)** | Controls wheels for forward, reverse, stop, turn motions |
| **Servo Motor**          | Rotates ultrasonic sensor for wide-angle scanning        |
| **Camera (optional)**    | Streams live footage for surveillance                    |
| **Full Integration**     | Autonomous navigation with obstacle detection            |

---

## 🧰 Hardware Used

* ✅ Raspberry Pi 4B (4GB)
* ✅ L298N Motor Driver
* ✅ DC Motors (2 or 4-wheel drive)
* ✅ IR Sensors (Line Detection)
* ✅ Ultrasonic Sensor (HC-SR04)
* ✅ Servo Motor (SG90)
* ✅ Power Bank / Battery Pack
* ✅ Jumper Wires, Breadboard, Chassis, Wheels

---

## 🖥️ Software & Libraries

* Python 3.7+
* `RPi.GPIO`
* `time`
* `cv2` (for camera use)
* `flask` (for optional video streaming)
* `numpy`

Install dependencies using:

```bash
pip install -r requirements.txt
```

---

## 🚀 How to Run

### 🔌 Wiring Setup

Refer to:

```bash
wiring_diagrams/circuit_schematics.png
```

### ▶️ Running Basic Components

```bash
cd basic_components
python ir_sensor.py
python ultrasonic_sensor.py
python motor_control.py
python servo_control.py
```

### 🧠 Run Autonomous Surveillance System

```bash
cd autonomous
python full_autonomous_surveillance.py
```

> Make sure sensors and motors are connected properly before executing the autonomous module.

---


## 🗓 Project Goals

* ✅ Build and control an autonomous car using Raspberry Pi
* ✅ Integrate multiple sensors for real-world interaction
* ✅ Implement basic robotics logic in Python
* ✅ Capture and stream video for surveillance
* ✅ Prepare groundwork for AI-based vision (future extension)

---

## 💡 Future Enhancements

* 🧠 Object Detection using YOLO/MediaPipe
* 🌐 Remote control via web dashboard or mobile
* 🛱 GPS or LORA-based long-distance communication
* 🔊 Voice commands using Google Assistant API

---

## 👨‍💻 Authors & Contributors

* **Mojesh Chinna** – Full Stack Robotics Developer
* **Sai Pawan**, **Naveen**, **Bharathi** – Support & Testing

> Mentored by faculty at **Rajeev Gandhi Memorial College of Engineering and Technology**

---

## 📝 License

This project is licensed under the MIT License.

---

> 📬 For inquiries, contact via [LinkedIn](https://www.linkedin.com/in/mojeshsetty) or check out [YouTube Channel](https://youtube.com/@mojeshchinna) for demos!
