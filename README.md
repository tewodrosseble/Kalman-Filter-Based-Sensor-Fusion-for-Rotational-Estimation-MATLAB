# 🎯 Sensor Fusion for Real-Time Angle Estimation Using Kalman Filter

This project demonstrates a real-time **angle estimation system** using sensor fusion of **IMU (gyroscope and accelerometer)** and **rotary encoder** data. A **Kalman filter** is implemented in MATLAB to accurately estimate the angle by fusing noisy and drifting signals — addressing real-world challenges such as sensor noise, latency, and signal degradation.

> 📁 This repository includes MATLAB scripts, a live GUI dashboard, and a demo video showcasing system functionality.

---

## 🧠 Features

### ✅ Kalman Filter-Based Sensor Fusion
- Combines gyroscope and accelerometer data to estimate the angular position.
- Filters out gyro drift and accelerometer noise effectively.
- Real-time updates across all signal transitions.

### 🧪 Sensor Simulation
- Encoder with **quantization error**
- Gyroscope with **random noise and constant drift**
- Accelerometer with **high noise profile**

### 🖥️ MATLAB GUI Dashboard
A full-featured **interactive dashboard** built using MATLAB GUI elements:
- 🟢 **Start/Stop buttons** to control animation
- 🎚️ **Manual time slider** to scrub through the simulation timeline
- ☑️ **Real-time noise toggle** to dynamically alter sensor noise profiles
- 🎯 **Rotating pointer animation** representing the current estimated angle in real time
- 📊 Live plotting of:
  - True angle
  - Kalman estimate
  - Estimation error

### 🎥 Automatic Video Recording
- Simulation automatically saves a video (`sensor_fusion_demo.mp4`) of the GUI and animation using MATLAB’s `VideoWriter`.

---

## 📽️ Demo Video

https://user-images.githubusercontent.com/tewodrosseble/sensor_fusion_demo.mp4

> Replace the URL above with the actual uploaded video location from your GitHub repo or upload via GitHub's “Releases” tab or a platform like YouTube.

---

## 📂 Files Included
| File | Description |
|------|-------------|
| `sensor_fusion_gui_recordable.m` | Main GUI script with all features enabled |
| `sensor_fusion_demo.mp4`         | Example output video of the simulation |
| `sensor_simulation_data.m`       | (Optional) Raw data generator for individual sensor tests |
| `README.md`                      | Project documentation |

---

## 🛠️ Requirements
- MATLAB R2021a or later (GUI and `VideoWriter` support required)
- No additional toolboxes required

---

## 🧠 Educational Value
This project is especially useful for students or engineers learning:
- Sensor fusion fundamentals
- Kalman filtering
- IMU and encoder signal behavior
- Embedded system estimation in control loops

---

## 🔗 Applications
- Robotics and autonomous systems
- Motor control systems (servo/BLDC)
- Wearable and inertial navigation systems
- Mechatronics and real-time feedback

---

## 📜 License
This project is open-source under the MIT License. See `LICENSE` for details.

---

## 🙋‍♂️ Author
**[Your Name]**  
LinkedIn: [Your LinkedIn]  
GitHub: [Your GitHub Profile]  

---

