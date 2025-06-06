# 🎯 Sensor Fusion for Real-Time Angle Estimation Using Kalman Filter

This project demonstrates a real-time **angle estimation system** using sensor fusion of **IMU (gyroscope and accelerometer)** and **rotary encoder** data. A **Kalman filter** is implemented in MATLAB to accurately estimate the angle by fusing noisy and drifting signals — addressing real-world challenges such as sensor noise, latency, and signal degradation.

> 📁 This repository includes MATLAB scripts, a live GUI dashboard, and a demo video showcasing system functionality.

---

## 🧠 Features

### ✅ Kalman Filter-Based Sensor Fusion
- Combines gyroscope and accelerometer data to estimate angular position.
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
- Simulation automatically records the dashboard as `sensor_fusion_demo.mp4` using MATLAB’s `VideoWriter`.

---

## 📽️ Demo Video

🎬 **Click below to watch the demo in your browser:**

[![Watch the demo video](https://img.youtube.com/vi/placeholder/hqdefault.jpg)](https://github.com/tewodrosseble/Kalman-Filter-Based-Sensor-Fusion-for-Rotational-Estimation-MATLAB/blob/main/sensor_fusion_demo.mp4)

> If the video doesn't autoplay, GitHub may preview it in-browser or require download depending on the user's settings.

---

## 📂 Files Included

| File                             | Description                                      |
|----------------------------------|--------------------------------------------------|
| `sensor_fusion_gui_recordable.m` | Main GUI script with all features enabled        |
| `sensor_fusion_demo.mp4`         | Recorded output of the simulation dashboard      |
| `README.md`                      | Project documentation                            |

---

## 🛠️ Requirements

- MATLAB R2021a or later  
- No additional toolboxes required  
- Compatible with most platforms supporting MATLAB GUI and plotting

---

## 🎓 Educational Value

This project helps build intuition for:
- Kalman filters and sensor fusion
- IMU signal characteristics and drift
- Embedded system estimation techniques
- Real-time signal integration challenges

---

## 🌍 Applications

- Robotics and autonomous navigation  
- Servo/motor control systems  
- Inertial navigation units (IMUs)  
- Real-time estimation in control loops  

---

## 📜 License

This project is licensed under the MIT License. See `LICENSE` for more details.

---

## 🙋‍♂️ Author

**Tewodros Seble**  
🔗 [GitHub Profile](https://github.com/tewodrosseble)

---

