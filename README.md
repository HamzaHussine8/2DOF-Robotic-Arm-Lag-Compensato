# Design of Lag Compensator for Two Degree of Freedom Robotic Arm



### Author:
- Hamza Hussein Mohamed  


## 📘 Project Overview

This project implements a control system for a 2-degree-of-freedom (2-DOF) robotic arm using a **lag compensator**. The robot is designed and simulated using Simulink and MATLAB, with real-time data acquisition and hardware-in-the-loop (HIL) implementation.

---

## 📐 1. Construction

- Robot: 2-DOF arm actuated by **DC motors (JGA25-370, 210 RPM)** with **2-channel encoders**
- Designed using **Autodesk Inventor**
- 3D printed using **PLA+ material**
- Encoders provide position, speed, and direction feedback
- Circuit simulated using **Proteus**

---

## 🔍 2. Theory of Operation

- Robot modeled in **Simulink** with **lead-lag compensator**
- Designed to enhance:
  - Stability
  - Response time
  - Accuracy
- Closed-loop control with encoder feedback
- Simulation + real-time tuning before hardware implementation

---

## 🧪 3. Designing Steps

### Data Collection:
- Arduino IDE used to control motors and log encoder pulses
- Serial communication to log input/output in Excel
- Preprocessing: filtering and noise removal

### System Identification:
- Used **MATLAB System Identification Toolbox**
- Generated transfer functions from real data
- Validated model accuracy

### Lag Compensator Design:
- Designed using **SISO Design Tool**
- Tuned for stability, overshoot, and settling time

### Simulink Implementation:
- Integrated with **Arduino toolbox**
- HIL simulation (sampling time = 0.001 sec, baud rate = 9600)

### Simulation & Validation:
- Tested with various waveforms and disturbances
- Compared responses with and without compensation

---

## ⚙️ 4. Hardware Implementation

- 2 DC motors + encoders
- **L298N driver**
- **Arduino Mega** as the interface (data acquisition and control)
- Simulated in **Proteus** and **Simscape**

---

## 📊 5. Results and Conclusions

- **Square and sine wave inputs** used to evaluate performance
- Comparisons show **better stability and precision** with lag compensator
- Validated on both base and end-effector motion

---

## 📁 Files Included

- `hamza_modern_project.slx`: Simulink model
---

## 🛠 Tools Used

- MATLAB / Simulink
- Arduino IDE
- Proteus
- Autodesk Inventor
- 3D Printing (PLA+)

---

## 🔗 License

This project is for educational use. Contact the authors for reuse or collaboration.

