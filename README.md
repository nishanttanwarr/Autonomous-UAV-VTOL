# Autonomous Landing of a VTOL UAV on a Moving Vehicle (Simulation Framework)

This project presents a **MATLAB-based simulation framework** for autonomous landing of a **VTOL UAV (drone)** on a **moving ground vehicle** under realistic, non-ideal conditions.  
It extends earlier simplified work by introducing **sensor uncertainty, environmental disturbances, advanced control algorithms, security layers, and operator monitoring tools**.

---

## 🚀 Features

- **Realistic UAV & Vehicle Simulation**
  - UAV dynamics with wind disturbances and rotor thrust modeling.
  - Vehicle with non-uniform, unpredictable velocity profiles.

- **Sensor Modeling & Fusion**
  - GPS, IMU, and barometer simulated with **noise and bias**.
  - **Extended Kalman Filter (EKF)** for robust state estimation.

- **Optimal Control**
  - **Linear Quadratic Regulator (LQR)** for stable trajectory control under disturbances.
  - Smooth and efficient tracking compared to basic PD control.

- **Mission State Machine**
  - Structured mission logic with four phases:
    1. **SEARCHING** – UAV scans for the target vehicle.
    2. **APPROACH** – UAV moves toward the predicted vehicle position.
    3. **ALIGN** – UAV positions itself above the landing pad.
    4. **LANDING** – Controlled descent onto the moving vehicle.

- **Probabilistic Predictive Search**
  - Uses last known vehicle position + velocity updates to compute intercept points.
  - Handles delayed and uncertain target data.

- **Security Layer**
  - **A3 authentication model** prevents spoofing attacks using fake ArUco markers.

- **Operator Dashboard**
  - Real-time visualization of UAV telemetry, vehicle dynamics, EKF state estimation, marker detection, and spoofing logs.

---

## 🛠️ Tech Stack

- **MATLAB / Simulink**
- **Computer Vision Toolbox** (ArUco detection, camera modeling)
- **Control System Toolbox** (LQR design, state-space modeling)
- **Custom EKF Implementation**

