# Systemkomponenten

**Stand:** 2025-12-21 | **Projektstand:** Phase 1–3 ✅, Phase 4 🔜

Diese Hardware-/Software-Auswahl ist auf **ROS 2 Humble (Docker auf Pi 5)** und eine **hybride Architektur** ausgelegt: **ESP32-S3 = Echtzeit/Drivebase**, **Pi 5 = ROS/Perception/Navigation**.

---

## 1) Compute: High-Level-Recheneinheit (ROS)

### Raspberry Pi 5 (8 GB)

| Aspekt    | Ist-Stand                                           |
| --------- | --------------------------------------------------- |
| RAM       | 8 GB                                                |
| OS (Host) | Raspberry Pi OS 64-bit (Bookworm)                   |
| ROS 2     | Humble **in Docker**                                |
| Kühlung   | Active Cooler                           |
| Geräte    | USB: `/dev/ttyACM0` (ESP32), `/dev/ttyUSB0` (LiDAR) |

**Container (Ist):**

| Container   | Zweck                                       |
| ----------- | ------------------------------------------- |
| `amr_agent` | micro-ROS Agent (Serial)                    |
| `amr_dev`   | ROS 2 Humble Workspace (Nodes/Launch/Tools) |

---

## 2) Control: Low-Level-Echtzeit (Drivebase)

### Controller: Seeed XIAO ESP32-S3

| Aspekt         | Ist-Stand                                           |
| -------------- | --------------------------------------------------- |
| Rolle          | Echtzeit-Control + Odometrie + Safety               |
| RT-Loop        | $100\,\mathrm{Hz}$ (Control Task)                 |
| ROS-Anbindung  | **micro-ROS Client** über **USB-CDC Serial**        |
| Agent-Baudrate | $921600\,\mathrm{baud}$                           |
| Safety         | Failsafe-Stop nach $2000\,\mathrm{ms}$ Timeout    |
| Regelung       | aktuell **Feedforward/Open-Loop** (PID deaktiviert) |

**Drivebase-Hardware (Ist):**

| Komponente                     | Ist-Stand                                                                     |
| ------------------------------ | ----------------------------------------------------------------------------- |
| Motortreiber                   | Cytron MDD3A (Dual-PWM pro Motor)                                             |
| Motoren                        | JGA25-370 (12 V) + Encoder                                                    |
| Encoder (kalibriert)           | links $374{,}3\,\mathrm{ticks/rev}$, rechts $373{,}6\,\mathrm{ticks/rev}$ |
| Geometrie (Firmware-Parameter) | Raddurchmesser $0{,}065\,\mathrm{m}$, Spurbreite $0{,}178\,\mathrm{m}$    |

---

## 3) Sensorik: Geometrie (Phase 3 abgeschlossen)

### LiDAR: RPLidar A1

| Aspekt   | Ist-Stand                                                              |
| -------- | ---------------------------------------------------------------------- |
| Treiber  | `sllidar_ros2`                                                         |
| Device   | `/dev/ttyUSB0` (cp210x)                                                |
| Topic    | `/scan` (`sensor_msgs/msg/LaserScan`)                                  |
| Frequenz | ca. $7{,}6\,\mathrm{Hz}$                                             |
| Range    | `range_min` $0{,}05\,\mathrm{m}$, `range_max` $12{,}0\,\mathrm{m}$ |
| Frame-ID | `laser` (TF-Anbindung folgt in Phase 4)                                |

### IMU (später)

| Komponente | Status                                                  |
| ---------- | ------------------------------------------------------- |
| MPU6050    | reserviert/noch nicht integriert (Phase 4+ oder später) |

### Vision / AI (später)

| Komponente                     | Status  |
| ------------------------------ | ------- |
| Global Shutter Kamera (IMX296) | geplant |
| Hailo-8L                       | geplant |

---

## 4) Power Tree (Status: teils projektspezifisch, Werte noch zu verifizieren)

| Block      | Empfehlung/Annahme                                               |
| ---------- | ---------------------------------------------------------------- |
| Akku       | 3S (nominal $11{,}1\,\mathrm{V}$, voll $12{,}6\,\mathrm{V}$) |
| 5V-Schiene | DC/DC $5\,\mathrm{V}$ mit Reserve (Pi 5 + LiDAR)               |
| Trennung   | Motor-/Lastpfad vs. Logikpfad (Noise/Resets vermeiden)           |

**Hinweis:** Ein belastbarer Laufzeitwert braucht Messung unter realer Fahr-Last (Motorstrom dominiert).

---

## 5) Software-Referenz (Ist)

| Aspekt                       | Ist-Stand                                  |
| ---------------------------- | ------------------------------------------ |
| ROS Distribution             | Humble                                     |
| micro-ROS Transport          | Serial (USB-CDC)                           |
| Agent                        | `microros/micro-ros-agent:humble` (Docker) |
| LiDAR-Stack                  | `sllidar_ros2` → `/scan`                   |
| Nächster Integrationsschritt | **Phase 4: URDF + TF + (optional EKF)**    |

---

*Dokumentation: Jan Unger | Stand: 2025-12-21*
