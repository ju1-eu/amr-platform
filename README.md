# AMR Platform: Autonomous Mobile Robot

**Ein hybrides AMR-System basierend auf ESP32-S3 (Real-Time Control) und Raspberry Pi 5 (High-Level Navigation).**

Dieses Repository enthält die Firmware, Treiber und Konfigurationen für einen Differential-Drive-Roboter, der für **SLAM** und **autonome Navigation (Nav2)** unter ROS 2 Humble entwickelt wurde.

---

## 🏗 Architektur

Das System folgt einer **Hybrid-Echtzeit-Architektur**:

* **Low-Level (ESP32-S3):** Harte Echtzeit-Regelung der Motoren (100 Hz), Odometrie-Integration und Safety-Features. Implementiert als **Dual-Core FreeRTOS** Applikation mit micro-ROS.
* **High-Level (Raspberry Pi 5):** ROS 2 Humble in Docker, LiDAR-Verarbeitung, SLAM und Navigation.
* **Kommunikation:** micro-ROS (XRCE-DDS) über USB-CDC (921600 Baud).

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 (micro-ROS Client) - Firmware v3.2.0              │
│                                                             │
│  Core 0: Control Task (100 Hz)                              │
│    - Feedforward-Steuerung (Gain=2.0)                       │
│    - Encoder-Auswertung                                     │
│    - Odometrie-Integration                                  │
│    - Failsafe (2000ms Timeout)                              │
│                                                             │
│  Core 1: micro-ROS Communication                            │
│    - /cmd_vel Subscriber                                    │
│    - /odom_raw Publisher (20 Hz)                            │
│    - /esp32/heartbeat Publisher (1 Hz)                      │
└─────────────────────────────────────────────────────────────┘
                            │
                      USB-CDC (921600 Baud)
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Docker)                                    │
│                                                             │
│  Container: amr_agent (micro-ROS Agent)                     │
│  Container: amr_dev (ROS 2 Humble Workspace)                │
│    - RPLidar A1 → /scan (7.6 Hz)                            │
│    - Navigation Stack                                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### 1. Voraussetzungen

* **Host:** Raspberry Pi 5 mit Raspberry Pi OS 64-bit
* **Dev-PC:** VS Code mit PlatformIO Extension
* **Docker:** Docker Engine + Docker Compose

### 2. Firmware flashen (ESP32-S3)

```bash
cd firmware
pio run -e seeed_xiao_esp32s3 -t upload
# Nach dem Upload blinkt die LED (sucht Agent)
```

### 3. Docker Container starten (Raspberry Pi)

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose logs microros_agent --tail 5
# Erwartung: "running... | fd: 3"
```

### 4. RPLidar starten

```bash
# Terminal 1: Lidar-Node
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### 5. Smoke-Tests

```bash
# Terminal 2: Topics prüfen
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom_raw --once
```

### 6. Motor-Test (⚠️ Räder aufbocken!)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
# Ctrl+C → Failsafe stoppt nach 2s
```

---

## 🧩 Hardware Setup

| Komponente | Typ | Funktion |
|------------|-----|----------|
| **MCU** | Seeed XIAO ESP32-S3 | Dual-Core FreeRTOS + micro-ROS |
| **Treiber** | Cytron MDD3A | Dual-PWM Motor Driver |
| **Motoren** | JGA25-370 (12V) | Encoder-Motoren (374 Ticks/Rev) |
| **LiDAR** | RPLidar A1 | 360° 2D Scan (12m, 7.6 Hz) |
| **SBC** | Raspberry Pi 5 (8GB) | ROS 2 Humble + Docker |
| **AI** | Hailo-8L | AI Accelerator (später) |
| **Power** | 3S Li-Ion (12V) | Stromversorgung |

### Pin-Belegung (ESP32-S3)

| Pin | Funktion | Hardware |
|-----|----------|----------|
| D0-D3 | Motor PWM | Cytron MDD3A |
| D6, D7 | Encoder | JGA25-370 |
| D10 | LED/MOSFET | Status/Failsafe |
| D4, D5 | I2C | *Reserviert (IMU)* |

---

## 📂 Projektstruktur

```
amr-platform/
├── firmware/                 # ESP32-S3 Firmware (v3.2.0)
│   ├── src/main.cpp          # Dual-Core FreeRTOS + micro-ROS
│   └── include/config.h      # Hardware-Konfiguration
├── docker/
│   ├── docker-compose.yml    # amr_agent + amr_dev
│   └── Dockerfile
├── ros2_ws/
│   └── src/
│       ├── sllidar_ros2/     # RPLidar Treiber
│       ├── amr_description/  # URDF (Phase 4)
│       └── amr_bridge/       # odom_converter (Phase 4)
└── docs/
    ├── systemdokumentation.md
    ├── entwicklerdokumentation.md
    └── phases/               # Phasen-Dokumentation
```

---

## 🛠 Status & Roadmap

| Phase | Feature | Status |
|-------|---------|--------|
| **1** | micro-ROS ESP32-S3 (Dual-Core) | ✅ Done |
| **2** | Docker-Infrastruktur | ✅ Done |
| **3** | RPLidar A1 Integration | ✅ Done |
| **4** | URDF + TF-Baum | 🔜 Next |
| **5** | SLAM (slam_toolbox) | ⬜ Planned |
| **6** | Nav2 Autonomie | ⬜ Planned |

### Aktuelle Topics

```
/cmd_vel          # Input: Twist
/odom_raw         # Output: Pose2D (20 Hz)
/esp32/heartbeat  # Output: Int32 (1 Hz)
/scan             # Output: LaserScan (7.6 Hz)
```

---

## ⚙️ Konfiguration

### Firmware (v3.2.0)

| Parameter | Wert |
|-----------|------|
| Baudrate | 921600 |
| Feedforward Gain | 2.0 |
| PID | Deaktiviert (Encoder-Polarität) |
| Failsafe Timeout | 2000 ms |
| Control Rate | 100 Hz |
| Odom Publish | 20 Hz |

### Docker Container

| Container | Image | Port |
|-----------|-------|------|
| `amr_agent` | `microros/micro-ros-agent:humble` | /dev/ttyACM0 |
| `amr_dev` | Custom (ROS 2 Humble) | /dev/ttyUSB0 |

---

## 🐛 Troubleshooting

| Problem | Lösung |
|---------|--------|
| Agent "Serial port not found" | ESP32 Reboot, `docker compose restart microros_agent` |
| `/scan` fehlt | RPLidar-Node manuell starten |
| Motor eskaliert | Feedforward statt PID nutzen |
| Port blockiert | `sudo fuser /dev/ttyUSB0` prüfen |

---

## 📜 Lizenz & Credits

Entwickelt als Master-Projekt.

* **Autor:** Jan Unger
* **Frameworks:** ROS 2 Humble, micro-ROS, FreeRTOS
* **Hardware:** ESP32-S3, Raspberry Pi 5, RPLidar A1
