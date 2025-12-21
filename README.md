# AMR Platform (ROS 2 Humble) – ESP32-S3 Drivebase + Raspberry Pi 5

Kurzbeschreibung: Repository für einen Differential-Drive-AMR mit **ESP32-S3** (Low-Level Control via micro-ROS) und **Raspberry Pi 5** (ROS 2 Humble, SLAM/Nav2).
**Status:** Phase 1–3 abgeschlossen (Stand: 2025-12-20)

---

## Ziel und Regel dieser README

**Ziel:** In **5–10 Minuten** von „Repo geöffnet“ zu „System läuft“ (Agent verbunden, Topics sichtbar, kurzer Smoke-Test).
**Regel:** README ist der **Einstieg (Happy Path)**. Details gehören in:

- **Systemdokumentation:** Zweck/Scope, Betriebsmodi, Safety, Abnahme
- **Entwicklerdokumentation:** Architektur im Code, HAL, Parameter-Details, Debug/Troubleshooting tief

---

## Was ist hier drin?

- **Firmware** (ESP32-S3): micro-ROS Client, Motorsteuerung, `/odom_raw`, Failsafe
- **Docker Setup** (Pi 5): micro-ROS Agent + ROS 2 Workspace
- **ROS 2 Pakete**: LiDAR-Treiber, (Phase 4+) URDF/Bridge

---

## Quick Start (Happy Path)

### 1) Voraussetzungen

- Raspberry Pi 5 (64-bit OS), Docker + Docker Compose
- Dev-PC mit VS Code + PlatformIO
- ESP32-S3 (Seeed XIAO) per USB verbunden
- RPLidar A1 per USB verbunden

### 2) Firmware flashen (ESP32-S3)

```bash
cd firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

### 3) Docker starten (Pi 5)

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose logs microros_agent --tail 5
```

Erwartung: Agent läuft (z. B. `running... | fd: 3`).

### 4) LiDAR starten (RPLidar A1)

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### 5) Smoke-Checks (Topics & Daten)

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom_raw --once
```

### 6) Motor-Test (⚠️ Räder aufbocken!)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
# Ctrl+C → Failsafe stoppt nach ~2 s
```

---

## Schnittstellen (nur Überblick)

**Wichtige Topics:**

```
/cmd_vel          (in)  geometry_msgs/Twist
/odom_raw         (out) geometry_msgs/Pose2D
/esp32/heartbeat  (out) std_msgs/Int32
/scan             (out) sensor_msgs/LaserScan
```

**Ports (Standard):**

- ESP32-S3: `/dev/ttyACM0` (921600 Baud)
- RPLidar A1: `/dev/ttyUSB0`

---

## Projektstruktur

```
amr-platform/
├─ firmware/           # ESP32-S3 Firmware
├─ docker/             # docker-compose (Agent + Dev-Container)
├─ ros2_ws/            # ROS 2 Workspace (Treiber, später URDF/Bridge)
└─ docs/               # System-/Entwicklerdoku + Phasen
```

---

## Status (Roadmap auf einen Blick)

| Phase | Inhalt               | Status |
| ----: | -------------------- | :----: |
|     1 | micro-ROS ESP32-S3   |    ✅   |
|     2 | Docker-Infrastruktur |    ✅   |
|     3 | RPLidar A1           |    ✅   |
|     4 | URDF + TF + EKF      |   🔜   |
|     5 | SLAM (slam_toolbox)  |    ⬜   |
|     6 | Nav2 Navigation      |    ⬜   |

---

## Wo geht’s weiter?

- **Systemdokumentation:** `docs/systemdokumentation.md`
- **Entwicklerdokumentation (Firmware):** `docs/entwicklerdokumentation.md`
- **Phasen-Doku:** `docs/phases/`

---

## Lizenz / Credits

- Autor: Jan Unger
- Stack: ROS 2 Humble, micro-ROS, FreeRTOS
- Hardware: ESP32-S3, Raspberry Pi 5, RPLidar A1
