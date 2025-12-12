# AMR Platform - Autonomous Mobile Robot

Autonome mobile Roboterplattform mit ROS 2 Jazzy auf Raspberry Pi 5.

## Architektur

```
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Raspberry Pi OS Lite + Docker)            │
│  ├── libcamera     → IMX296 Global Shutter Kamera          │
│  ├── HailoRT 4.23  → Hailo-8L AI Beschleuniger             │
│  └── Docker        → ROS 2 Jazzy Container                 │
├─────────────────────────────────────────────────────────────┤
│  Container: amr_perception                                  │
│  ├── Nav2, SLAM Toolbox, robot_localization                │
│  ├── rplidar_ros, camera_ros                               │
│  └── amr_description (URDF), amr_bringup (Launch)          │
├─────────────────────────────────────────────────────────────┤
│  Container: amr_micro_ros                                   │
│  └── micro-ROS Agent → Serial Bridge                       │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ USB Serial (115200 Baud)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 XIAO (Echtzeit-Controller)                        │
│  ├── Motor-PWM      → Cytron MDD3A (D0-D3)                  │
│  ├── Encoder        → JGA25-370 (D6, D7)                    │
│  ├── IMU            → MPU6050 (I2C: D4, D5)                 │
│  └── micro-ROS      → /cmd_vel, /odom                       │
└─────────────────────────────────────────────────────────────┘
```

## Dokumentation

| Nr | Dokument | Beschreibung |
|----|----------|--------------|
| 01 | `docs/01-Pi-OS-flashen.md` | Raspberry Pi OS + Docker Setup |
| 02 | `docs/02-hailo-setup.md` | Hailo-8L Treiber (HailoRT 4.23.0) |
| 03 | `docs/03-ros2-docker.md` | ROS 2 Container + URDF |
| 04 | `docs/04-esp32-firmware.md` | PlatformIO + micro-ROS |

## Schnellstart

### 1. Auf dem Entwicklungs-Mac

```bash
# VS Code Workspace öffnen
code amr-platform.code-workspace

# ESP32 Firmware flashen
cd firmware && pio run --target upload
```

### 2. Auf dem Raspberry Pi

```bash
# Docker-Stack starten
cd ~/amr && docker compose up -d

# In Container wechseln
docker exec -it amr_perception bash
```

### 3. Motor-Test

```bash
# Teleop starten (im Container)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Verzeichnisstruktur

```
amr-platform/
├── firmware/              # ESP32-S3 XIAO (PlatformIO)
│   ├── src/main.cpp
│   ├── platformio.ini
│   └── lib/
├── ros2_ws/               # ROS 2 Workspace
│   ├── src/
│   │   ├── amr_description/   # URDF, Meshes
│   │   └── amr_bringup/       # Launch-Files
│   ├── config/            # YAML-Konfigurationen
│   └── maps/              # SLAM-Karten
├── docker/                # Docker-Infrastruktur
│   ├── docker-compose.yml
│   └── Dockerfile
├── docs/                  # Dokumentation (01-04)
└── scripts/               # Utility-Scripts
```

## Hardware

| Komponente | Modell | Funktion |
|------------|--------|----------|
| Compute | Raspberry Pi 5 (8GB) | ROS 2 Host |
| AI | Hailo-8L (13 TOPS) | Objekterkennung |
| LiDAR | RPLIDAR A1 | 2D SLAM |
| Kamera | IMX296 Global Shutter | Semantik |
| MCU | ESP32-S3 XIAO | Echtzeit-Control |
| Motoren | JGA25-370 (2×) | Antrieb |
| Treiber | Cytron MDD3A | H-Brücke |

## Budget

Gesamtkosten: **482,48 €**
- Intelligence (69%): Raspberry Pi, Hailo, Kamera, LiDAR
- Mechanik (29%): Motoren, Chassis, Servos
- Infrastruktur (2%): DC/DC-Wandler
