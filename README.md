# AMR Platform - Autonomous Mobile Robot

> **Bachelor-Thesis:** Konzeption und Realisierung einer autonomen mobilen Roboterplattform
> **Status:** Phase 1 ✅ Abgeschlossen | **Firmware:** v0.3.0-serial

Autonome mobile Roboterplattform mit ROS 2 Jazzy auf Raspberry Pi 5 und ESP32-S3 Echtzeit-Controller.

---

## Architektur

```
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Raspberry Pi OS Lite + Docker)            │
│  ├── libcamera 0.3+ → IMX296 Global Shutter Kamera         │
│  ├── HailoRT 4.23   → Hailo-8L AI Beschleuniger (13 TOPS)  │
│  └── Docker         → ROS 2 Jazzy Container                │
├─────────────────────────────────────────────────────────────┤
│  Container: perception                                      │
│  ├── Nav2, SLAM Toolbox, robot_localization                │
│  ├── rplidar_ros, camera_ros                               │
│  └── amr_description (URDF), amr_bringup (Launch)          │
├─────────────────────────────────────────────────────────────┤
│  Container: serial_bridge                                   │
│  └── ROS 2 Serial Bridge → /cmd_vel → ESP32                │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ USB-CDC Serial (115200 Baud)
                         │ Protokoll: V:<m/s>,W:<rad/s>\n
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 XIAO (Echtzeit-Controller)                       │
│  ├── Dual-PWM       → Cytron MDD3A (D0-D3)                 │
│  ├── Encoder        → JGA25-370 Hall (D6, D7)              │
│  ├── IMU            → MPU6050 (I2C: D4, D5)                │
│  ├── LED-Status     → MOSFET IRLZ24N (D10)                 │
│  └── Serial-Bridge  → Differential Drive Kinematik         │
└─────────────────────────────────────────────────────────────┘
```

---

## Aktueller Status

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| 0 | Fundament (Pi OS, Docker, Hailo) | ✅ Fertig |
| 1 | Motor-Test (Serial-Bridge → Motor) | ✅ **Abgeschlossen** |
| 2 | Odometrie (Encoder, /odom) | ◄── **Aktuell** |
| 3 | SLAM (LiDAR, Kartierung) | ⬜ Offen |
| 4 | Navigation (Nav2, autonom) | ⬜ Offen |
| 5 | Kamera + AI (Hailo, YOLOv8) | ⬜ Offen |

---

## Schnellstart

### 1. Repository klonen

```bash
git clone git@github.com:unger-robotics/amr-platform.git
cd amr-platform
```

### 2. ESP32 Firmware flashen (Mac/Linux)

```bash
cd firmware_serial
pio run --target upload
pio device monitor
```

**Erwartete Ausgabe:**

```
AMR Serial-Bridge v0.3.0
Format: V:<m/s>,W:<rad/s>
READY
```

### 3. Docker-Stack starten (Raspberry Pi)

```bash
ssh pi@rover
cd ~/amr-platform/docker
docker compose up -d

# Logs prüfen
docker compose logs -f serial_bridge
```

**Erwartete Ausgabe:**

```
[INFO] [serial_bridge]: Serial Bridge gestartet: /dev/ttyACM0 @ 115200 baud
[INFO] [serial_bridge]: ESP32 bereit
```

### 4. Teleop-Test

```bash
# In Perception-Container
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:** `i`=Vorwärts, `,`=Rückwärts, `j`/`l`=Drehen, `k`=Stopp

### 5. Manueller Motor-Test (Alternative)

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash

# Motor vorwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Drehung
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Stopp
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

---

## Verzeichnisstruktur

```
amr-platform/
├── firmware/                 # micro-ROS Firmware (nicht verwendet)
├── firmware_serial/          # Serial-Bridge Firmware ✅ AKTIV
│   ├── src/main.cpp          # Serial-Parser, Differential Drive
│   ├── include/config.h      # Hardware-Parameter
│   └── platformio.ini
├── firmware_test/            # Hardware-Validierung
├── ros2_ws/                  # ROS 2 Workspace
│   └── src/
│       ├── amr_description/  # URDF, Launch
│       ├── amr_bringup/      # Konfiguration
│       └── amr_serial_bridge/# Serial Bridge Node ✅
├── docker/                   # Container-Infrastruktur
│   ├── docker-compose.yml    # perception + serial_bridge
│   └── perception/
├── docs/                     # Dokumentation
├── config/                   # Runtime-Konfiguration
├── maps/                     # SLAM-Karten
└── scripts/                  # Deploy-Scripts
```

---

## Dokumentation

| Nr | Dokument | Inhalt |
|----|----------|--------|
| 01 | `01-Pi-OS-flashen.md` | Raspberry Pi OS, SSH, Docker |
| 02 | `02-hailo-setup.md` | Hailo-8L Treiber, Benchmark |
| 03 | `03-ros2-docker.md` | ROS 2 Container, URDF |
| 04 | `04-esp32-firmware.md` | PlatformIO, Serial-Bridge |
| 05 | `05-git-vscode-platformio.md` | Entwicklungsumgebung |
| 06 | `06-git-workflow.md` | Git Mac ↔ GitHub ↔ Pi |
| 07 | `07-hardware-diagnose.md` | Debugging, Fehlersuche |
| 08 | `08-entwicklerdoku-status.md` | Projektstatus, Checklisten |

---

## Hardware

| Komponente | Modell | Funktion | Preis |
|------------|--------|----------|-------|
| Compute | Raspberry Pi 5 (8GB) | ROS 2 Host, SLAM | 82,90 € |
| AI | Hailo-8L Kit | Objekterkennung (13 TOPS) | 78,85 € |
| LiDAR | RPLIDAR A1 | 360° Scan, 8k Samples/s | 89,90 € |
| Kamera | IMX296 Global Shutter | Bewegungserkennung | 58,90 € |
| MCU | ESP32-S3 XIAO | Echtzeit-Control | 8,50 € |
| Motoren | JGA25-370 (2×) | 170 RPM, Encoder | 20,37 € |
| Treiber | Cytron MDD3A | Dual-PWM, 3A | 8,50 € |

**Gesamtkosten:** 482,48 € (35% unter Referenz-Budget)

---

## Firmware v0.3.0-serial Highlights

| Feature | Beschreibung |
|---------|--------------|
| **Serial-Bridge** | Einfaches V:x,W:y Protokoll (kein micro-ROS) |
| **Dual-PWM** | Korrekte MDD3A-Ansteuerung (nicht PWM+DIR) |
| **Deadzone-Kompensation** | Auch kleine Geschwindigkeiten funktionieren |
| **Failsafe** | Motoren stoppen nach 500 ms ohne Befehl |
| **LED-Status** | Breathing (Idle), Dauerlicht (Bewegung), Blinken (Failsafe) |
| **Differential Drive** | Kinematik-Berechnung auf ESP32 |

---

## Serial-Protokoll

| Richtung | Format | Beispiel |
|----------|--------|----------|
| Host → ESP32 | `V:<m/s>,W:<rad/s>\n` | `V:0.20,W:0.50\n` |
| ESP32 → Host | `OK:<v>,<w>` | `OK:0.200,0.500` |
| ESP32 → Host | `FAILSAFE:TIMEOUT` | Nach 500ms ohne Befehl |
| ESP32 → Host | `ERR:CMD_OUT_OF_RANGE` | Werte außerhalb Limit |

---

## Pin-Belegung ESP32-S3 XIAO

| Komponente | Signal | Pin | PWM-Kanal |
|------------|--------|-----|-----------|
| Motor Links | PWM A (vorwärts) | D0 | CH 0 |
| Motor Links | PWM B (rückwärts) | D1 | CH 1 |
| Motor Rechts | PWM A (vorwärts) | D2 | CH 2 |
| Motor Rechts | PWM B (rückwärts) | D3 | CH 3 |
| IMU | SDA | D4 | – |
| IMU | SCL | D5 | – |
| Encoder Links | Phase A | D6 | – |
| Encoder Rechts | Phase A | D7 | – |
| LED-Strip | MOSFET Gate | D10 | CH 4 |

---

## Git-Workflow

```bash
# Mac: Entwickeln und pushen
git pull origin main
# ... arbeiten ...
git add . && git commit -m "feat: Beschreibung"
git push origin main

# Pi: Deployen
ssh pi@rover "cd ~/amr-platform && git pull && cd docker && docker compose up -d"
```

---

## Standards

- **REP-103:** SI-Einheiten (Meter, Radiant)
- **REP-105:** TF-Frames (map → odom → base_link)
- **Safety:** Failsafe-Timeout (500ms), LED-Feedback

---

## Lizenz

MIT License – siehe [LICENSE](LICENSE)

---

*Aktualisiert: 2025-12-12 | Autor: Jan Unger | <https://github.com/unger-robotics/amr-platform>*
