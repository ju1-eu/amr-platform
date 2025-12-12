# AMR Entwicklerdokumentation

> **Stand:** 2025-12-12 | **Phase:** 1 ✅ ABGESCHLOSSEN | **Firmware:** v0.3.0-serial

---

## 1. Aktueller Projektstatus

### 1.1 Was funktioniert ✅

| Komponente | Status | Validiert |
|------------|--------|-----------|
| Raspberry Pi 5 | ✅ | OS, Docker, SSH |
| Hailo-8L | ✅ | 31 FPS, HailoRT 4.23.0 |
| RPLIDAR A1 | ✅ | 7.5 Hz, TF korrekt |
| Kamera IMX296 | ✅ | libcamera 0.3+ |
| ESP32-S3 XIAO | ✅ | USB-CDC erkannt |
| Motor Forward | ✅ | Beide Räder vorwärts |
| Motor Backward | ✅ | Beide Räder rückwärts |
| Motor Turn Left | ✅ | Differential Drive |
| Motor Turn Right | ✅ | Differential Drive |
| LED-Status (D10) | ✅ | Breathing + Blink |
| PWM (20 kHz) | ✅ | Unhörbar |
| Failsafe (500ms) | ✅ | Motoren stoppen bei Timeout |
| Deadzone-Kompensation | ✅ | Auch kleine Geschwindigkeiten funktionieren |
| Git-Workflow | ✅ | Mac → GitHub → Pi |
| Docker-Stack | ✅ | perception + serial_bridge Container |
| **Serial-Bridge** | ✅ | ROS 2 /cmd_vel → ESP32 |
| **Teleop** | ✅ | Tastatursteuerung funktioniert |

### 1.2 Was blockiert ist ⚠️

| Problem | Ursache | Workaround |
|---------|---------|------------|
| **micro-ROS Build** | Python 3.13 inkompatibel | ✅ Serial-Bridge implementiert |

### 1.3 Offene Punkte (Phase 2)

- [ ] Encoder-Kalibrierung
- [ ] Odometrie-Publisher
- [ ] PID-Regelung

---

## 2. Hardware-Konfiguration

### 2.1 Pin-Belegung ESP32-S3 XIAO

| Funktion | Pin | PWM-Kanal | Notiz |
|----------|-----|-----------|-------|
| Motor Links A (vorwärts) | D0 | CH 0 | Cytron M1A |
| Motor Links B (rückwärts) | D1 | CH 1 | Cytron M1B |
| Motor Rechts A (vorwärts) | D2 | CH 2 | Cytron M2A |
| Motor Rechts B (rückwärts) | D3 | CH 3 | Cytron M2B |
| I2C SDA (IMU) | D4 | - | MPU6050 |
| I2C SCL (IMU) | D5 | - | MPU6050 |
| Encoder Links | D6 | - | Interrupt |
| Encoder Rechts | D7 | - | Interrupt |
| LED-Strip MOSFET | D10 | CH 4 | IRLZ24N |

### 2.2 Cytron MDD3A - Dual-PWM Steuerung

**WICHTIG:** Der MDD3A verwendet **Dual-PWM**, NICHT PWM+DIR!

| M1A | M1B | Ergebnis |
|-----|-----|----------|
| PWM | 0 | Vorwärts |
| 0 | PWM | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| PWM | PWM | Active Brake |

---

## 3. Software-Architektur

### 3.1 Serial-Bridge (Phase 1 Lösung)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Serial-Bridge Architektur                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ROS 2 (Docker)              USB-Serial              ESP32-S3          │
│  ┌──────────────┐            ┌────────┐            ┌──────────────┐    │
│  │ Teleop       │            │ Bridge │            │ Differential │    │
│  │ /cmd_vel     │───────────▶│ Node   │───────────▶│ Drive        │    │
│  │ Twist msg    │            │ Python │  V:0.2,    │ Kinematik    │    │
│  └──────────────┘            └────────┘  W:0.5\n   └──────────────┘    │
│                               /dev/ttyACM0                              │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Serial-Protokoll

| Richtung | Format | Beispiel |
|----------|--------|----------|
| Host → ESP32 | `V:<m/s>,W:<rad/s>\n` | `V:0.20,W:0.50\n` |
| ESP32 → Host | `OK:<v>,<w>` | `OK:0.20,0.50` |
| ESP32 → Host | `FAILSAFE:TIMEOUT` | Nach 500ms ohne Befehl |
| ESP32 → Host | `ERR:CMD_OUT_OF_RANGE` | Bei ungültigen Werten |

### 3.3 Docker Container

| Container | Image | Funktion |
|-----------|-------|----------|
| `amr_perception` | amr_perception:jazzy | ROS 2 Stack, Teleop |
| `amr_serial_bridge` | ros:jazzy-ros-base | Serial-Bridge Node |

---

## 4. Projekt-Struktur

```
amr-platform/                    # GitHub: unger-robotics/amr-platform
├── firmware/                    # micro-ROS Firmware (nicht verwendet)
├── firmware_serial/             # Serial-Bridge Firmware ✅
│   ├── platformio.ini
│   ├── include/config.h
│   └── src/main.cpp
├── firmware_test/               # Hardware-Test Firmware
├── docker/
│   ├── docker-compose.yml       # perception + serial_bridge
│   └── perception/
├── ros2_ws/
│   └── src/
│       ├── amr_description/     # URDF
│       ├── amr_bringup/
│       └── amr_serial_bridge/   # ROS 2 Serial Bridge ✅
│           ├── amr_serial_bridge/
│           │   └── serial_bridge.py
│           ├── launch/
│           ├── package.xml
│           └── setup.py
└── scripts/
    └── deploy.sh
```

---

## 5. Validierte Test-Befehle

### 5.1 ESP32 Firmware flashen (auf Mac)

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform/firmware_serial
pio run --target upload
pio device monitor
```

**Serial-Befehle:**

- `V:0.2,W:0.0` → Vorwärts
- `V:0.0,W:0.5` → Drehen
- `V:0.0,W:0.0` → Stopp

### 5.2 Docker-Stack (auf Pi)

```bash
ssh pi@rover
cd ~/amr-platform/docker
docker compose up -d
docker compose logs -f serial_bridge
```

### 5.3 Teleop-Test

```bash
# Terminal 1: Serial Bridge läuft bereits via Docker

# Terminal 2: Teleop starten
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:** `i`=Vorwärts, `,`=Rückwärts, `j`/`l`=Drehen, `k`=Stopp

### 5.4 Git-Workflow

```bash
# Mac: Entwickeln
cd /Users/jan/daten/start/IoT/AMR/amr-platform
git pull origin main
# ... arbeiten ...
git add .
git commit -m "feat: Beschreibung"
git push origin main

# Pi: Deployen
ssh pi@rover
cd ~/amr-platform
git pull origin main
docker compose -f docker/docker-compose.yml up -d
```

---

## 6. Phase 1 Abschluss-Protokoll

### 6.1 Durchgeführte Tests

| Test | Ergebnis | Datum |
|------|----------|-------|
| Motor vorwärts | ✅ Beide Räder drehen | 2025-12-12 |
| Motor rückwärts | ✅ Beide Räder drehen | 2025-12-12 |
| Differential Drive links | ✅ Roboter dreht | 2025-12-12 |
| Differential Drive rechts | ✅ Roboter dreht | 2025-12-12 |
| Failsafe Timeout | ✅ Motoren stoppen nach 500ms | 2025-12-12 |
| Teleop via Docker | ✅ Tastatursteuerung funktioniert | 2025-12-12 |

### 6.2 Lessons Learned

1. **micro-ROS + Python 3.13 = Inkompatibel** → Serial-Bridge als Workaround
2. **Deadzone-Kompensation notwendig** für kleine Geschwindigkeiten
3. **ESP32 Arduino 2.x API** verwendet `ledcSetup()` + `ledcAttachPin()`, nicht `ledcAttach()`
4. **Docker `ros:jazzy-ros-base`** hat kein pip3 → `apt install python3-serial`

---

## 7. Referenzen

| Dokument | Beschreibung |
|----------|--------------|
| `AMR_Implementierungsplan.md` | Phasen 0–6 |
| `Industriestandards-AMR.md` | REP-103, REP-105, Safety |
| `06-git-workflow.md` | Git Mac ↔ Pi |
| `ros2_ws/src/amr_serial_bridge/SETUP.md` | Serial-Bridge Setup |

---

## 8. Changelog

| Datum | Version | Änderung |
|-------|---------|----------|
| 2025-12-12 | v0.3.0-serial | **Phase 1 abgeschlossen** |
| 2025-12-12 | v0.3.0-serial | Serial-Bridge Firmware mit Deadzone-Kompensation |
| 2025-12-12 | - | ROS 2 Serial Bridge Package erstellt |
| 2025-12-12 | - | Docker Integration (serial_bridge Container) |
| 2025-12-12 | - | Teleop-Test erfolgreich |
| 2025-12-12 | - | Git-Workflow Mac ↔ GitHub ↔ Pi eingerichtet |

---

## 9. Nächste Schritte: Phase 2

1. **Encoder-Kalibrierung** (10-Umdrehungen-Test)
2. **Odometrie auf ESP32** berechnen
3. **Serial-Protokoll erweitern:** `ODOM:left_ticks,right_ticks\n`
4. **ROS 2 `/odom` Publisher** in Bridge-Node
5. **TF-Broadcast:** `odom` → `base_link`

---

*Dokumentation erstellt: 2025-12-12 | Autor: Jan Unger | FH Aachen*
