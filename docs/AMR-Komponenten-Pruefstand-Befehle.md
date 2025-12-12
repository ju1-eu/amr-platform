# Befehls-Liste Komponenten-Prüfstand (v1.3)

> **Stand:** 2025-12-12 | **Firmware:** v0.3.0-serial | **Phase:** 1 ✅ Abgeschlossen

---

## Übersicht: Was ist validiert?

| Komponente | Status | Bemerkung |
|------------|--------|-----------|
| Raspberry Pi 5 | ✅ | OS, Docker, Hailo erkannt |
| RPLIDAR A1 | ✅ | 7.5 Hz, TF korrekt |
| Kamera IMX296 | ✅ | libcamera 0.3+ |
| Hailo-8L | ✅ | 31 FPS, HailoRT 4.23.0 |
| ESP32-S3 XIAO | ✅ | Serial-Bridge Firmware |
| Motoren | ✅ | Teleop validiert |
| Differential Drive | ✅ | Links/Rechts drehen |
| Failsafe | ✅ | 500ms Timeout |

---

## 1. Raspberry Pi 5 — Das Fundament

### 1.1 Host-System prüfen

```bash
ssh pi@rover

# System aktualisieren
sudo apt update && sudo apt full-upgrade -y

# Temperatur prüfen (Ziel: < 50°C Leerlauf)
vcgencmd measure_temp

# Hailo-8L erkennen
lspci | grep Hailo
```

**Erwartetes Ergebnis:**

```
temp=48.3'C
0001:01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor
```

### 1.2 Docker-Stack

```bash
# Projektstruktur
ls -la ~/amr
# Erwartung: amr -> /home/pi/amr-platform (Symlink)

# Container starten
cd ~/amr-platform/docker
docker compose up -d

# Status prüfen
docker compose ps
```

**Erwartetes Ergebnis:**

```
NAME               IMAGE                    STATUS
amr_perception     amr_perception:jazzy     Up
amr_serial_bridge  ros:jazzy-ros-base       Up
```

---

## 2. ESP32-S3 XIAO — Serial-Bridge

### 2.1 Hardware-Erkennung (auf Pi)

```bash
# USB-Gerät prüfen (ESP32-S3 = USB-CDC, kein CH340!)
lsusb | grep -i "espressif"

# Serieller Port
ls -l /dev/ttyACM*

# Rechte setzen (einmalig, danach Logout/Login)
sudo usermod -aG dialout $USER
```

**Erwartetes Ergebnis:**

```
Bus 001 Device 003: ID 303a:1001 Espressif USB JTAG/serial debug unit
/dev/ttyACM0 existiert
```

### 2.2 Firmware flashen (auf Mac)

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform/firmware_serial

# Kompilieren + Flashen
pio run --target upload

# Serial Monitor
pio device monitor
```

**Erwartete Ausgabe:**

```
AMR Serial-Bridge v0.3.0
Format: V:<m/s>,W:<rad/s>
READY
```

**LED-Status nach Flash:**

| Muster | Bedeutung |
|--------|-----------|
| Aufdimmen → Abdimmen | Boot-Animation |
| Langsames Pulsieren (Breathing) | Idle, wartet auf Befehle |
| Dauerlicht | Bewegung aktiv |
| Schnelles Blinken | Failsafe (Timeout) |

### 2.3 Manueller Serial-Test (auf Mac)

Im Serial Monitor eingeben (mit Newline):

```
V:0.2,W:0.0
```

**Erwartung:** Beide Räder drehen vorwärts, Antwort: `OK:0.200,0.000`

```
V:0.0,W:0.5
```

**Erwartung:** Roboter dreht sich (Differential Drive), Antwort: `OK:0.000,0.500`

```
V:0.0,W:0.0
```

**Erwartung:** Stopp, Antwort: `OK:0.000,0.000`

### 2.4 Serial-Bridge Container starten (auf Pi)

```bash
cd ~/amr-platform/docker

# Container starten
docker compose up -d

# Logs prüfen
docker compose logs -f serial_bridge
```

**Erwartetes Ergebnis:**

```
[INFO] [serial_bridge]: Serial Bridge gestartet: /dev/ttyACM0 @ 115200 baud
[INFO] [serial_bridge]: ESP32 bereit
```

### 2.5 Motor-Test via ROS 2

```bash
# In einem neuen Terminal
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash

# Node prüfen
ros2 node list
# Erwartung: /serial_bridge

# Motor vorwärts (0.2 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Motor rückwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.2}}" --once

# Drehung links
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Drehung rechts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: -0.5}}" --once

# Stopp
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### 2.6 Teleop-Test (empfohlen)

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:**

| Taste | Aktion |
|-------|--------|
| `i` | Vorwärts |
| `,` | Rückwärts |
| `j` | Links drehen |
| `l` | Rechts drehen |
| `k` | Stopp |
| `q`/`z` | Geschwindigkeit +/- |

### 2.7 Failsafe-Test

```bash
# Serial-Bridge Container stoppen
docker compose stop serial_bridge

# Beobachten: LED muss schnell blinken, Motoren stoppen nach 500ms

# Container wieder starten
docker compose start serial_bridge
```

---

## 3. RPLIDAR A1 — LiDAR-Sensor

### 3.1 Hardware-Check

```bash
# USB-Erkennung (Silicon Labs CP210x)
lsusb | grep -i "silicon"

# Serieller Port
ls -la /dev/ttyUSB0
```

### 3.2 ROS 2 Node starten

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash

# LiDAR starten
ros2 launch rplidar_ros rplidar.launch.py frame_id:=laser_frame
```

**Erwartetes Ergebnis:**

- Motor dreht hörbar
- Keine "Operation timeout" Fehler

### 3.3 Daten prüfen

```bash
# In neuem Terminal
docker exec -it amr_perception bash

# Scan-Rate prüfen
ros2 topic hz /scan
# Erwartung: average rate: 7.5 Hz

# Daten ansehen
ros2 topic echo /scan --once
```

---

## 4. Kamera (IMX296) & Hailo-8L

### 4.1 Kamera-Erkennung

```bash
# Kernel-Log
sudo dmesg | grep imx296

# Kamera-Liste (neue Befehle für Bookworm!)
rpicam-hello --list-cameras
```

**Erwartetes Ergebnis:**

```
Available cameras
-----------------
0 : imx296 [1456x1088 10-bit MONO] (/base/soc/i2c0mux/...)
```

### 4.2 Hailo-Validierung

```bash
# Version prüfen (MUSS 4.23.0 sein!)
hailortcli --version

# Firmware prüfen
hailortcli fw-control identify

# Benchmark
cd ~/hailo_models
hailortcli benchmark yolov5s.hef
```

**Erwartetes Ergebnis:**

```
HailoRT: 4.23.0
Firmware Version: 4.23.0
FPS: ~31, Latency: ~30ms
```

---

## 5. TF-Baum validieren

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash

# Robot State Publisher starten
ros2 launch amr_description display.launch.py

# In neuem Terminal: TF prüfen
ros2 run tf2_ros tf2_echo base_link laser_frame
```

**Erwartete Translation:**

```
Translation: [0.060, 0.000, 0.045]
```

---

## 6. Git-Workflow

### 6.1 Auf dem Mac entwickeln

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform

# Vor der Arbeit
git pull origin main

# Nach Änderungen
git add .
git commit -m "feat: Beschreibung"
git push origin main
```

### 6.2 Auf dem Pi deployen

```bash
cd ~/amr-platform
git pull origin main

# Container neu starten (falls Docker-Dateien geändert)
cd docker
docker compose down
docker compose up -d
```

---

## Status-Checkliste Phase 1 ✅

### ESP32 Serial-Bridge Firmware

- [x] ESP32-S3 XIAO an Pi angeschlossen
- [x] `/dev/ttyACM0` erkannt
- [x] Firmware v0.3.0-serial geflasht
- [x] LED Boot-Animation sichtbar
- [x] LED Breathing (Idle)
- [x] Serial-Bridge Container verbunden
- [x] `/serial_bridge` Node sichtbar
- [x] Motor vorwärts funktioniert
- [x] Motor rückwärts funktioniert
- [x] Drehung links funktioniert
- [x] Drehung rechts funktioniert
- [x] Failsafe-Test bestanden (500ms Timeout)
- [x] Teleop-Test bestanden

### Cytron MDD3A Verkabelung

| ESP32 Pin | MDD3A | Funktion |
|-----------|-------|----------|
| D0 | M1A | Motor Links Vorwärts |
| D1 | M1B | Motor Links Rückwärts |
| D2 | M2A | Motor Rechts Vorwärts |
| D3 | M2B | Motor Rechts Rückwärts |
| GND | GND | Masse |

**Wichtig:** MDD3A nutzt **Dual-PWM**, NICHT PWM+DIR!

---

## Performance-Zusammenfassung

| Komponente | Metrik | Wert | Status |
|------------|--------|------|--------|
| **Hailo-8L** | FPS | 31.32 | ✅ |
| | Latenz | 29.73 ms | ✅ |
| **RPLIDAR A1** | Scan-Rate | 7.5 Hz | ✅ |
| | Range | 0.19–4.0 m | ✅ |
| **ESP32 Firmware** | Version | v0.3.0-serial | ✅ |
| | Failsafe | 500 ms | ✅ |
| | PWM Deadzone | 35 (kompensiert) | ✅ |
| **Serial Bridge** | Send-Rate | 20 Hz | ✅ |
| | Latenz | < 50 ms | ✅ |

---

## Fehlerbehebung

### ESP32 wird nicht erkannt

```bash
# Boot-Modus erzwingen:
# 1. BOOT-Taste gedrückt halten
# 2. RESET-Taste drücken und loslassen
# 3. BOOT-Taste loslassen
# 4. pio run --target upload
```

### Serial-Bridge verbindet nicht

```bash
# Container-Logs prüfen
docker compose logs serial_bridge

# Device durchgereicht?
docker exec amr_serial_bridge ls -la /dev/ttyACM0

# Neustart
docker compose restart serial_bridge
```

### Motor dreht nicht

1. **12V Versorgung messen** (Multimeter am Cytron VIN)
2. **PWM-Signal prüfen** (LED an D0/D2)
3. **Cytron Status-LED** (Grün = OK, Rot = Fehler)
4. **Deadzone prüfen** (PWM muss > 35 sein, wird automatisch kompensiert)

### Motoren stoppen sofort (Failsafe)

```bash
# Serial-Bridge Logs prüfen
docker compose logs -f serial_bridge

# Heartbeat-Problem: Bridge sendet nicht kontinuierlich
# → Container neu starten
docker compose restart serial_bridge
```

### Alter Container blockiert

```bash
# Alle Container stoppen und entfernen
docker compose down

# Neu starten
docker compose up -d
```

---

## Nächste Schritte: Phase 2

| Phase | Aufgabe | Status |
|-------|---------|--------|
| 1 | ESP32 Serial-Bridge, Motor-Test, Teleop | ✅ Abgeschlossen |
| 2 | Encoder-Kalibrierung, /odom Publisher | ◄── Aktuell |
| 3 | SLAM mit slam_toolbox | ⬜ Offen |
| 4 | Nav2 autonome Navigation | ⬜ Offen |
| 5 | Kamera + YOLOv8 auf Hailo | ⬜ Offen |

### Phase 2 Aufgaben

1. **Encoder-Kalibrierung** (10-Umdrehungen-Test)
2. **Odometrie auf ESP32** berechnen
3. **Serial-Protokoll erweitern:** `ODOM:left_ticks,right_ticks,x,y,theta\n`
4. **ROS 2 `/odom` Publisher** in Bridge-Node
5. **TF-Broadcast:** `odom` → `base_link`

---

*Aktualisiert: 2025-12-12 | Firmware: v0.3.0-serial | Git: github.com/unger-robotics/amr-platform*
