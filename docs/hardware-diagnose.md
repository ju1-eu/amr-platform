# Hardware-Diagnose & Debugging

Systematische Fehlersuche für das AMR-System. Wir arbeiten nach dem Prinzip: **Host → Container → Hardware**.

> **Stand:** 2025-12-12 | **Firmware:** v0.3.0-serial | **Architektur:** Serial-Bridge

---

## 1. Verbindung zum Roboter

### SSH-Zugang

```bash
# Via Hostname (mDNS)
ssh pi@rover

# Via IP-Adresse (falls mDNS nicht funktioniert)
ssh pi@192.168.1.24
```

### Grundlegende System-Info

```bash
# IP-Adressen anzeigen
ip a
hostname -I

# OS-Version
cat /etc/os-release

# Kernel
uname -r
```

---

## 2. System-Gesundheit (Host-Ebene)

### 2.1 Temperatur

```bash
# Einmalige Abfrage
vcgencmd measure_temp

# Live-Monitoring (alle 2 Sekunden)
watch -n 2 vcgencmd measure_temp
```

**Grenzwerte:**

| Temperatur | Status | Aktion |
|------------|--------|--------|
| < 60°C | ✅ Optimal | — |
| 60–70°C | ✅ Normal | — |
| 70–80°C | ⚠️ Warm | Lüfter prüfen |
| > 80°C | ❌ Throttling | Kühlung verbessern |
| > 85°C | ❌ Kritisch | System herunterfahren |

### 2.2 Speicher & CPU

```bash
# RAM-Nutzung
free -h

# CPU-Auslastung (interaktiv)
htop

# Disk-Nutzung
df -h
```

### 2.3 System-Updates

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt autoremove -y
sudo reboot
```

---

## 3. USB-Geräte (Host-Ebene)

### 3.1 Geräte auflisten

```bash
# Alle USB-Geräte
lsusb

# Serielle Ports
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

**Erwartete Geräte:**

| Gerät | USB-ID | Device |
|-------|--------|--------|
| RPLIDAR A1 | `10c4:ea60` (CP210x) | `/dev/ttyUSB0` |
| ESP32-S3 XIAO | `303a:1001` (USB-CDC) | `/dev/ttyACM0` |

### 3.2 Geräte-Details

```bash
# RPLIDAR Details
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|manufacturer"

# ESP32 Details
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|manufacturer"
```

### 3.3 Berechtigungen

```bash
# Aktueller User in dialout-Gruppe?
groups

# Falls nicht:
sudo usermod -aG dialout $USER
# Dann ausloggen und neu einloggen!
```

---

## 4. PCIe & Hailo-8L (Host-Ebene)

```bash
# PCIe-Geräte
lspci

# Hailo spezifisch
lspci | grep -i hailo

# Kernel-Modul geladen?
lsmod | grep hailo

# Hailo-Version
hailortcli --version

# Hailo-Status
hailortcli fw-control identify
```

**Erwartete Ausgabe:**

```
0000:01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor
```

---

## 5. Kamera IMX296 (Host-Ebene)

```bash
# Kernel-Log prüfen
dmesg | grep -i imx

# Kamera-Liste (Bookworm: rpicam-hello, nicht libcamera-hello!)
rpicam-hello --list-cameras

# Testbild aufnehmen
rpicam-still -o /tmp/test.jpg
ls -la /tmp/test.jpg
```

**Erwartete Ausgabe:**

```
0 : imx296 [1456x1088 10-bit MONO] (/base/axi/pcie@120000/rp1/i2c@88000/imx296@1a)
```

---

## 6. Docker-Container

### 6.1 Status prüfen

```bash
# Laufende Container
docker ps

# Alle Container (auch gestoppte)
docker ps -a

# Container-Ressourcen
docker stats --no-stream
```

**Erwartete Container:**

| Name | Image | Status |
|------|-------|--------|
| `amr_perception` | `amr_perception:jazzy` | Up |
| `amr_serial_bridge` | `ros:jazzy-ros-base` | Up |

### 6.2 Logs ansehen

```bash
# Perception-Container
docker logs -f amr_perception

# Serial-Bridge (ersetzt micro-ROS)
docker logs -f amr_serial_bridge

# Letzte 50 Zeilen
docker logs --tail 50 amr_serial_bridge
```

### 6.3 Container neu starten

```bash
cd ~/amr-platform/docker

# Sanfter Neustart
docker compose restart

# Komplett neu bauen
docker compose down
docker compose up -d --build
```

---

## 7. Diagnose im Container

### 7.1 In Container wechseln

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
```

### 7.2 ROS 2 Diagnose

```bash
# Im Container:

# Alle Nodes
ros2 node list
# Erwartung: /serial_bridge

# Alle Topics
ros2 topic list

# Topic-Frequenz prüfen
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Topic-Inhalt anzeigen
ros2 topic echo /scan --once
ros2 topic echo /cmd_vel --once

# TF-Baum visualisieren
ros2 run tf2_tools view_frames
# Generiert: frames.pdf
```

### 7.3 USB-Geräte im Container

```bash
# Im Container:

# USB-Geräte
lsusb

# Serielle Ports
ls -la /dev/ttyUSB* /dev/ttyACM* /dev/hailo* 2>/dev/null
```

### 7.4 Serial-Bridge Diagnose

```bash
# Bridge-Node aktiv?
ros2 node list | grep serial_bridge

# cmd_vel Subscriber vorhanden?
ros2 topic info /cmd_vel
# Erwartung: Subscription count: 1

# Manuell Befehl senden
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}" --once
```

---

## 8. Typische Probleme & Lösungen

### Problem: RPLIDAR "Operation timeout"

**Symptom:** LiDAR-Motor dreht nicht oder stoppt.

**Ursache:** Spannungsabfall durch dünnes/langes USB-Kabel.

**Lösung:**

```bash
# Kurzes, dickes Kabel verwenden (< 50cm, 24 AWG)
# Oder: Powered USB-Hub

# USB-Stromlimit prüfen
cat /boot/firmware/config.txt | grep usb_max_current
# Sollte enthalten: usb_max_current_enable=1
```

### Problem: ESP32 nicht erkannt

**Symptom:** `/dev/ttyACM0` existiert nicht.

**Lösung:**

```bash
# USB-Gerät prüfen
lsusb | grep -i espressif

# Falls nicht gefunden: Boot-Mode erzwingen
# 1. BOOT-Taste halten
# 2. RESET drücken und loslassen
# 3. BOOT loslassen

# Kernel-Log prüfen
dmesg | tail -20
```

### Problem: Serial-Bridge verbindet nicht

**Symptom:** `ros2 node list` zeigt `/serial_bridge` nicht.

**Lösung:**

```bash
# Container-Logs prüfen
docker logs -f amr_serial_bridge

# Device korrekt durchgereicht?
docker exec amr_serial_bridge ls -la /dev/ttyACM0

# Container neu starten
docker compose restart serial_bridge
```

### Problem: Motoren stoppen sofort (Failsafe)

**Symptom:** LED blinkt schnell, Motoren stoppen nach 500ms.

**Ursache:** Kein kontinuierlicher Heartbeat von der Bridge.

**Lösung:**

```bash
# Bridge-Logs prüfen
docker compose logs -f serial_bridge

# Prüfen ob cmd_vel gesendet wird
ros2 topic echo /cmd_vel

# Container neu starten
docker compose restart serial_bridge
```

### Problem: Motor reagiert nicht auf kleine Geschwindigkeiten

**Symptom:** Bei V:0.0,W:0.3 passiert nichts.

**Ursache:** PWM unter Deadzone (< 35).

**Lösung:** Firmware v0.3.0-serial hat Deadzone-Kompensation eingebaut. Falls alte Firmware: Neu flashen.

```bash
# Auf Mac:
cd /Users/jan/daten/start/IoT/AMR/amr-platform/firmware_serial
pio run --target upload
```

### Problem: Kamera wird nicht erkannt

**Symptom:** `rpicam-hello --list-cameras` zeigt "No cameras".

**Lösung:**

```bash
# CSI-Kabel prüfen (Kontakte zur Platine!)
# Kabel neu einstecken

# Kernel-Log
dmesg | grep -i imx

# Nach Neustart testen
sudo reboot
```

### Problem: Hailo nicht erkannt

**Symptom:** `lspci | grep Hailo` zeigt nichts.

**Lösung:**

```bash
# Physische Verbindung prüfen (HAT sitzt plan?)
# Komplett ausschalten (nicht nur reboot)
sudo shutdown -h now
# Strom trennen, 10s warten, wieder einschalten
```

---

## 9. Serial-Protokoll Debugging

### Direkter Serial-Test (auf Pi, ohne Docker)

```bash
# pyserial installieren
pip3 install pyserial --break-system-packages

# Python-Script für direkten Test
python3 << 'EOF'
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # ESP32 Reset abwarten

# Vorwärts fahren
ser.write(b'V:0.2,W:0.0\n')
print(ser.readline().decode().strip())

time.sleep(2)

# Stopp
ser.write(b'V:0.0,W:0.0\n')
print(ser.readline().decode().strip())

ser.close()
EOF
```

### Serial Monitor (ohne ROS)

```bash
# Mit screen
screen /dev/ttyACM0 115200

# Oder mit minicom
minicom -D /dev/ttyACM0 -b 115200

# Befehle eingeben:
# V:0.2,W:0.0    → Vorwärts
# V:0.0,W:0.5    → Drehen
# V:0.0,W:0.0    → Stopp
```

### Protokoll-Referenz

| Richtung | Format | Beispiel |
|----------|--------|----------|
| Host → ESP32 | `V:<m/s>,W:<rad/s>\n` | `V:0.20,W:0.50\n` |
| ESP32 → Host | `OK:<v>,<w>` | `OK:0.200,0.500` |
| ESP32 → Host | `FAILSAFE:TIMEOUT` | Nach 500ms ohne Befehl |
| ESP32 → Host | `ERR:CMD_OUT_OF_RANGE` | Werte außerhalb Limit |
| ESP32 → Host | `READY` | Nach Boot |

---

## 10. Schnell-Referenz

### Tägliche Befehle

```bash
# System-Status
ssh pi@rover
vcgencmd measure_temp
docker ps

# Updates holen
cd ~/amr-platform && git pull

# Container neu starten
cd ~/amr-platform/docker && docker compose restart

# In ROS-Container
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

### Diagnose-Workflow

```
1. Host erreichbar?     → ping rover
2. SSH funktioniert?    → ssh pi@rover
3. Container laufen?    → docker ps
4. USB-Geräte da?       → lsusb && ls /dev/ttyACM*
5. Serial-Bridge OK?    → docker logs amr_serial_bridge
6. ROS 2 Node?          → docker exec amr_perception ros2 node list
7. Teleop-Test          → ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Aliase für .bashrc (Pi)

```bash
# Status-Befehle
alias temp='vcgencmd measure_temp'
alias status='docker ps && echo "---" && vcgencmd measure_temp'

# Docker-Shortcuts
alias ros='docker exec -it amr_perception bash'
alias logs='docker compose -f ~/amr-platform/docker/docker-compose.yml logs -f'
alias restart='cd ~/amr-platform/docker && docker compose restart'

# Serial-Bridge spezifisch
alias bridge-logs='docker logs -f amr_serial_bridge'
alias bridge-restart='docker compose -f ~/amr-platform/docker/docker-compose.yml restart serial_bridge'

# Update-Workflow
alias update='cd ~/amr-platform && git pull && cd docker && docker compose down && docker compose up -d'
```

---

## 11. Architektur-Übersicht

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        AMR System-Architektur                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌─────────────────────────────────────────────────────────────────┐   │
│   │                    Raspberry Pi 5 (Host)                        │   │
│   │  ┌─────────────────┐    ┌─────────────────────────────────────┐ │   │
│   │  │ amr_perception  │    │ amr_serial_bridge                   │ │   │
│   │  │ (Docker)        │    │ (Docker)                            │ │   │
│   │  │                 │    │                                     │ │   │
│   │  │ • ROS 2 Jazzy   │────│ • /cmd_vel Subscriber               │ │   │
│   │  │ • Teleop        │    │ • Serial Writer (20 Hz)             │ │   │
│   │  │ • RPLIDAR       │    │ • Failsafe Handling                 │ │   │
│   │  │ • Hailo-8L      │    │                                     │ │   │
│   │  └─────────────────┘    └──────────────┬──────────────────────┘ │   │
│   │                                        │ /dev/ttyACM0           │   │
│   └────────────────────────────────────────┼────────────────────────┘   │
│                                            │ USB-CDC                    │
│   ┌────────────────────────────────────────┼────────────────────────┐   │
│   │                    ESP32-S3 XIAO       │                        │   │
│   │                                        ▼                        │   │
│   │  ┌─────────────────────────────────────────────────────────────┐│   │
│   │  │ Serial-Bridge Firmware v0.3.0                               ││   │
│   │  │ • V:x,W:y Parser                                            ││   │
│   │  │ • Differential Drive Kinematik                              ││   │
│   │  │ • Deadzone-Kompensation                                     ││   │
│   │  │ • 500ms Failsafe Timeout                                    ││   │
│   │  └─────────────────────────────────────────────────────────────┘│   │
│   │           │ D0/D1              │ D2/D3                          │   │
│   └───────────┼────────────────────┼────────────────────────────────┘   │
│               ▼                    ▼                                    │
│   ┌───────────────────┐  ┌───────────────────┐                          │
│   │ Cytron MDD3A      │  │ (Dual-PWM)        │                          │
│   │ Motor Links       │  │ Motor Rechts      │                          │
│   └───────────────────┘  └───────────────────┘                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

*Aktualisiert: 2025-12-12 | Firmware: v0.3.0-serial | Phase 1 abgeschlossen*
