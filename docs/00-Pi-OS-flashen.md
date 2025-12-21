---
title: "System-Installation (Pi 5 Host): Fundament für AMR-Stack (Docker + ROS 2 Humble)"
status: "active"
updated: "2025-12-20"
project_phase: "1–3 abgeschlossen | Phase 4 als Nächstes (URDF/TF/EKF)"
repo: "amr-platform"
---

# System-Installation: Fundament (Raspberry Pi 5 Host)

**Stand:** 2025-12-20
**Projektstatus:** Phase 1–3 ✅ | **Nächste Phase:** 4 (URDF/TF/EKF)
**Ziel:** Pi 5 headless + Docker so einrichten, dass der AMR-Stack reproduzierbar läuft (micro-ROS Agent, ROS 2 Workspace, LiDAR-Device-Passthrough).

---

## 0) Ziel & Definition of Done (DoD)

### Ziel

- Raspberry Pi 5 läuft **headless** (SSH, WLAN) stabil.
- Host stellt nur **Treiber + Devices + Docker** bereit.
- ROS läuft **im Repo-Compose** (`amr-platform/docker`):
  - `amr_agent` (micro-ROS Agent) verbindet über **/dev/ttyACM0 @ 921600**
  - `amr_dev` (ROS 2 Humble Workspace) sieht **/dev/ttyUSB0** (RPLidar)

### DoD (prüfbar)

- [ ] `ssh pi@rover` funktioniert.
- [ ] `docker compose up -d` im Repo startet ohne manuelle Nacharbeit.
- [ ] `docker compose logs microros_agent --tail 5` zeigt `running... | fd: 3`.
- [ ] In `amr_dev`: `ros2 topic list` zeigt mind. `/cmd_vel`, `/odom_raw`, `/esp32/heartbeat`, `/scan` (wenn LiDAR-Node läuft).

---

## 1) Host-OS Auswahl (Projektstandard)

**Projektstandard:** Raspberry Pi OS Lite (64-bit) als Host, ROS 2 läuft in Docker.

- **Host-Aufgaben:** WLAN/SSH, USB/PCIe Devices, Docker Engine/Compose, udev.
- **Container-Aufgaben:** ROS 2 Humble, Packages, Build, Launch, Tests.

> Hinweis: Kamera (IMX296) und Hailo-8L sind optional/später; der aktuelle AMR-Stand (Phase 1–3) braucht sie nicht zwingend.

---

## 2) Image schreiben (Headless)

### Raspberry Pi Imager

1. Device: **Raspberry Pi 5**
2. OS: **Raspberry Pi OS Lite (64-bit)**
3. Settings (OS Customisation):
   - Hostname: `rover`
   - User: `pi`
   - WLAN: SSID/Passwort, Land `DE`
   - SSH: **Enable SSH**

### Erster Boot

- SD einstecken, booten, 2–3 Minuten warten.
- SSH:

```bash
ssh pi@rover
# später
ssh rover
```

---

## 3) Basis-Härtung & Update

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt autoremove -y
```

**Empfohlen (SSH-Key statt Passwort):**

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
# public key vom Dev-PC nach ~/.ssh/authorized_keys kopieren
chmod 600 ~/.ssh/authorized_keys
```

---

## 4) Device-Zugriff (Seriell/I2C/Video)

```bash
sudo usermod -aG dialout $USER   # /dev/ttyACM*, /dev/ttyUSB*
sudo usermod -aG video   $USER   # Kamera (optional)
sudo usermod -aG i2c     $USER   # IMU am Pi (optional)
```

Danach einmal neu einloggen (oder reboot).

### Ports prüfen (Projektstandard)

```bash
ls -l /dev/ttyACM*   # Erwartung: /dev/ttyACM0 (ESP32-S3)
ls -l /dev/ttyUSB*   # Erwartung: /dev/ttyUSB0 (RPLidar A1)
```

---

## 5) USB-Leistungsbudget (Pi 5)

Raspberry Pi 5 kann – abhängig vom Netzteil/USB-PD – USB-Ports begrenzen. Mit dem offiziellen 27W-Netzteil sind höhere USB-Ströme möglich; bei schwächeren Netzteilen kann das System in einen konservativen Modus fallen (z. B. ~600 mA Limit).

---

## 6) Docker installieren (Host)

### Docker Engine

```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
exit
```

Neu einloggen, dann:

```bash
docker run hello-world
```

### Docker Compose Plugin

```bash
sudo apt install -y docker-compose-plugin
docker compose version
```

---

## 7) Repo-Setup & Start (aktueller Projektstand)

### Repo (Beispiel)

```bash
cd ~
git clone <DEIN-REPO-URL> amr-platform
cd ~/amr-platform/docker
docker compose up -d
```

### Agent prüfen (Phase 1/2 Standardtest)

```bash
docker compose logs microros_agent --tail 10
# Erwartung: "... running... | fd: 3"
```

### In ROS-Container gehen

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
```

---

## 8) Smoke-Checks (Phase 1–3)

### Topics sichtbar?

```bash
ros2 topic list
```

### Heartbeat (ESP32 verbunden)?

```bash
ros2 topic echo /esp32/heartbeat
# Erwartung: ~1 Hz Counter
```

### Odom kommt?

```bash
ros2 topic echo /odom_raw --once
```

### LiDAR Device da?

```bash
ls -l /dev/ttyUSB0
```

*(Der `/scan`-Topic ist erst da, wenn die LiDAR-Node läuft – siehe Phase-3-Doku.)*

---

## 9) Architektur (aktueller Stand)

```
┌─────────────────────────────────────────────────────────────┐
│ Raspberry Pi 5 (Raspberry Pi OS Lite 64-bit)                │
│  - Docker Engine + Compose                                  │
│  - Device-Passthrough: /dev/ttyACM0, /dev/ttyUSB0           │
└─────────────────────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ docker compose (amr-platform/docker)                         │
│  - amr_agent: micro-ROS Agent (serial /dev/ttyACM0 @ 921600) │
│  - amr_dev:   ROS 2 Humble Workspace                         │
│      - /cmd_vel, /odom_raw, /scan, ...                       │
└─────────────────────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────┐
│ ESP32-S3 (Firmware v3.2.0)                                   │
│  - micro-ROS Client (USB-CDC)                                │
│  - Motor-Control 100 Hz + Failsafe 2000 ms                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 11) Nächste Schritte (Phase 4)

- URDF + `robot_state_publisher`
- TF-Baum `odom → base_footprint → base_link → laser`
- `odom_converter.py` (/odom + TF) und optional EKF (robot_localization)
