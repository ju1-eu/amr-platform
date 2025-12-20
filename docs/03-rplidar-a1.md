---
title: "Phase 3 – RPLidar A1 Integration + /scan"
status: "completed"
updated: "2025-12-20"
version: "3.0"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker ROS 2 Humble) ✅"
next:
  - "Phase 4 (URDF + TF + EKF)"
---

# Phase 3: RPLidar A1 Integration (+ /scan)

## Zielbild & Definition of Done

### Zielbild

- RPLidar A1 läuft am Raspberry Pi 5 (Docker/ROS 2 Humble) als Laser-Treiber.
- Topic `/scan` (`sensor_msgs/msg/LaserScan`) ist stabil und hat plausible Werte.
- Frame-ID des Scans ist **`laser`** (später in Phase 4 per TF mit `base_link` verbunden).

### DoD (verifiziert 2025-12-20)

- [x] `ros2 topic hz /scan` zeigt stabile Frequenz (~7.6 Hz)
- [x] `ros2 topic echo /scan --once` liefert plausible Werte
- [x] frame_id: `laser`
- [x] Scan-Daten: -π bis +π, 0.05-12m Range

---

## 1) Hardware-Info (verifiziert)

### 1.1 Sensor

| Parameter | Wert |
|-----------|------|
| Modell | RPLidar A1 |
| S/N | 74A5FA89C7E19EC8BCE499F0FF725670 |
| Firmware | 1.29 |
| Hardware Rev | 7 |
| Scan Mode | Sensitivity |
| Sample Rate | 8 kHz |

### 1.2 Gemessene Werte

| Parameter | Wert |
|-----------|------|
| Frequenz | ~7.6 Hz |
| range_min | 0.05 m |
| range_max | 12.0 m |
| angle_min | -3.14 rad |
| angle_max | +3.14 rad |

### 1.3 Anschluss

| Device | Port |
|--------|------|
| RPLidar A1 | `/dev/ttyUSB0` |
| USB-Adapter | cp210x |

---

## 2) Docker-Integration

### 2.1 docker-compose.yml

**Wichtig:** Kein separater `rplidar` Container! Dies verursacht Port-Konflikte. RPLidar wird im `amr_dev` Container gestartet.

```yaml
services:
  # micro-ROS Agent für ESP32-S3
  microros_agent:
    image: microros/micro-ros-agent:humble
    container_name: amr_agent
    network_mode: host
    privileged: true
    restart: always
    command: serial --dev /dev/ttyACM0 -b 921600
    devices:
      - /dev/ttyACM0:/dev/ttyACM0

  # ROS 2 Humble Workspace (inkl. RPLidar)
  amr_dev:
    build: .
    container_name: amr_base
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
    volumes:
      - ../ros2_ws:/root/ros2_ws
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    command: tail -f /dev/null
```

---

## 3) Installation

### 3.1 sllidar_ros2 klonen (im Container)

```bash
cd ~/amr-platform/docker
docker compose exec amr_dev bash

cd /root/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### 3.2 Bauen

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

---

## 4) RPLidar starten

### 4.1 Terminal 1: Lidar-Node

```bash
cd ~/amr-platform/docker
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

**Erwartete Ausgabe:**

```
[sllidar_node-1] [INFO] SLLidar S/N: 74A5FA89C7E19EC8BCE499F0FF725670
[sllidar_node-1] [INFO] Firmware Ver: 1.29
[sllidar_node-1] [INFO] Hardware Rev: 7
[sllidar_node-1] [INFO] SLLidar health status : OK.
[sllidar_node-1] [INFO] current scan mode: Sensitivity, sample rate: 8 Khz, max_distance: 12.0 m
```

### 4.2 Terminal 2: Smoke-Tests

```bash
ssh pi@rover
cd ~/amr-platform/docker
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```

---

## 5) Smoke-Tests

### 5.1 Topics prüfen

```bash
ros2 topic list
```

**Erwartung:**

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/odom_raw
/scan
/parameter_events
/rosout
```

### 5.2 Frequenz prüfen

```bash
ros2 topic hz /scan
```

**Erwartung:** ~7.6 Hz

### 5.3 Daten prüfen

```bash
ros2 topic echo /scan --once
```

**Prüfpunkte:**

| Feld | Erwartung |
|------|-----------|
| `header.frame_id` | `laser` |
| `angle_min` | -3.14 |
| `angle_max` | +3.14 |
| `range_min` | 0.05 |
| `range_max` | 12.0 |
| `ranges[]` | Werte 0.05-12.0, `inf` bei keiner Reflexion |

---

## 6) Troubleshooting

### 6.1 "Operation timeout"

**Ursache:** Port von anderem Prozess blockiert.

```bash
# Auf Pi Host prüfen
sudo fuser /dev/ttyUSB0

# Falls belegt, Prozesse beenden
sudo kill <PID>
```

### 6.2 `/scan` nicht vorhanden

**Ursache:** Lidar-Node läuft nicht.

- Lidar-Node muss in Terminal 1 **weiterlaufen**
- Mit `Ctrl+C` gestoppt = kein `/scan`

### 6.3 Device nicht vorhanden

```bash
ls -l /dev/ttyUSB*
dmesg | grep -i usb | tail -10
```

### 6.4 Container sieht Device nicht

docker-compose.yml prüfen:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

### 6.5 Baudrate falsch

Standard ist 115200. Falls Timeout, versuche:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=256000
```

---

## 7) Statischer TF (temporär für RViz2)

Bis Phase 4 (URDF/EKF) fertig ist:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.12 0.0 0.15 0 0 0 base_link laser
```

---

## 8) Nächste Schritte (Phase 4)

1. URDF erstellen mit `base_link → laser` Transform
2. `robot_state_publisher` für statische TFs
3. `odom_converter.py` Bridge Node
4. Optional: EKF (robot_localization)

---

## 9) Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v3.0 | 2025-12-20 | Status: completed, kein separater rplidar Container, verifizierte Hardware-Info, Troubleshooting erweitert |
| v2.0 | 2025-12-20 | Humble statt Jazzy |
| v1.0 | 2025-12-19 | Initiale Version |
