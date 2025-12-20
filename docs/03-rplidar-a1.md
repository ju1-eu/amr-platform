---
title: "Phase 3 – RPLidar A1 Integration + /scan + RViz2"
status: "active"
updated: "2025-12-20"
version: "2.0"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker ROS 2 Humble) ✅"
next:
  - "Phase 4 (URDF + TF + EKF)"
---

# Phase 3: RPLidar A1 Integration (+ /scan + RViz2)

## Zielbild & Definition of Done

### Zielbild

- RPLidar A1 läuft am Raspberry Pi 5 (Docker/ROS 2 Humble) als Laser-Treiber.
- Topic `/scan` (`sensor_msgs/msg/LaserScan`) ist stabil und hat plausible Werte.
- Frame-ID des Scans ist **`laser`** (später in Phase 4 per TF mit `base_link` verbunden).

### DoD (prüfbar)

- [ ] `ros2 topic hz /scan` zeigt stabile Frequenz (~5-10 Hz)
- [ ] `ros2 topic echo /scan --once` liefert plausible Werte
- [ ] RViz2: LaserScan sichtbar (auf Pi oder remote)
- [ ] RPLidar startet automatisch mit Docker

---

## 1) Hardware-Status

### 1.1 Verifiziert

```bash
ls -l /dev/ttyUSB*
# Erwartung: /dev/ttyUSB0 (RPLidar A1)
```

### 1.2 Sensor-Eckdaten

| Parameter | Wert |
|-----------|------|
| Reichweite | 0.15 - 12 m |
| Scanrate | 5.5 - 10 Hz |
| Punkte/Scan | ~360-720 |
| Baudrate | 115200 |
| Interface | USB-Serial |

---

## 2) Docker-Integration

### 2.1 docker-compose.yml erweitern

```yaml
services:
  microros_agent:
    image: microros/micro-ros-agent:humble
    container_name: amr_agent
    network_mode: host
    privileged: true
    restart: always
    command: serial --dev /dev/ttyACM0 -b 921600
    devices:
      - /dev/ttyACM0:/dev/ttyACM0

  rplidar:
    image: ros:humble-ros-base
    container_name: amr_lidar
    network_mode: host
    privileged: true
    restart: always
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    volumes:
      - ../ros2_ws:/root/ros2_ws
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /root/ros2_ws/install/setup.bash 2>/dev/null || true &&
        ros2 launch sllidar_ros2 sllidar_a1_launch.py
      "

  amr_dev:
    build: .
    container_name: amr_base
    network_mode: host
    privileged: true
    volumes:
      - ../ros2_ws:/root/ros2_ws
    command: tail -f /dev/null
```

---

## 3) Treiber installieren

### 3.1 sllidar_ros2 klonen (auf Pi)

```bash
cd ~/amr-platform/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### 3.2 Im Container bauen

```bash
cd ~/amr-platform/docker
docker compose exec amr_dev bash

# Im Container
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --packages-select sllidar_ros2
source install/setup.bash
```

---

## 4) Manueller Test (vor Docker-Autostart)

### 4.1 RPLidar starten

```bash
# Im amr_dev Container
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### 4.2 In zweitem Terminal prüfen

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

# Topics prüfen
ros2 topic list | grep scan

# Frequenz prüfen
ros2 topic hz /scan

# Daten prüfen
ros2 topic echo /scan --once
```

---

## 5) Smoke-Tests

### 5.1 Topic vorhanden

```bash
ros2 topic list
```

**Erwartung:** `/scan` in der Liste

### 5.2 Frequenz stabil

```bash
ros2 topic hz /scan
```

**Erwartung:** ~5-10 Hz

### 5.3 Daten plausibel

```bash
ros2 topic echo /scan --once
```

**Prüfpunkte:**

| Feld | Erwartung |
|------|-----------|
| `header.frame_id` | `laser` |
| `angle_min` | ~ -3.14 |
| `angle_max` | ~ +3.14 |
| `range_min` | ~ 0.15 |
| `range_max` | ~ 12.0 |
| `ranges[]` | Werte zwischen 0.15-12.0, `inf` bei keiner Reflexion |

---

## 6) RViz2 (Remote oder auf Pi)

### 6.1 Auf Pi mit Display

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
rviz2
```

### 6.2 Remote (Mac/PC)

Voraussetzung: ROS 2 Humble auf Mac/PC installiert

```bash
# Auf Mac/PC
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
rviz2
```

### 6.3 RViz2 Konfiguration

1. **Fixed Frame:** `laser` (temporär, bis TF in Phase 4)
2. **Add → LaserScan**
   - Topic: `/scan`
   - Size: 0.05
3. **Erwartung:** Punkte erscheinen als Ring/Kreis

---

## 7) Statischer TF (temporär für RViz2)

Bis Phase 4 (URDF/EKF) fertig ist, kannst du einen statischen TF setzen:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.12 0.0 0.15 0 0 0 base_link laser
```

Dann in RViz2: Fixed Frame auf `base_link` setzen.

---

## 8) Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| `/scan` nicht vorhanden | Treiber läuft nicht | Launch prüfen, Logs checken |
| "Permission denied" | Device-Rechte | `privileged: true` in compose |
| Keine Daten (ranges leer) | Lidar blockiert | Freie Sicht prüfen |
| Frequenz < 1 Hz | USB-Bandbreite | Anderer USB-Port |
| RViz2 zeigt nichts | Frame falsch | Fixed Frame auf `laser` |

### 8.1 Logs prüfen

```bash
docker compose logs rplidar --tail 20
```

### 8.2 Device prüfen

```bash
# Auf Pi Host
ls -l /dev/ttyUSB0
dmesg | grep -i usb | tail -10
```

---

## 9) Alle Topics nach Phase 3

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

---

## 10) Nächste Schritte (Phase 4)

Nach erfolgreichem `/scan`:

1. URDF erstellen mit `base_link → laser` Transform
2. `robot_state_publisher` für statische TFs
3. `robot_localization` EKF für `odom → base_link`
4. Vollständiger TF-Baum für SLAM

---

## 11) Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v2.0 | 2025-12-20 | Humble statt Jazzy, Docker-Integration, aktuelle Container-Namen |
| v1.0 | 2025-12-19 | Initiale Jazzy-Version |
