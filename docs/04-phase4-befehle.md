# Phase 4: TF-Baum – Befehlsreferenz

**Status:** ✅ Abgeschlossen
**Version:** 2.3
**Datum:** 2025-12-21

---

## 1) Projektstruktur

```
ros2_ws/src/
├── amr_bridge/
│   ├── amr_bridge/
│   │   ├── __init__.py
│   │   └── odom_converter.py    # QoS: Best Effort
│   ├── package.xml
│   ├── resource/
│   │   └── amr_bridge
│   ├── setup.cfg
│   └── setup.py
├── amr_bringup/
│   ├── CMakeLists.txt
│   ├── launch/
│   │   └── amr.launch.py        # Default: /dev/rplidar
│   └── package.xml
├── amr_description/
│   ├── CMakeLists.txt
│   ├── launch/
│   │   └── description.launch.py
│   ├── package.xml
│   └── urdf/
│       └── amr.urdf
└── sllidar_ros2/

docker/
├── Dockerfile                   # ROS auto-source in .bashrc
├── docker-compose.yml           # /dev:/dev Volume Mount
└── ros_entrypoint.sh

scripts/
├── 99-amr-usb.rules             # udev-Regeln
└── install_udev_rules.sh
```

---

## 2) Einmalige Installation (nur beim ersten Mal)

### 2.1 udev-Regeln installieren (Pi)

```bash
cd ~/amr-platform/scripts
chmod +x install_udev_rules.sh
sudo ./install_udev_rules.sh

# USB-Geraete abstecken/anstecken, dann pruefen:
ls -la /dev/rplidar /dev/esp32
```

### 2.2 Docker-Image bauen (Pi)

```bash
cd ~/amr-platform/docker
docker compose build --no-cache
```

### 2.3 ROS-Packages bauen (einmalig im Container)

```bash
docker compose up -d
docker compose exec amr_dev bash

cd /root/ros2_ws
colcon build --packages-select amr_bridge amr_description amr_bringup
```

---

## 3) Normaler Start (nach Reboot)

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose exec amr_dev bash

# Starten - kein serial_port mehr noetig!
ros2 launch amr_bringup amr.launch.py
```

---

## 4) Smoke-Tests

**Zweites Terminal:**

```bash
docker compose exec amr_dev bash

# Topics pruefen
ros2 topic list | grep -E "odom|scan|tf"

# TF-Baum pruefen
ros2 run tf2_ros tf2_echo odom laser

# TF-Graph erzeugen
ros2 run tf2_tools view_frames
```

---

## 5) Erwartete Topics

```
/cmd_vel           # Teleop-Befehle
/esp32/heartbeat   # ESP32 Status
/odom_raw          # ESP32 (Pose2D)
/odom              # odom_converter (Odometry)
/scan              # RPLidar
/tf                # Dynamische TFs
/tf_static         # Statische TFs (URDF)
/robot_description # URDF
```

---

## 6) TF-Baum

```
odom (dynamisch: odom_converter, ~8 Hz)
  └── base_footprint
        └── base_link (statisch: URDF)
              └── laser (statisch: URDF)
```

---

## 7) Troubleshooting

| Problem | Ursache | Loesung |
|---------|---------|---------|
| `ros2: command not found` | ROS nicht gesourced | Dockerfile neu bauen oder `source /opt/ros/humble/setup.bash` |
| `/dev/rplidar` fehlt | udev-Regel nicht aktiv | USB abstecken/anstecken oder `sudo udevadm trigger` |
| RPLidar Timeout | Port belegt oder Kabel | USB neu einstecken, Container neu starten |
| QoS Warning odom_raw | Alte odom_converter Version | `colcon build --packages-select amr_bridge` |
| TF odom fehlt | ESP32 sendet nicht | `ros2 topic echo /odom_raw --once` pruefen |
| Container sieht USB nicht | Falscher Mount | docker-compose.yml: `volumes: - /dev:/dev` |

---

## 8) USB-Geraete

| Geraet | Symlink | Physisch | Verwendung |
|--------|---------|----------|------------|
| RPLidar A1 | `/dev/rplidar` | ttyUSB* | LaserScan |
| ESP32-S3 | `/dev/esp32` | ttyACM0 | micro-ROS |

Die Symlinks bleiben stabil, auch wenn sich ttyUSB0/1/2 aendert.

---

## 9) URDF-Masse anpassen

Falls der Laser in RViz2 falsch positioniert ist:

```bash
# Auf Mac editieren
nano ros2_ws/src/amr_description/urdf/amr.urdf
```

```xml
<!-- base_link Hoehe ueber Boden -->
<origin xyz="0 0 0.05" rpy="0 0 0"/>

<!-- Laser Position relativ zu base_link -->
<origin xyz="0.08 0 0.08" rpy="0 0 0"/>
```

Nach Aenderung: Git push, Pi pull, `colcon build`.

---

## 10) Definition of Done (DoD)

- [x] `ros2 topic list` zeigt /odom, /tf, /tf_static, /scan
- [x] `ros2 run tf2_ros tf2_echo odom laser` liefert kontinuierliche Werte
- [x] `ros2 run tf2_tools view_frames` erzeugt zusammenhaengenden Baum
- [x] udev-Regeln fuer stabile USB-Ports
- [x] QoS Best Effort fuer micro-ROS Kompatibilitaet

---

## 11) Naechste Phase

**Phase 5: SLAM** - Kartenerstellung mit slam_toolbox

Voraussetzungen erfuellt:

- /scan im Frame `laser` ✓
- TF-Baum `odom -> base_footprint -> base_link -> laser` ✓
- Odometrie auf /odom ✓
