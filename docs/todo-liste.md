# ToDo-Liste AMR-Projekt

> **Stand:** 2025-12-20 | **Aktuelle Phase:** 4 (URDF/TF/EKF)

---

## Phasen-Übersicht

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 1 | micro-ROS auf ESP32-S3 | ✅ Abgeschlossen |
| Phase 2 | Docker-Infrastruktur | ✅ Abgeschlossen |
| Phase 3 | RPLidar A1 Integration | ✅ Abgeschlossen |
| Phase 4 | URDF + TF + EKF | ◄── **NÄCHSTE** |
| Phase 5 | SLAM (slam_toolbox) | ⬜ |
| Phase 6 | Nav2 Autonome Navigation | ⬜ |

---

## ✅ Phase 1: micro-ROS ESP32-S3 – ABGESCHLOSSEN

### Firmware v3.2.0 (2025-12-20)

- [x] micro-ROS Client über USB-CDC (Serial)
- [x] Dual-Core FreeRTOS (Core 0: Control, Core 1: Comms)
- [x] `/cmd_vel` → Motorsteuerung
- [x] `/odom_raw` → Odometrie (Pose2D)
- [x] `/esp32/heartbeat` → Lebenszeichen (1 Hz)
- [x] Failsafe (2000ms Timeout)
- [x] Feedforward-Steuerung (Gain=2.0)

---

## ✅ Phase 2: Docker-Infrastruktur – ABGESCHLOSSEN

- [x] `amr_agent` Container (micro-ROS Agent)
- [x] `amr_dev` Container (ROS 2 Humble Workspace)
- [x] `docker compose up -d` funktioniert
- [x] Volumes für ros2_ws gemountet

---

## ✅ Phase 3: RPLidar A1 – ABGESCHLOSSEN

### Verifiziert (2025-12-20)

- [x] sllidar_ros2 gebaut
- [x] `/scan` publiziert (~7.6 Hz)
- [x] Scan-Daten plausibel (0.05-12m Range)
- [x] frame_id: `laser`
- [x] docker-compose.yml aktualisiert

### Hardware-Info

| Parameter | Wert |
|-----------|------|
| S/N | 74A5FA89C7E19EC8BCE499F0FF725670 |
| Firmware | 1.29 |
| Scan Mode | Sensitivity |
| Sample Rate | 8 kHz |
| Frequenz | ~7.6 Hz |

---

## 🔜 Phase 4: URDF + TF + EKF – NÄCHSTE

- [ ] URDF erstellen
  - [ ] `base_footprint` (Boden)
  - [ ] `base_link` (Chassis)
  - [ ] `laser` Frame
- [ ] `robot_state_publisher` für statische TFs
- [ ] `odom_converter.py` Bridge Node
  - [ ] `/odom_raw` → `/odom`
  - [ ] TF: `odom → base_footprint`
- [ ] Optional: EKF (robot_localization)

### TF-Baum Ziel

```
odom → base_footprint → base_link → laser
```

---

## ⬜ Phase 5: SLAM

- [ ] `slam_toolbox` installieren
- [ ] Online Async SLAM
- [ ] Testraum kartieren
- [ ] Karte speichern

---

## ⬜ Phase 6: Nav2

- [ ] Nav2 Stack
- [ ] AMCL Lokalisierung
- [ ] Costmap
- [ ] Autonome Navigation

---

## Aktuelle Hardware Ports

| Device | Port | Funktion |
|--------|------|----------|
| ESP32-S3 | `/dev/ttyACM0` | micro-ROS (921600 Baud) |
| RPLidar A1 | `/dev/ttyUSB0` | LaserScan |

---

## Quick Reference

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
```

### RPLidar starten

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### Topics prüfen

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```
