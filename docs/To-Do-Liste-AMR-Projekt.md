# To-Do-Liste AMR-Projekt

> **Stand:** 2025-12-20
> **Hinweis (To-Do-Regel):** Dieses Dokument ist eine **Aufgabenliste** (Was fehlt noch?).

---

## Projektstatus (kurz)

| Phase | Beschreibung | Status |
|------|--------------|------|
| 1 | micro-ROS auf ESP32-S3 | ✅ |
| 2 | Docker-Infrastruktur | ✅ |
| 3 | RPLidar A1 Integration | ✅ |
| 4 | URDF + TF + EKF | ◄── **als Nächstes** |
| 5 | SLAM (slam_toolbox) | ⬜ |
| 6 | Nav2 Autonome Navigation | ⬜ |

---

## To-Dos (nach Phasen gruppiert)

### Phase 4 — URDF + TF + EKF (als Nächstes)

- [ ] URDF-Grundgerüst anlegen
  - [ ] `base_footprint`
  - [ ] `base_link`
  - [ ] `laser` (oder `laser_frame`, Projektstandard festlegen)
- [ ] `robot_state_publisher` einbinden (statische TFs aus URDF)
- [ ] `odom_converter.py` hinzufügen (Bridge)
  - [ ] `/odom_raw` → `/odom`
  - [ ] TF publizieren: `odom -> base_footprint`
- [ ] EKF (robot_localization) einführen (optional/anschließend)
  - [ ] EKF-Config-Datei anlegen
  - [ ] Inputs festlegen (z. B. Odom, optional IMU)
  - [ ] Output-Frames prüfen

**Ziel-Frames (Projektziel):**

```
odom -> base_footprint -> base_link -> laser
```

---

### Phase 5 — SLAM

- [ ] `slam_toolbox` installieren/integrieren
- [ ] Online SLAM (async) starten
- [ ] Testraum abfahren und Karte erzeugen
- [ ] Karte speichern (map file)

---

### Phase 6 — Nav2

- [ ] Nav2-Stack integrieren
- [ ] Lokalisierung (z. B. AMCL) einrichten
- [ ] Costmaps konfigurieren
- [ ] Autonome Navigation testen (Goal setzen, fahren)

---

## Hardware/Ports (Ist-Zustand)

| Device | Port | Zweck |
|--------|------|-------|
| ESP32-S3 | `/dev/ttyACM0` | micro-ROS (921600 Baud) |
| RPLidar A1 | `/dev/ttyUSB0` | LaserScan |

---

## Quick Reference (Arbeitsbefehle)

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
````

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
