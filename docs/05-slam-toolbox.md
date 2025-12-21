---
title: "Phase 5 – SLAM Toolbox: Mapping (/map) + TF map→odom"
type: "phase-doc"
goal: "Online-Mapping etablieren: SLAM Toolbox erzeugt eine nutzbare Occupancy-Map (/map) und die TF-Kante map→odom, sodass Phase 6 (Nav2) auf einer gespeicherten Karte oder im SLAM-Modus arbeiten kann."
status: "active"
updated: "2025-12-21"
version: "1.1"
depends_on:
  - "Phase 2 (Docker ROS 2 Humble) ✅"
  - "Phase 3 (RPLidar A1 + /scan) ✅"
  - "Phase 4 (URDF + TF + /odom) ⬜/planned"
next:
  - "Phase 6 (Nav2 Navigation auf Map)"
---

# Phase 5 – SLAM Toolbox (Mapping)

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Diese Phasen-Doku ist ein **Phasen-Nachweis**: Sie beschreibt für Phase 5 die **Lieferobjekte** (Map + map→odom), die **prüfbaren Abnahmekriterien (DoD)** und einen **kurzen Verifikationspfad** (Topics/TF/RViz/Map-Save), damit Phase 6 darauf aufsetzen kann.

### Regel (Scope-Grenze)

- Zielbild, DoD, minimaler TF-/Topic-Vertrag, Smoke-Checks, Map-Save, typische Fehlerbilder.

---

## 1) Zielbild (Soll-Zustand)

- SLAM Toolbox läuft im ROS-Container auf dem Pi 5.
- Eingänge sind stabil:
  - `/scan` (LaserScan)
  - `/odom` (Odometry) **oder** `/odometry/filtered` (falls EKF genutzt wird; ggf. remap)
  - TF `odom → base_footprint` + statische TFs bis `laser`
- SLAM Toolbox liefert:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - TF `map → odom`
- Die Map kann als `<name>.yaml` + `<name>.pgm` gespeichert werden.

---

## 2) Definition of Done (DoD) – prüfbar

- [ ] `ros2 topic list` enthält `/scan` und `/map`
- [ ] `ros2 run tf2_ros tf2_echo map odom` liefert fortlaufend gültige Transform
- [ ] RViz2 (Fixed Frame `map`): Map sichtbar, Scan liegt plausibel in der Karte, Pose/Bewegung konsistent
- [ ] Map-Save erzeugt Dateien: `<name>.yaml` und `<name>.pgm`

---

## 3) TF-/Topic-Vertrag (muss vorher stimmen)

### 3.1 Topics

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`) **oder** `/odometry/filtered` (remap auf `/odom`, wenn nötig)

### 3.2 TF (minimal)

```
odom → base_footprint → base_link → laser
map → odom            (von SLAM Toolbox)
```

**Regel:** Pro TF-Kante genau **eine** Quelle (kein Doppel-Publishing von `odom → base_*`).

---

## 4) Artefakte (Repo-Orientierung)

Empfohlen ab Phase 5:

```
ros2_ws/src/amr_bringup/
├─ config/
│  └─ slam_toolbox_online_async.yaml
└─ launch/
└─ slam_toolbox.launch.py
```

Ziel: Start ist **reproduzierbar** über ein eigenes Bringup-Launchfile.

---

## 5) Start (Online Mapping)

### 5.1 Minimalstart (ohne eigenes Bringup, nur zum ersten Nachweis)

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 5.2 Minimal-Parameter (für späteres Repo-Config)

- `scan_topic: /scan`
- `map_frame: map`
- `odom_frame: odom`
- `base_frame: base_footprint` (oder konsistent `base_link`)

---

## 6) Smoke-Checks (kurz, zielorientiert)

### 6.1 Topics vorhanden?

```bash
ros2 topic list | egrep "/scan|/map|/tf"
ros2 topic hz /scan
ros2 topic hz /map
```

### 6.2 TF-Kette vollständig?

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

### 6.3 RViz2 Minimal-Check

- Fixed Frame: `map`
- Displays: **Map** (`/map`), **LaserScan** (`/scan`), **TF**

Erwartung: Scan liegt plausibel auf/gegen die Map, keine „schwimmende“ Wolke.

---

## 7) Map speichern (Übergabe an Phase 6)

### 7.1 Speicherziel (Docker-Volume empfohlen)

- Host: `amr-platform/maps/`
- Container: `/maps`

Compose-Idee:

```yaml
volumes:
  - ../maps:/maps:rw
```

### 7.2 Save

```bash
ros2 run nav2_map_server map_saver_cli -f /maps/amr_map
```

**Erwartete Dateien:**

- `/maps/amr_map.yaml`
- `/maps/amr_map.pgm`

---

## 8) Typische Fehlerbilder (schnelle Eingrenzung)

### 8.1 `/map` bleibt leer

- `/scan` fehlt oder falsches `scan_topic`
- TF `odom → base_*` fehlt/instabil
- `frame_id` von `/scan` passt nicht zum TF-Baum

Checks:

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
```

### 8.2 Map driftet / Scan „klebt“ am Roboter

- `map → odom` fehlt oder wird überschrieben
- Doppel-Publishing `odom → base_*`

Check:

```bash
ros2 run tf2_tools view_frames
ros2 topic info /tf
```

### 8.3 Map verzerrt

- Wheel-Odom Skalierung unkalibriert (Ticks/m)
- LiDAR-Yaw im URDF falsch
- Mapping zu schnell (anfangs langsam fahren)

---

## 9) Übergabe an Phase 6 (Nav2)

Phase 6 kann beginnen, wenn:

- `/map` + `map → odom` stabil sind **und**
- Map-Dateien gespeichert wurden **oder** SLAM im Live-Modus als Map-Quelle läuft.

---

## 10) Changelog (phase-relevant)

| Version | Datum      | Änderung                                                                             |
| ------- | ---------- | ------------------------------------------------------------------------------------ |
| v1.1    | 2025-12-21 | Abhängigkeiten auf Humble/Phase-4-Definition konsolidiert, Fokus: DoD + Verifikation |
| v1.0    | 2025-12-19 | Initial: Online-Async Mapping, Smoke-Checks, Map-Save, TF-Vertrag                    |
