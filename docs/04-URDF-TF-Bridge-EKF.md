---
title: "Phase 4 – URDF + TF Frames + EKF Sensor Fusion"
type: "phase-doc"
goal: "Konsistenten TF-Baum und ein ROS-konformes Odometrie-Interface (/odom + TF) herstellen, damit LaserScan korrekt relativ zum Roboter darstellbar ist und Phase 5 (SLAM) ohne TF-Probleme starten kann."
status: "planned"
updated: "2025-12-20"
version: "2.1"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker-Infrastruktur) ✅"
  - "Phase 3 (RPLidar A1) ✅"
next:
  - "Phase 5 (SLAM)"
---

# Phase 4 – URDF + TF Frames + EKF Sensor Fusion

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Die Phasen-Doku ist ein **Phasen-Nachweis**: Sie definiert für Phase 4 die **konkreten Lieferobjekte** (URDF/TF/Bridge/EKF), die **prüfbaren Abnahmekriterien (DoD)** und die **Mess-/Verifikationsschritte**, damit Phase 5 (SLAM) auf einer stabilen Transformationskette aufsetzt.

### Regel (Scope-Grenze)

- Zielbild, DoD, Abhängigkeiten, Artefakte, Messwerte/Offsets (soweit bekannt), Smoke-Checks, bekannte Grenzen.

---

## 1) Zielbild (TF-Baum)

**Minimalziel für SLAM-Start:**

```
odom → base_footprint → base_link → laser
```

**Erweiterung (später, SLAM/AMCL):**

```
map → odom → base_footprint → base_link → laser
```

---

## 2) Definition of Done (DoD) – prüfbar

### 2.1 Pflicht (für Phase 5)

- [ ] `robot_state_publisher` publiziert statische TFs aus URDF (`/tf_static`)
- [ ] `odom_converter` publiziert `/odom` (`nav_msgs/Odometry`)
- [ ] `odom_converter` publiziert dynamische TF `odom → base_footprint`
- [ ] `ros2 run tf2_tools view_frames` erzeugt einen **konsistenten**, zusammenhängenden Baum (ohne Loops)
- [ ] `ros2 run tf2_ros tf2_echo odom laser` liefert kontinuierliche Werte
- [ ] RViz2 zeigt **RobotModel + LaserScan** korrekt relativ zueinander

### 2.2 Optional (wenn IMU vorhanden / Nutzen gegeben)

- [ ] EKF (`robot_localization`) erzeugt `/odometry/filtered` und (optional) TF `odom → base_footprint`
- [ ] Es existiert **genau eine** TF-Quelle für `odom → base_footprint` (kein Double-Publish)

---

## 3) Eingänge aus Phase 1–3 (Ist-Schnittstellen)

| Signal | Typ | Quelle |
|--------|-----|--------|
| `/odom_raw` | `geometry_msgs/Pose2D` | ESP32 (micro-ROS) |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar A1 |
| `/cmd_vel` | `geometry_msgs/Twist` | Host/Teleop |

---

## 4) Konventionen (ROS-De-facto: REP-103/105)

- Achsen: \(x\) vorwärts, \(y\) links, \(z\) oben
- Rollen:
  - URDF / `robot_state_publisher`: **statische** Frames (Link-Kette)
  - `odom_converter` oder EKF: **dynamisch** `odom → base_footprint`
  - SLAM/AMCL (Phase 5/6): **dynamisch** `map → odom`

---

## 5) Artefakte (Repo-Zielstruktur)

```
ros2_ws/src/amr_description/
├─ urdf/amr.urdf(.xacro)
├─ launch/description.launch.py
└─ rviz/amr.rviz

ros2_ws/src/amr_bridge/
├─ amr_bridge/odom_converter.py
└─ setup.py / package.xml
```

---

## 6) Messungen/Offsets (TODO → Messwerte eintragen)

**Referenzdefinitionen:**

- `base_footprint`: Projektion auf Boden, \(z=0\), Mittelpunkt zwischen Radaufstandspunkten
- `base_link`: Chassis-Frame (z-Offset über `base_footprint`)
- `laser`: Ursprung am LiDAR-Optikzentrum

| Parameter | Wert | Einheit | Bedeutung |
|----------|------|--------:|----------|
| `base_link_z` | TBD | m | Höhe `base_link` über Boden |
| `laser_x` | TBD | m | LiDAR vor(+) / hinter(-) `base_link` |
| `laser_y` | 0.00 | m | LiDAR links/rechts |
| `laser_z` | TBD | m | LiDAR über `base_link` |
| `laser_yaw` | 0 oder \(\pi\) | rad | Ausrichtung |

---

## 7) Komponenten von Phase 4 (Lieferobjekte)

### 7.1 URDF + robot_state_publisher (statisch)

- liefert:
  - `base_footprint → base_link`
  - `base_link → laser`

### 7.2 Odom-Bridge (`odom_converter`) (dynamisch)

- Input: `/odom_raw (Pose2D)`
- Output:
  - `/odom (nav_msgs/Odometry)`
  - TF: `odom → base_footprint`

### 7.3 EKF (optional)

- Nutzen erst dann klar, wenn IMU oder zusätzliche Odom-Quellen vorliegen.
- Wenn EKF aktiv ist: `publish_tf`/TF-Quelle eindeutig halten (keine Doppelquelle).

---

## 8) Smoke-Checks (Abnahme-Protokoll)

### 8.1 Topics (Soll)

```
/odom_raw
/scan
/odom          # neu (Phase 4)
/tf
/tf_static
```

### 8.2 TF-Kette prüfen

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom laser
```

### 8.3 TF-Graph erzeugen

```bash
ros2 run tf2_tools view_frames
# Erwartung: frames.pdf mit zusammenhängendem Baum
```

### 8.4 RViz2 (funktionaler Check)

- Display: **RobotModel** (`/robot_description`)
- Display: **TF**
- Display: **LaserScan** (`/scan`, Frame: `laser`)
- Erwartung: Scan liegt konsistent am LiDAR-Frame und bewegt sich plausibel bei Rotation.

---

## 9) Troubleshooting (typische Fehlerbilder)

| Symptom                         | Ursache                             | Fix                      |
| ------------------------------- | ----------------------------------- | ------------------------ |
| TF „disconnected“               | `robot_state_publisher` läuft nicht | Launch prüfen            |
| `/odom` fehlt                   | `odom_converter` läuft nicht        | Node starten             |
| `odom → base_footprint` springt | 2 TF-Quellen aktiv                  | nur eine Quelle zulassen |
| Laser „steht falsch herum“      | `laser_yaw` falsch                  | URDF rpy korrigieren     |
| Laser „schwebt“                 | `laser_z`/`base_link_z` falsch      | Offsets messen/setzen    |

---

## 10) Übergabe an Phase 5 (SLAM)

Phase 4 gilt als „bereit“, wenn:

- `/scan` im Frame `laser` ankommt **und**
- `tf2_echo odom laser` stabil ist **und**
- RViz RobotModel + LaserScan deckungsgleich sind (keine Frame-Wildwüchse)

---

## 11) Changelog (phase-relevant)

| Version | Datum      | Änderung                                                           |
| ------- | ---------- | ------------------------------------------------------------------ |
| v2.1    | 2025-12-20 | Fokus auf Minimalziel für SLAM: URDF + TF + `/odom` (EKF optional) |
| v2.0    | 2025-12-20 | EKF als fester Bestandteil (überarbeitet)                          |
| v1.0    | 2025-12-19 | Initial                                                            |
