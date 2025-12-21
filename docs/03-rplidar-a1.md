---
title: "Phase 3 – RPLidar A1 Integration + /scan"
type: "phase-doc"
goal: "2D-Laserscan im ROS-2-System bereitstellen (stabiler /scan-Stream mit plausiblen Messwerten und definierter frame_id) als Grundlage für URDF/TF/SLAM/Nav2."
status: "completed"
updated: "2025-12-20"
version: "3.0"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker ROS 2 Humble) ✅"
next:
  - "Phase 4 (URDF + TF + EKF)"
---

# Phase 3 – RPLidar A1 Integration (+ /scan)

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Die Phasen-Doku ist ein **Phasen-Nachweis**: Sie hält fest, dass Phase 3 einen **stabilen LaserScan-Datenstrom** liefert, inklusive **Mess-/Verifikationswerten** und den **Artefakten/Parametern**, die den Stand reproduzierbar machen.

### Regel (Scope-Grenze)

- Zielbild, DoD, Messwerte, Startpunkt (kurzer Smoke-Test), relevante Compose-/Device-Integration, bekannte Grenzen.

---

## 1) Zielbild (Soll-Zustand)

- RPLidar A1 läuft auf dem Raspberry Pi 5 (Docker/ROS 2 Humble) als ROS-Node.
- Topic `/scan` (`sensor_msgs/msg/LaserScan`) ist stabil und liefert plausible Messwerte.
- `header.frame_id` ist **`laser`** (TF-Anbindung folgt in Phase 4).

---

## 2) Definition of Done (DoD) – verifiziert am 2025-12-20

- [x] `/scan` publiziert stabil bei ~\(7{,}6\,\mathrm{Hz}\) (`ros2 topic hz /scan`)
- [x] Einzelmessage ist plausibel (`ros2 topic echo /scan --once`)
- [x] `header.frame_id = laser`
- [x] Wertebereich plausibel:
  - Winkelbereich ~\(-\pi\) bis \(+\pi\) rad
  - `range_min ≈ 0.05 m`, `range_max ≈ 12.0 m`
  - `ranges[]` enthält plausible Distanzen (inkl. `inf` bei keiner Reflexion)

---

## 3) Artefakte (Repo-/Setup-Wahrheit)

- Docker Compose Device-Passthrough im `amr_dev` Container (kein separater LiDAR-Container)
- ROS 2 Workspace: `ros2_ws/src/` enthält `sllidar_ros2` (Treiberpaket)

---

## 4) Hardware-Stand (verifiziert)

| Parameter | Wert |
|-----------|------|
| Modell | RPLidar A1 |
| S/N | 74A5FA89C7E19EC8BCE499F0FF725670 |
| Firmware | 1.29 |
| Hardware Rev | 7 |
| Scan Mode | Sensitivity |
| Sample Rate | \(8\,\mathrm{kHz}\) |
| Host-Port | `/dev/ttyUSB0` |
| USB-Bridge | cp210x |

---

## 5) Verifikation (Messwerte vom 2025-12-20)

| Feld | Erwartung/Beobachtung |
|------|------------------------|
| Frequenz | ~\(7{,}6\,\mathrm{Hz}\) |
| `header.frame_id` | `laser` |
| `angle_min` | \(\approx -3.14\,\mathrm{rad}\) |
| `angle_max` | \(\approx +3.14\,\mathrm{rad}\) |
| `range_min` | \(\approx 0.05\,\mathrm{m}\) |
| `range_max` | \(\approx 12.0\,\mathrm{m}\) |

---

## 6) Container-Integration (entscheidende Regel aus Phase 3)

**Regel:** RPLidar läuft im `amr_dev` Container, kein separater `rplidar` Container.
**Begründung:** vermeidet serielle Port-Konflikte auf `/dev/ttyUSB0`.

**Erforderlich:** Device-Mount in `amr_dev`:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

---

## 7) Smoke-Test (kurz, reproduzierbar)

1. `/scan` sichtbar:

```bash
ros2 topic list
```

1. Frequenz prüfen:

```bash
ros2 topic hz /scan
```

1. Plausibilität prüfen:

```bash
ros2 topic echo /scan --once
```

---

## 8) Übergangslösung (bis Phase 4)

Wenn RViz/TF kurzfristig benötigt wird, kann ein **temporärer** statischer TF gesetzt werden:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.12 0.0 0.15 0 0 0 base_link laser
```

*(In Phase 4 wird das durch URDF + `robot_state_publisher` abgelöst.)*

---

## 9) Bekannte Einschränkungen (für Phase 4 relevant)

- `laser` ist noch nicht per URDF/TF sauber an `base_link` angebunden.
- Serial-Port ist exklusiv: parallele Prozesse/Container blockieren `/dev/ttyUSB0`.

---

## 10) Übergabe an Phase 4 (Was ist jetzt „bereit“?)

- `/scan` ist verfügbar, stabil und plausibel.
- `frame_id` ist festgelegt (`laser`).
- Host-Container sieht den Sensor über `/dev/ttyUSB0`.

---

## 11) Changelog (phase-relevant)

| Version | Datum      | Änderung                                                                    |
| ------- | ---------- | --------------------------------------------------------------------------- |
| v3.0    | 2025-12-20 | Status completed, Device im `amr_dev`, verifizierte Messwerte/Hardwaredaten |
| v2.0    | 2025-12-20 | ROS 2 Humble konsolidiert                                                   |
| v1.0    | 2025-12-19 | Initial                                                                     |
