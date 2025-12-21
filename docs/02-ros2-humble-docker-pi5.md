---
title: "Phase 2 – ROS 2 Humble auf Pi 5 via Docker + micro-ROS Agent"
type: "phase-doc"
goal: "Reproduzierbare Host-Umgebung (Docker) bereitstellen, die micro-ROS Agent stabil betreibt und Phase 1 (ESP32) über USB-Serial integriert."
status: "completed"
updated: "2025-12-20"
version: "2.0"
depends_on:
  - "Phase 1 (micro-ROS auf ESP32-S3)"
next:
  - "Phase 3 (RPLidar A1)"
---

# Phase 2 – ROS 2 Humble auf Raspberry Pi 5 (Docker) + micro-ROS Agent

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Die Phasen-Doku ist ein **Phasen-Nachweis** (evidence-based):
Sie hält fest, **was** Phase 2 liefert, **wie** es verifiziert wurde, und welche **Artefakte/Parameter** im Repo den Stand reproduzierbar machen. Phase 3+ soll darauf ohne Rätselraten aufbauen.

### Regel (Scope-Grenze)

- Zielbild, DoD, Verifikationsprotokoll, Artefakte (docker-compose/Dockerfile), verifizierte Parameter, bekannte Grenzen.

---

## 1) Zielbild (Soll-Zustand)

- Raspberry Pi 5 (Raspberry Pi OS 64-bit) betreibt **ROS 2 Humble** in **Docker**.
- micro-ROS Agent läuft als Container und verbindet sich über **USB-Serial** mit dem ESP32-S3.
- Host-ROS kann:
  - `/cmd_vel` publizieren → Drivebase reagiert
  - `/odom_raw` empfangen → Werte kommen an
  - `/esp32/heartbeat` empfangen → Verbindung ist nachweisbar
- Setup ist nach Reboot reproduzierbar (`docker compose up -d` genügt).

---

## 2) Definition of Done (DoD) – verifiziert am 2025-12-20

- [x] `docker compose up -d` startet Container ohne manuelle Nacharbeit.
- [x] Agent verbindet stabil über `/dev/ttyACM0` mit `921600` Baud.
- [x] ROS Smoke-Checks sind grün:
  - [x] `ros2 topic list` zeigt `/cmd_vel`, `/odom_raw`, `/esp32/heartbeat`, `/esp32/led_cmd`
  - [x] `ros2 topic pub /cmd_vel ...` bewirkt Motorreaktion (Bench-Test)
  - [x] `ros2 topic echo /odom_raw` liefert Werte
- [x] Failsafe der Drivebase stoppt nach ~\(2\,\mathrm{s}\) bei ausbleibenden Kommandos (Phase-1-Funktion über Host verifiziert).

---

## 3) Artefakte (Repo-Wahrheit)

- Compose: `docker/docker-compose.yml`
- Dev-Image: `docker/Dockerfile`
- Workspace-Mount: `ros2_ws/` (Host-Volume in Container)

---

## 4) Container-Setup (verifizierter Stand)

| Container | Image | Aufgabe |
|-----------|-------|---------|
| `amr_agent` | `microros/micro-ros-agent:humble` | micro-ROS Agent (Serial) |
| `amr_dev` | Custom (ROS 2 Humble) | Tools + Workspace (colcon, rviz2, xacro, tf2-tools) |

**Verifizierte Betriebsparameter:**

- Device: `/dev/ttyACM0`
- Baudrate: `921600`
- Netzwerk: `network_mode: host`

---

## 5) Verifikation (Smoke-Checks vom 2025-12-20)

> Zweck: Nachweis, dass Phase 2 die Integration zu Phase 1 herstellt.

### 5.1 Verbindung (Agent)

- Erwartung: Agent läuft und meldet „running“ (fd stabil).

### 5.2 Topics sichtbar

- Erwartete Topics:

```
/cmd_vel
/odom_raw
/esp32/heartbeat
/esp32/led_cmd
```

### 5.3 Datenfluss

- `/esp32/heartbeat`: ~\(1\,\mathrm{Hz}\)
- `/odom_raw`: Werte kommen an (Format `Pose2D`)
- `/cmd_vel`: bewirkt Motorreaktion (Bench-Test)

---

## 6) Host-Voraussetzungen (minimal, für Reproduktion)

- Raspberry Pi 5
- Raspberry Pi OS 64-bit
- Docker Engine + Docker Compose Plugin
- USB-Gerät ESP32 als `/dev/ttyACM0` verfügbar

*(LiDAR `/dev/ttyUSB0` ist Thema von Phase 3; in Phase 2 nur „sichtbar“, aber nicht erforderlich.)*

---

## 7) Smoke-Test (kurz, reproduzierbar)

1) Container starten:

```bash
cd ~/amr-platform/docker
docker compose up -d
```

1. In Dev-Container:

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
```

1. Topics prüfen:

```bash
ros2 topic list
```

1. Heartbeat prüfen:

```bash
ros2 topic echo /esp32/heartbeat
```

1. Odom einmal:

```bash
ros2 topic echo /odom_raw --once
```

*(Motor-Test bleibt Bench-only und ist als Phase-1-Nachweis bereits geführt.)*

---

## 8) Bekannte Einschränkungen (für Phase 3+ relevant)

- Setup ist host-netzwerkbasiert (`network_mode: host`), um Domain-/Discovery-Probleme zu vermeiden.
- Der Agent ist an einen festen Device-Pfad gebunden (`/dev/ttyACM0`); udev-Regel/By-Id kann später stabilisieren.

---

## 9) Übergabe an Phase 3 (Was ist jetzt „bereit“?)

- Host-ROS Umgebung steht reproduzierbar (Docker).
- micro-ROS Agent verbindet zuverlässig zur Drivebase.
- ROS CLI kann Topics sehen, schreiben und lesen.
- Grundlage für LiDAR-Integration (Phase 3) ist vorhanden.

---

## 10) Changelog (phase-relevant)

| Version | Datum      | Änderung                                                                   |
| ------- | ---------- | -------------------------------------------------------------------------- |
| v2.0    | 2025-12-20 | ROS 2 Humble + Agent `humble`, 921600 Baud, Container/Compose konsolidiert |
| v1.0    | 2025-12-19 | Initial (Jazzy-Variante, überholt)                                         |
