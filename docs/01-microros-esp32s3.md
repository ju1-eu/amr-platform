---
title: "Phase 1 – micro-ROS auf ESP32-S3 (USB-Serial)"
type: "phase-doc"
goal: "Nachweis einer lauffähigen Drivebase-Grundfunktion mit ROS-2-Anbindung (micro-ROS) inkl. Mess-/Testprotokoll."
status: "completed"
updated: "2025-12-20"
version: "3.2.0"
source:
  firmware: "firmware/src/main.cpp"
  config: "firmware/include/config.h"
  platformio: "firmware/platformio.ini"
---

# Phase 1 – micro-ROS auf ESP32-S3 (USB-Serial)

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Die Phasen-Doku ist ein **Phasen-Nachweis** (evidence-based):
Sie beschreibt **Zielbild**, **Definition of Done**, **Mess-/Testergebnisse**, **Artefakte im Repo** und **bekannte Grenzen**, damit Phase 2+ zuverlässig darauf aufbauen kann.

### Regel (Scope-Grenze)

- Was wurde geliefert, wie wurde es verifiziert, welche Parameter/Artefakte gehören zur Phase, welche Einschränkungen bleiben offen.

---

## 1) Zielbild (Soll-Zustand)

- ESP32-S3 arbeitet als **micro-ROS Client** über **USB-CDC (Serial)**.
- ROS-Input `/cmd_vel` steuert die Motoren (Cytron MDD3A, Dual-PWM).
- ROS-Output `/odom_raw` wird als `geometry_msgs/Pose2D` publiziert und ist **plausibel**.
- **Failsafe** stoppt Motoren nach `FAILSAFE_TIMEOUT_MS = 2000`.

---

## 2) Definition of Done (DoD) – verifiziert am 2025-12-20

- [x] micro-ROS Agent verbindet stabil (Reconnect reproduzierbar).
- [x] `/cmd_vel` wirkt (vor/zurück/rotieren).
- [x] `/odom_raw` plausibel (x steigt bei Vorwärtsfahrt, \(\theta\) ändert bei Rotation).
- [x] Timeout-Failsafe stoppt deterministisch nach ~\(2\,\mathrm{s}\).
- [x] `/esp32/heartbeat` läuft bei ~\(1\,\mathrm{Hz}\).

---

## 3) Artefakte (Repo-Wahrheit)

- Firmware: `firmware/src/main.cpp`
- Konfiguration: `firmware/include/config.h`
- Build: `firmware/platformio.ini`

---

## 4) Systemkontext (kurz, nur für Phase-Verständnis)

```
ESP32-S3 (micro-ROS Client) ──USB-CDC 921600──► Pi 5 (Docker)
Core 0: Control @ 100 Hz                    amr_agent: micro-ros-agent
Core 1: micro-ROS comms                     amr_dev: ROS 2 Tools/Workspace
```

---

## 5) Schnittstellen dieser Phase (verifiziert)

### 5.1 Topics

| Topic | Typ | Dir | Zweck |
|------|-----|-----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Fahrbefehl (linear.x, angular.z) |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | Odometrie (x, y, theta) |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | LED/MOSFET |

---

## 6) Verifikation (Testergebnisse vom 2025-12-20)

| Test | Ergebnis | Status |
|------|----------|:------:|
| Agent-Verbindung | `fd: 3` stabil | ✅ |
| Heartbeat | ~\(1\,\mathrm{Hz}\) | ✅ |
| Vorwärts | `linear.x = 0.15` → Räder vorwärts | ✅ |
| Rückwärts | `linear.x = -0.15` → Räder rückwärts | ✅ |
| Drehen links | `angular.z = 0.5` → links | ✅ |
| Drehen rechts | `angular.z = -0.5` → rechts | ✅ |
| Failsafe | Stop nach ~\(2\,\mathrm{s}\) | ✅ |
| Odom | x/y/\(\theta\) plausibel | ✅ |

**Odom-Beispiel (nach Testfahrt):**

```yaml
x: 0.899
y: -0.329
theta: 6.09
```

---

## 7) Konfigurationsstand dieser Phase (Auszug, v3.2.0)

| Parameter             | Wert    | Zweck               |
| --------------------- | ------- | ------------------- |
| `LOOP_RATE_HZ`        | 100     | Control-Loop        |
| `ODOM_PUBLISH_HZ`     | 20      | Odom Publish (Soll) |
| `FAILSAFE_TIMEOUT_MS` | 2000    | Timeout bis Stop    |
| `MOTOR_PWM_FREQ`      | 20000   | PWM                 |
| `MOTOR_PWM_BITS`      | 8       | PWM-Auflösung       |
| `PWM_DEADZONE`        | 35      | Mindest-PWM         |
| `WHEEL_DIAMETER`      | 0.065 m | Kinematik           |
| `WHEEL_BASE`          | 0.178 m | Kinematik           |

### PWM-Kanäle (A↔B getauscht)

```cpp
#define PWM_CH_LEFT_A  1
#define PWM_CH_LEFT_B  0
#define PWM_CH_RIGHT_A 3
#define PWM_CH_RIGHT_B 2
```

### Control-Mode (Phase-Entscheidung)

- Feedforward aktiv (`feedforward_gain = 2.0`)
- PID deaktiviert (`Kp = Ki = Kd = 0.0`)

---

## 8) Hardwarebezug (nur was für Phase 1 nötig ist)

| Komponente          | Rolle                      |
| ------------------- | -------------------------- |
| Seeed XIAO ESP32-S3 | micro-ROS Client + Control |
| Cytron MDD3A        | Dual-PWM Motortreiber      |
| JGA25-370 (2×)      | Antrieb + Encoder          |
| Raspberry Pi 5      | micro-ROS Agent (Docker)   |

---

## 9) Smoke-Test (kurz, reproduzierbar)

> Zweck: „Phase 1 läuft noch“ in <5 Minuten prüfen.

1. Topics sichtbar:

```bash
ros2 topic list
```

1. Heartbeat:

```bash
ros2 topic echo /esp32/heartbeat
```

1. Odom einmal:

```bash
ros2 topic echo /odom_raw --once
```

1. Motor (⚠️ aufbocken):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

---

## 10) Bekannte Einschränkungen (offen für spätere Phasen)

1. Open-Loop (keine echte Geschwindigkeitsregelung)
2. Encoder A-only (Richtung aus Sollwert abgeleitet)
3. Effektive Odom-Rate geringer als Soll (Serial-Transport)

---

## 11) Übergabe an Phase 2/3 (Was ist jetzt „bereit“?)

- Drivebase ist über ROS steuerbar (`/cmd_vel`)
- Odometrie-Grundsignal liegt vor (`/odom_raw`)
- Kommunikations- und Safety-Grundverhalten ist nachgewiesen (Reconnect, Failsafe, Heartbeat)

---

## 12) Changelog (Phase-relevant)

| Version | Datum      | Änderung                                                      |
| ------- | ---------- | ------------------------------------------------------------- |
| v1.0    | 2025-12-19 | Initial                                                       |
| v3.1.0  | 2025-12-20 | Baudrate 921600, PID testweise aktiv                          |
| v3.2.0  | 2025-12-20 | PWM A↔B, Feedforward stabil, PID deaktiviert, Tests bestanden |
