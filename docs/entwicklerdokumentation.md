# Entwicklerdokumentation – AMR Low-Level Controller (ESP32-S3 Firmware)

**Komponente:** AMR-LLC Firmware (ESP32-S3)
**Version:** 3.2.0
**Stand:** 2025-12-20
**Status:** Phase 1–3 verifiziert, Phase 4 folgt

---

## 0. Ziel und Regel dieser Entwicklerdokumentation

### Ziel

Diese Entwicklerdokumentation ermöglicht es, die Firmware **zu bauen, zu flashen, zu konfigurieren, lokal zu testen und reproduzierbar zu debuggen**. Sie beschreibt **Dateien, Schnittstellen auf Code-Ebene, Parameter, Kommandos und typische Fehlerbilder**.

### Regel (Scope-Grenze)

- **Developer View:** Repo-Pfade, Build/Flash/Run, Konfig-Parameter, Tasks/Threads, HAL, ROS-Topic-API aus Entwicklerperspektive, Smoke-Tests, Troubleshooting.

---

## 1. Codebasis und Einstiegsstellen

### 1.1 Quellpfade (Single Source of Truth im Repo)

- Firmware-Entry: `firmware/src/main.cpp`
- Parameter: `firmware/include/config.h`
- Build-Config: `firmware/platformio.ini`

### 1.2 Versionslogik

- Release-Notizen stehen im Changelog (Abschnitt 11).
- Die Doku beschreibt den Stand **v3.2.0**; Abweichungen im Code haben Vorrang.

---

## 2. Architektur (aus Entwicklerperspektive)

### 2.1 Dual-Core Task-Zuordnung (ESP32-S3 / FreeRTOS)

Ziel der Aufteilung: **deterministischer Control-Loop** getrennt von **best-effort Kommunikation**.

```

ESP32-S3 Firmware (v3.2.0)

Core 0 (Control / RT-nah):

* controlTask @ 100 Hz
* Encoder ISR
* Odom-Integration
* Failsafe-Check
* Motor PWM Ausgabe

Core 1 (Comms / best-effort):

* Arduino loop()
* micro-ROS executor spin
* Publish: /odom_raw (Soll: 20 Hz), /esp32/heartbeat (~1 Hz)

Shared Data:

* Datenaustausch über SharedData + Mutex/Semaphore

````

### 2.2 Datenrichtung (für Debugging relevant)

- Core 1 → Core 0: Zielwerte (`/cmd_vel`), LED-Command, `last_cmd_time`
- Core 0 → Core 1: Odometrie-Zustand (`x,y,theta`), Status

---

## 3. HAL: Pinout und PWM-Kanäle

### 3.1 Pin-Mapping (Ist-Stand v3.2.0)

| Ressource | Pin | PWM-Kanal | Verantwortlich | Zweck |
|-----------|-----|-----------|----------------|------|
| Motor L-A | D0 | CH 1 | Core 0 | PWM vorwärts |
| Motor L-B | D1 | CH 0 | Core 0 | PWM rückwärts |
| Motor R-A | D2 | CH 3 | Core 0 | PWM vorwärts |
| Motor R-B | D3 | CH 2 | Core 0 | PWM rückwärts |
| Encoder L | D6 | – | Core 0 | ISR (Rising) |
| Encoder R | D7 | – | Core 0 | ISR (Rising) |
| LED/MOSFET | D10 | – | Core 0 | Digital Out |
| I²C SDA/SCL | D4/D5 | – | Core 1 | reserviert (IMU) |
| Servo Pan/Tilt | D8/D9 | – | Core 1 | reserviert |

### 3.2 PWM-Channel-Swap (Richtungskorrektur)

```cpp
// firmware/include/config.h
#define PWM_CH_LEFT_A  1
#define PWM_CH_LEFT_B  0
#define PWM_CH_RIGHT_A 3
#define PWM_CH_RIGHT_B 2
````

---

## 4. Control-Loop Implementierung (Core 0)

### 4.1 Task: `controlTask`

| Eigenschaft | Wert                              |
| ----------- | --------------------------------- |
| Frequenz    | `LOOP_RATE_HZ = 100`              |
| Timing      | `vTaskDelayUntil`                 |
| Priorität   | hoch (`configMAX_PRIORITIES - 1`) |
| Stack       | 4096 Bytes                        |

### 4.2 Ablauf (Debug-Reihenfolge)

1. SharedData lesen (Mutex)
2. Encoderstände atomar lesen (ISR-safe)
3. Odometrie integrieren (`x, y, theta`)
4. Rad-Sollwerte berechnen (Inverse Kinematik)
5. Stellgröße berechnen (Feedforward, PID aktuell = 0)
6. Failsafe prüfen (`FAILSAFE_TIMEOUT_MS`)
7. SharedData schreiben (Mutex)
8. Motor-PWM ausgeben

### 4.3 Stellgrößenberechnung (Feedforward + PID-Stub)

```cpp
// Twist -> Radgeschwindigkeit
float set_v_l = target_v - (target_w * WHEEL_BASE / 2.0f);
float set_v_r = target_v + (target_w * WHEEL_BASE / 2.0f);

// Feedforward + PID (PID deaktiviert: Kp=Ki=Kd=0)
float feedforward_gain = 2.0f;
float pwm_l = feedforward_gain * set_v_l + pid_left.compute(set_v_l, v_enc_l, dt);
float pwm_r = feedforward_gain * set_v_r + pid_right.compute(set_v_r, v_enc_r, dt);

pwm_l = constrain(pwm_l, -1.0f, 1.0f);
pwm_r = constrain(pwm_r, -1.0f, 1.0f);

hal_motor_write(pwm_l, pwm_r);
```

**Entwicklerhinweis:** PID bleibt deaktiviert, solange Encoder-Richtung (A-only) nicht zuverlässig bestimmt ist.

---

## 5. micro-ROS Loop (Core 1)

### 5.1 Aufgaben im `loop()`

- `rclc_executor_spin_some` (Rx/Tx)
- Publish `/odom_raw` (Soll: `ODOM_PUBLISH_HZ = 20`)
- Publish `/esp32/heartbeat` (~1 Hz)
- Empfang `/cmd_vel`, `/esp32/led_cmd` → SharedData aktualisieren

### 5.2 SharedData Struktur (Kontrakt zwischen Cores)

```cpp
struct SharedData {
  // Input (Core 1 -> Core 0)
  float target_lin_x;
  float target_ang_z;
  bool  led_cmd_active;
  unsigned long last_cmd_time;

  // Output (Core 0 -> Core 1)
  float odom_x;
  float odom_y;
  float odom_theta;
};
```

---

## 6. ROS 2 Topic-API (für Integration/Debug)

| Topic              | Typ                    | Dir | QoS         | Soll-Frequenz |
| ------------------ | ---------------------- | --- | ----------- | ------------- |
| `/cmd_vel`         | `geometry_msgs/Twist`  | Sub | Reliable    | extern        |
| `/odom_raw`        | `geometry_msgs/Pose2D` | Pub | Best Effort | 20 Hz         |
| `/esp32/heartbeat` | `std_msgs/Int32`       | Pub | Best Effort | 1 Hz          |
| `/esp32/led_cmd`   | `std_msgs/Bool`        | Sub | Reliable    | extern        |

Beispiel Payloads:

```yaml
# /cmd_vel
linear:  {x: 0.15}
angular: {z: 0.50}
```

```yaml
# /odom_raw
x: 0.899
y: -0.329
theta: 6.09
```

---

## 7. Konfiguration (`config.h`) – Parameter, die Entwickler typischerweise anfassen

### 7.1 Timing/Safety

| Parameter             | Wert |
| --------------------- | ---- |
| `LOOP_RATE_HZ`        | 100  |
| `ODOM_PUBLISH_HZ`     | 20   |
| `FAILSAFE_TIMEOUT_MS` | 2000 |

### 7.2 Kinematik/Encoder

| Parameter             | Wert    |
| --------------------- | ------- |
| `WHEEL_DIAMETER`      | 0.065 m |
| `WHEEL_BASE`          | 0.178 m |
| `TICKS_PER_REV_LEFT`  | 374.3   |
| `TICKS_PER_REV_RIGHT` | 373.6   |

### 7.3 PWM/Antrieb

| Parameter          | Wert  |
| ------------------ | ----- |
| `MOTOR_PWM_FREQ`   | 20000 |
| `MOTOR_PWM_BITS`   | 8     |
| `PWM_DEADZONE`     | 35    |
| `feedforward_gain` | 2.0   |

---

## 8. Build / Flash / Monitor (PlatformIO)

### 8.1 Flash (macOS)

```bash
cd ~/daten/start/IoT/AMR/amr-platform/firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

### 8.2 Serial Monitor

```bash
pio device monitor -b 921600
```

---

## 9. Host-Umgebung (Docker) – nur für Dev-Tests relevant

### 9.1 Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose ps
```

### 9.2 Agent-Logs (Verbindung prüfen)

```bash
docker compose logs microros_agent --tail 10
```

---

## 10. Smoke-Tests (Developer Checks)

> Ziel: schnell feststellen, ob Build/Flash/Link/API noch funktioniert.

1. Topics sichtbar:

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
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

1. Motor (⚠️ Räder aufbocken):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

Stop per `Ctrl+C` → Failsafe nach ~2 s.

---

## 11. Troubleshooting (typische Dev-Fehlerbilder)

| Symptom                       | Wahrscheinliche Ursache                    | Dev-Fix                                        |
| ----------------------------- | ------------------------------------------ | ---------------------------------------------- |
| Topics fehlen                 | Agent nicht verbunden                      | `docker compose restart microros_agent`        |
| Motor reagiert nicht          | Gain/Deadzone zu niedrig, Pinout falsch    | `feedforward_gain`, `PWM_DEADZONE`, HAL prüfen |
| Failsafe stoppt während Fahrt | Timeout zu klein / Kommandorate zu niedrig | `FAILSAFE_TIMEOUT_MS` prüfen                   |
| PID eskaliert                 | Encoder-Richtung unklar (A-only)           | PID deaktiviert lassen, Richtungskette fixen   |
| Räder drehen falsch           | Kanal-/Leitungsdreher                      | PWM A/B Mapping prüfen                         |

---

## 12. Changelog (Entwicklerrelevant)

### v3.2.0 (2025-12-20)

- PWM-Kanäle A↔B getauscht (Richtung)
- Feedforward stabil, PID deaktiviert
- Timeout auf `2000 ms`
- Smoke-Tests: Motion + Heartbeat + Odom plausibel

### v3.1.0 (2025-12-20)

- Baudrate `921600`
- PID testweise aktiv (Kp=1.0) → instabil wegen Encoder-Polarität

### v3.0.0 (2025-12-14)

- Dual-Core/FreeRTOS Einführung
- SharedData + Mutex Synchronisation
- Pose2D Optimierung
