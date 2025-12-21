# ESP32-S3 Firmware (Drive-Base / Low-Level Controller)

**Stand:** 2025-12-20
**Firmware:** v3.2.0
**Projektstatus:** Phase 1–3 ✅ abgeschlossen (Phase 4 als Nächstes)

Der **Seeed XIAO ESP32-S3** ist der Echtzeit-Controller des AMR. Er übernimmt zeitkritische Funktionen (Motoransteuerung, Encoder-ISR, Odometrie, Failsafe) und stellt die ROS-Schnittstellen als **micro-ROS Client** bereit. Der Raspberry Pi 5 führt den High-Level-Stack (LiDAR, SLAM, Nav2) im Docker aus.

---

## Regel

**Echtzeit-Regelung strikt von Kommunikation trennen:**

* **Core 0 (Pro CPU):** Control Loop in fester Frequenz (100 Hz)
* **Core 1 (App CPU):** micro-ROS Executor + Publisher (Best Effort)
* Datenaustausch über **Shared Memory** (Mutex-geschützt)

---

## Architektur

### Dual-Core Tasks (FreeRTOS)

* **Core 0: `controlTask` @ 100 Hz**

  * Encoder ISR auswerten (A-only)
  * Odometrie integrieren (x, y, θ)
  * Antrieb: **Feedforward (Open-Loop)**, PID aktuell deaktiviert
  * Failsafe: Motor Stop nach **2000 ms** ohne gültiges Command

* **Core 1: `loop()` / micro-ROS**

  * `/cmd_vel` empfangen
  * `/odom_raw` publizieren (20 Hz)
  * `/esp32/heartbeat` publizieren (1 Hz)
  * `/esp32/led_cmd` empfangen

---

## ROS-Schnittstellen (micro-ROS)

| Topic              | Typ                    | Richtung | Zweck                       |
| ------------------ | ---------------------- | -------: | --------------------------- |
| `/cmd_vel`         | `geometry_msgs/Twist`  |      Sub | Sollgeschwindigkeit (v, ω)  |
| `/odom_raw`        | `geometry_msgs/Pose2D` |      Pub | Odometrie (x, y, θ)         |
| `/esp32/heartbeat` | `std_msgs/Int32`       |      Pub | Verbindung/Liveness (~1 Hz) |
| `/esp32/led_cmd`   | `std_msgs/Bool`        |      Sub | LED/MOSFET Steuerung        |

**Transport:** USB-CDC Serial, `Serial.begin(921600)` (siehe `firmware/src/main.cpp`).

---

## Hardware-Ansteuerung

**Motortreiber:** Cytron **MDD3A** (Dual-PWM, kein DIR-Pin)

* Pro Motor: 2× PWM (Vorwärts/Rückwärts)
* PWM: `20 kHz`, 8-Bit (0–255), Deadzone `35`

### Pin-Mapping (ESP32-S3 XIAO)

| Funktion                     | Pin     | Status |
| ---------------------------- | ------- | -----: |
| Motor L PWM A/B              | D0 / D1 |      ✅ |
| Motor R PWM A/B              | D2 / D3 |      ✅ |
| Encoder L (A)                | D6      |      ✅ |
| Encoder R (A)                | D7      |      ✅ |
| LED/MOSFET                   | D10     |      ✅ |
| I²C SDA/SCL (IMU reserviert) | D4 / D5 |      ⏳ |
| Servo Pan/Tilt (reserviert)  | D8 / D9 |      ⏳ |

**Wichtig:** PWM-Kanäle wurden für korrekte Fahrtrichtung getauscht (A↔B), siehe `firmware/include/config.h`:

* `PWM_CH_LEFT_A=1`, `PWM_CH_LEFT_B=0`, `PWM_CH_RIGHT_A=3`, `PWM_CH_RIGHT_B=2`

---

## Regelung (aktueller Stand)

### Feedforward (aktiv)

* `feedforward_gain = 2.0f`
* Ausgabe wird auf `[-1.0, 1.0]` begrenzt und an HAL übergeben.

### PID (deaktiviert)

PID ist aktuell **aus**, weil die Encoder **A-only** liefern und die Richtung aus dem Sollwert abgeleitet wird. Mit aktivem PID kam es zu Instabilität/Eskalation (Fehlinterpretation der Bewegungsrichtung).

---

## Relevante Parameter (aus `config.h`)

| Parameter    |                                                                Wert |
| ------------ | ------------------------------------------------------------------: |
| Control Loop |                                                `LOOP_RATE_HZ = 100` |
| Odom Publish |                                              `ODOM_PUBLISH_HZ = 20` |
| Failsafe     |                                        `FAILSAFE_TIMEOUT_MS = 2000` |
| PWM          | `MOTOR_PWM_FREQ = 20000`, `MOTOR_PWM_BITS = 8`, `PWM_DEADZONE = 35` |
| Kinematik    |                      `WHEEL_DIAMETER = 0.065`, `WHEEL_BASE = 0.178` |
| Encoder      |         `TICKS_PER_REV_LEFT = 374.3`, `TICKS_PER_REV_RIGHT = 373.6` |

---

## Build / Flash / Monitor (PlatformIO)

### Flash (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform/firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

### Monitor

**Hinweis:** `main.cpp` nutzt `Serial.begin(921600)`. Falls du Logs per Monitor sehen willst:

```bash
pio device monitor -b 921600
```

(Dein `platformio.ini` steht aktuell auf `monitor_speed = 115200`; das passt nicht zu `Serial.begin(921600)`.)

---

## Smoke-Tests (über Pi / ROS)

* `ros2 topic echo /esp32/heartbeat` → Counter ~1 Hz
* `ros2 topic pub /cmd_vel ...` → Motor reagiert
* `ros2 topic echo /odom_raw --once` → plausible Werte

---

## Bekannte Einschränkungen

1. **Open-Loop**: keine echte Geschwindigkeitsregelung (PID aus)
2. **Encoder A-only**: Richtung wird aus Sollwert abgeleitet
3. **Odometrie-Rate**: publish 20 Hz, effektiv durch Serial/Agent oft geringer

---

## Dateien (Single Source of Truth)

* `firmware/src/main.cpp`
* `firmware/include/config.h`
* `firmware/platformio.ini`
