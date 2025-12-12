# AMR Implementierungsplan

## Vom Schaltplan zur autonomen Navigation

> **Version:** 1.2 | **Stand:** 2025-12-12 | **Firmware:** v0.3.0-serial

---

## Das Grundprinzip: Vertikale Scheiben statt horizontaler Schichten

Ein häufiger Fehler bei Robotik-Projekten: Man baut zuerst die gesamte Hardware auf, dann die gesamte Firmware, dann die gesamten Treiber – und am Ende, beim ersten Integrationstest, funktioniert nichts. Die Fehlersuche wird zum Albtraum, weil alles gleichzeitig neu ist.

**Unser Ansatz:** Wir schneiden das System in *vertikale Scheiben*. Jede Phase liefert ein lauffähiges Teilsystem, das wir testen können, bevor die nächste Komplexitätsstufe hinzukommt.

```
Klassisch (riskant):          Unser Weg (inkrementell):

┌──────────────────┐          Phase 1: ──────────────────► ✅
│    Navigation    │                   Motor + Failsafe
├──────────────────┤
│   Wahrnehmung    │          Phase 2: ──────────────────► ◄── AKTUELL
├──────────────────┤                   + Encoder + Odom
│    Firmware      │
├──────────────────┤          Phase 3: ──────────────────►
│    Hardware      │                   + LiDAR + SLAM
└──────────────────┘
       ↓                      Phase 4: ──────────────────►
  Big Bang Test                        + Navigation
  (Chaos)
                              Phase 5: ──────────────────►
                                       + Kamera + AI
```

---

## Phase 0: Fundament (Woche 1–2) ✅

**Ziel:** Eine saubere Entwicklungsumgebung, die uns später keine Steine in den Weg legt.

**Status:** ✅ Abgeschlossen

- Raspberry Pi OS Lite (64-bit, Bookworm)
- Docker und Docker Compose
- Hailo-8L Treiber (HailoRT 4.23.0)
- Git-Workflow Mac ↔ GitHub ↔ Pi

---

## Phase 1: Der erste Lebenshauch (Woche 3–4) ✅

**Ziel:** Ein Rad dreht sich auf Befehl. Das klingt trivial – ist aber der Beweis, dass die gesamte Kette von ROS 2 bis zum Motor funktioniert.

**Status:** ✅ Abgeschlossen (2025-12-12)

### 1.1 Erreichte Meilensteine

| Komponente | Status |
|------------|--------|
| ESP32 Serial-Bridge Firmware | ✅ v0.3.0-serial |
| Differential Drive Kinematik | ✅ |
| Deadzone-Kompensation | ✅ |
| Failsafe (500ms Timeout) | ✅ |
| ROS 2 Serial Bridge Node | ✅ |
| Docker Integration | ✅ |
| Teleop Tastatursteuerung | ✅ |

### 1.2 Architektur-Entscheidung

**Problem:** micro-ROS Build scheitert an Python 3.13 (Raspberry Pi OS Bookworm)

**Lösung:** Serial-Bridge als Workaround

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Serial-Bridge Architektur                        │
├─────────────────────────────────────────────────────────────────────────┤
│   ROS 2 (Docker)              USB-Serial              ESP32-S3          │
│  ┌──────────────┐            ┌────────┐            ┌──────────────┐    │
│  │ /cmd_vel     │───────────▶│ Bridge │───────────▶│ Differential │    │
│  │ Twist msg    │            │ Node   │  V:0.2,    │ Drive        │    │
│  └──────────────┘            │ Python │  W:0.5\n   │ Kinematik    │    │
│                              └────────┘            └──────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Cytron MDD3A – Dual-PWM Steuerung

> ⚠️ **Kritisch:** Der MDD3A verwendet **kein** DIR-Pin, sondern zwei PWM-Signale pro Motor!

| M1A (PWM) | M1B (PWM) | Ergebnis |
|-----------|-----------|----------|
| 200 | 0 | Vorwärts |
| 0 | 200 | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| 200 | 200 | Active Brake |

### 1.4 Deadzone-Kompensation

Kleine Geschwindigkeiten (z.B. bei Drehung mit W=0.5) fielen unter die Motor-Deadzone. Lösung:

```cpp
void hal_motor_set(uint8_t ch_a, uint8_t ch_b, float speed) {
    // Unter 5% ignorieren
    if (abs(speed) < 0.05f) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }

    // Deadzone-Kompensation: Skaliere auf [PWM_DEADZONE, PWM_MAX]
    int pwm = PWM_DEADZONE + (int)(abs(speed) * (MOTOR_PWM_MAX - PWM_DEADZONE));
    // ...
}
```

### 1.5 Validierung

```bash
# Teleop starten
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:** `i`=Vorwärts, `,`=Rückwärts, `j`/`l`=Drehen, `k`=Stopp

**Meilenstein Phase 1:** ✅ Teleop funktioniert, Failsafe aktiv.

---

## Phase 2: Bewegung mit Feedback (Woche 5–6) ◄── AKTUELL

**Ziel:** Der Roboter weiß, wo er ist (Odometrie). Wir können ihn per Tastatur steuern und seine Position in RViz2 sehen.

### 2.1 Encoder-Integration

Die Encoder liefern Impulse pro Radumdrehung. Der ESP32 zählt diese in einem Interrupt.

**Pin-Belegung:**

| Encoder | Pin | Interrupt |
|---------|-----|-----------|
| Links | D6 | RISING |
| Rechts | D7 | RISING |

**ISR:**

```cpp
volatile long encoder_left_ticks = 0;
volatile long encoder_right_ticks = 0;

void IRAM_ATTR encoder_left_isr() {
    encoder_left_ticks++;
}

void IRAM_ATTR encoder_right_isr() {
    encoder_right_ticks++;
}
```

### 2.2 Encoder-Kalibrierung

1. Rad markieren (Strich auf Reifen und Chassis)
2. Kalibrierungs-Sketch hochladen
3. Rad **exakt 10 Umdrehungen** von Hand drehen
4. Ticks ablesen und durch 10 teilen
5. **Für beide Motoren einzeln durchführen!**

Werte in `config.h` eintragen:

```cpp
#define TICKS_PER_REV_LEFT     390.5f  // Kalibriert
#define TICKS_PER_REV_RIGHT    392.0f  // Kalibriert
```

### 2.3 Odometrie-Berechnung

**Differentialkinematik:**

```
d_left  = (delta_ticks_left / TICKS_PER_REV) × WHEEL_CIRCUMFERENCE
d_right = (delta_ticks_right / TICKS_PER_REV) × WHEEL_CIRCUMFERENCE

d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / WHEEL_BASE

x += d_center × cos(theta + d_theta/2)
y += d_center × sin(theta + d_theta/2)
theta += d_theta
```

### 2.4 Serial-Protokoll erweitern

```
ESP32 → Host: ODOM:<left_ticks>,<right_ticks>,<x>,<y>,<theta>\n
```

### 2.5 ROS 2 Publisher

- `/odom` (Typ: `nav_msgs/Odometry`) – Position und Orientierung
- TF-Broadcast: `odom` → `base_link`

### 2.6 Validierung

- Roboter 1 m fahren → Odometrie muss ~1 m anzeigen (±5%)
- RViz2: Odometrie-Pfad visualisieren

**Meilenstein Phase 2:** Roboter fährt per Tastatur, Odometrie in RViz2 stimmt auf ±5 %.

---

## Phase 3: Sehen lernen – LiDAR & SLAM (Woche 7–9)

**Ziel:** Der Roboter baut eine Karte seiner Umgebung.

### 3.1 LiDAR-Treiber

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### 3.2 SLAM-Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=slam_params.yaml
```

**Meilenstein Phase 3:** Eine speicherbare Karte des Testraums existiert.

---

## Phase 4: Autonome Navigation (Woche 10–12)

**Ziel:** Wir setzen ein Ziel auf der Karte, der Roboter fährt autonom hin.

### 4.1 Nav2 Stack

| Komponente | Funktion |
|------------|----------|
| **AMCL** | Lokalisierung auf bekannter Karte |
| **Planner Server** | Globaler Pfad (A* / Dijkstra) |
| **Controller Server** | Lokale Hindernisvermeidung |
| **Costmap** | Hinderniskarte aus Sensordaten |
| **BT Navigator** | Verhaltenssteuerung |

**Meilenstein Phase 4:** Roboter navigiert autonom, weicht Hindernissen aus.

---

## Phase 5: Wahrnehmungserweiterung – Kamera & AI (Woche 13–15)

**Ziel:** Der Roboter erkennt Objekte und kann darauf reagieren.

- IMX296 Global Shutter Kamera
- YOLOv8 auf Hailo-8L (31 FPS validiert)
- Personen-Erkennung → Stopp-Verhalten

**Meilenstein Phase 5:** Roboter stoppt, wenn eine Person erkannt wird.

---

## Phase 6: Integration & Härtung (Woche 16–18)

**Ziel:** Robustes System für Demo und Dokumentation.

- Sensor Fusion (EKF)
- Systemstart automatisieren
- Demo: Karte, Waypoints, Video

**Meilenstein Phase 6:** Robustes System, startet automatisch, Demo-fähig.

---

## Risikomatrix

| Risiko | Wahrscheinlichkeit | Impact | Mitigation |
|--------|-------------------|--------|------------|
| libcamera-Inkompatibilität | Niedrig | Hoch | Raspberry Pi OS ✅ |
| Odometrie-Drift | Hoch | Mittel | Kalibrierung, später EKF |
| Hailo-Treiber instabil | Mittel | Mittel | Navigation funktioniert auch ohne AI |
| Nav2-Tuning aufwändig | Hoch | Mittel | Viel Zeit einplanen |
| **micro-ROS inkompatibel** | ~~Hoch~~ | ~~Hoch~~ | ✅ **Serial-Bridge** |
| **MDD3A-Ansteuerung** | ~~Hoch~~ | ~~Hoch~~ | ✅ **Dual-PWM** |

---

## Zeitplan (Übersicht)

```
Woche:  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18
        ════════════════════════════════════════════════════
Phase 0 ████                                                 Fundament     ✅
Phase 1       ████                                           Motor-Test    ✅
Phase 2             ████                                     Odometrie     ◄── AKTUELL
Phase 3                   ██████                             SLAM
Phase 4                            ██████                    Navigation
Phase 5                                     ██████           Kamera/AI
Phase 6                                              ██████  Integration
        ════════════════════════════════════════════════════
```

---

## Checkliste pro Phase

Jede Phase ist erst abgeschlossen, wenn:

- [x] Die definierten Tests bestanden sind
- [x] Der Code committet und dokumentiert ist
- [x] Die Konfigurationsdateien versioniert sind
- [x] Ein kurzes Protokoll die Ergebnisse festhält
- [x] Der nächste Schritt klar ist

**Phase 1:** ✅ Alle Punkte erfüllt

---

*Dieser Plan folgt dem Prinzip: Jede Woche ein lauffähiges System. Lieber weniger Features, die funktionieren, als viele Features, die zusammen crashen.*

*Aktualisiert: 2025-12-12 | Firmware: v0.3.0-serial | Phase 1 abgeschlossen*
