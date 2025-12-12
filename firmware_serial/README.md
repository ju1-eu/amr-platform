# AMR Firmware v1.1.0 - ESP32-S3

## Änderungen gegenüber v1.0

### 🔴 Kritische Korrekturen

| Problem | Lösung | Datei |
|---------|--------|-------|
| **MDD3A Ansteuerung falsch** | Dual-PWM statt PWM+DIR | `config.h`, `main.cpp` |
| **Race Condition** | FreeRTOS Mutex für shared state | `main.cpp` |
| **Kein Hardware-Watchdog** | ESP32 Task WDT aktiviert | `main.cpp` |

### 📁 Projektstruktur

```
amr_firmware/
├── platformio.ini      # Build-Konfiguration
├── include/
│   └── config.h        # Hardware-Definitionen, Parameter
└── src/
    └── main.cpp        # Firmware-Hauptprogramm
```

### 🔧 Installation

```bash
# 1. Projekt klonen/kopieren
cd ~/amr_firmware

# 2. micro-ROS Bibliothek bauen (dauert beim ersten Mal)
pio run

# 3. Flashen (ESP32-S3 XIAO angeschlossen)
pio run --target upload

# 4. Serial Monitor (optional)
pio device monitor
```

### ⚡ MDD3A Dual-PWM Logik

Der Cytron MDD3A verwendet **kein** DIR-Pin, sondern zwei PWM-Signale:

| M1A (PWM) | M1B (PWM) | Ergebnis |
|-----------|-----------|----------|
| 200 | 0 | Vorwärts |
| 0 | 200 | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| 200 | 200 | Active Brake |

### 🔒 Safety Features

1. **Failsafe Timeout (500ms)**: Motoren stoppen automatisch wenn kein `/cmd_vel` empfangen
2. **Task Watchdog (5s)**: ESP32 Reset bei Endlosschleife
3. **Mutex-geschützte Variablen**: Keine Race Conditions zwischen Cores
4. **Sauberes Cleanup**: Bei Verbindungsverlust werden alle ROS-Entities freigegeben

### 📊 LED Status-Codes

| Zustand | LED-Muster |
|---------|------------|
| Agent suchen | Langsames Pulsieren |
| Verbunden | Dauerlicht |
| Failsafe | Schnelles Blinken |
| Fehler | SOS-Muster |

### 🧪 Test-Befehle

```bash
# micro-ROS Agent starten (auf Raspberry Pi)
docker compose up -d micro_ros

# Node prüfen
ros2 node list
# Erwartung: /amr_esp32

# Motor-Test
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Drehung testen
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Stopp
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### 📋 Phase 2 TODO

- [ ] Encoder-Auswertung mit Quadratur (Richtungserkennung)
- [ ] Odometrie-Publisher (`/odom`)
- [ ] TF-Broadcast (`odom` → `base_link`)
- [ ] PID-Regelung für Geschwindigkeit
- [ ] Encoder-Kalibrierung (10-Umdrehungen-Test)

---
*Erstellt: 2025-12-12 | Autor: Jan Unger | Standard: REP-103, REP-105*
