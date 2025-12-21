# Systemdokumentation – AMR Low-Level Controller (ESP32-S3 Drivebase)

**System-ID:** AMR-LLC (Low-Level Controller)
**Version:** 3.2.0
**Datum:** 2025-12-20
**Systemstatus:** Phase 1–3 funktionsfähig, Phase 4 folgt (URDF/TF/EKF)

---

## Ziel dieser Systemdokumentation

1. **Systemverständnis** herzustellen
   - Zweck, Systemgrenzen, Betriebsarten, Annahmen und Abhängigkeiten sind eindeutig definiert.

2. **Schnittstellen eindeutig festzulegen**
   - Externe Schnittstellen (ROS-Topics, Datenbedeutung, Frequenzen) sind so beschrieben, dass nachgelagerte Systeme (URDF/TF/EKF/SLAM/Nav2) konsistent integrieren können.

3. **Sicherheits- und Fail-safe-Verhalten nachvollziehbar zu machen**
   - Welche Ereignisse zu welchem sicheren Zustand führen (z. B. Timeout → Motorstopp) ist klar dokumentiert.

4. **Nachweisbarkeit zu ermöglichen**
   - Verifikationspunkte beschreiben, *was* am System geprüft wird und *welches Verhalten* als bestanden gilt (ohne Build/Flash-Anleitung).

## 1. Zweck, Systemgrenzen, Annahmen

### 1.1 Zweck

Der AMR-LLC steuert die Drivebase (Motoren) und stellt eine ROS-2-kompatible Low-Level-Schnittstelle bereit. Er wandelt Geschwindigkeitsbefehle in Motorausgänge um und liefert Odometrie als Basis für spätere Lokalisierung und Navigation.

### 1.2 Systemgrenzen (Scope)

**Im System (AMR-LLC + Host-Integration):**

- Motoransteuerung (Dual-PWM)
- Encoder-Erfassung (A-only)
- Odometrie-Integration (Pose2D)
- Failsafe bei Kommunikationsausfall
- micro-ROS Transport via USB-CDC
- Host-Seite: micro-ROS Agent im Docker (Pi 5)

**Außerhalb des Systems (nicht Bestandteil dieser Version):**

- URDF/robot_state_publisher
- TF-Baum nach REP-105 (map/odom/base_*)
- EKF (robot_localization)
- SLAM / Nav2

### 1.3 Annahmen

- Host (Pi 5) stellt micro-ROS Agent bereit und sendet `/cmd_vel` regelmäßig.
- Mechanik ist differential drive (2 angetriebene Räder).
- Sicherheitsfunktion „Timeout → Stop“ ist lokal (ESP32) wirksam, unabhängig vom Hostzustand.

---

## 2. Systemkontext und Datenflüsse

### 2.1 Komponenten

- **ESP32-S3**: Echtzeitnahe Drivebase-Steuerung + micro-ROS Client
- **Motortreiber**: Cytron MDD3A (Dual-PWM)
- **Motoren**: JGA25-370 mit Hall-Encoder
- **Raspberry Pi 5**: Docker-Host (micro-ROS Agent + ROS 2 Tools)

### 2.2 Datenfluss (logisch)

```

ROS 2 /cmd_vel  ──►  Pi 5 (micro-ROS Agent)  ──USB-CDC──►  ESP32-S3 (micro-ROS Client)
│
├─► Motor PWM (links/rechts)
└─► /odom_raw, /esp32/heartbeat  ──► zurück zum ROS 2 System

```

### 2.3 Betriebsarten (operativ)

- **Bench Mode (Werkbank):** Räder entlastet/aufgebockt, Funktionsnachweis.
- **Drive Mode (Boden):** Niedrige Geschwindigkeit, kontrollierte Tests.
- **Fault Mode:** Timeout oder Fehler → Motoren aus.

---

## 3. Funktionale Anforderungen (Systemverhalten)

### 3.1 Motion Control (Soll → Stellgröße)

- Eingabe: `/cmd_vel` (linear.x, angular.z)
- Ausgabe: PWM links/rechts (begrenzter Wertebereich, Deadzone berücksichtigt)

### 3.2 Odometrie (Zustandsschätzung v1)

- Eingabe: Encoder-Ticks (A-only) + Sollrichtung (Heuristik)
- Ausgabe: `/odom_raw` als `Pose2D` (x, y, theta)

### 3.3 Sicherheitsfunktion (Failsafe)

- Regel: Bleiben gültige Steuer-Updates aus, werden Motoren nach `FAILSAFE_TIMEOUT_MS` abgeschaltet (PWM = 0).
- Ziel: Verhindern von „blindem Weiterfahren“ bei Kommunikationsverlust.

---

## 4. Echtzeit- und Ausführungsarchitektur

### 4.1 Task-Trennung (Dual-Core)

Das System trennt **zeitkritische Regel-/Safety-Funktionen** von **Kommunikation**.

**Core 0 – Control Task (Hard/firm real-time)**

- Takt: `LOOP_RATE_HZ = 100` (deterministisch via `vTaskDelayUntil`)
- Verantwortlich für:
  - Encoder-Auswertung (ISR-getrieben)
  - Odometrie-Integration
  - Stellgrößenberechnung (Feedforward; PID aktuell deaktiviert)
  - Failsafe-Überwachung

**Core 1 – Communication (micro-ROS)**

- Verantwortlich für:
  - micro-ROS Executor Spin
  - Publish `/odom_raw` (Soll: `ODOM_PUBLISH_HZ = 20`)
  - Publish `/esp32/heartbeat` (~1 Hz)
  - Transport/Serialisierung

### 4.2 Datenkonsistenz

- Shared-State zwischen Tasks in einem gemeinsamen Datenobjekt.
- Zugriff wird synchronisiert (Mutex/Semaphore), um Race Conditions zu vermeiden.

---

## 5. ROS-2 Schnittstelle (System-API)

### 5.1 Topics (externe Schnittstellen)

| Topic | Typ | Richtung | Ziel | Anmerkung |
|------|-----|----------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Fahrbefehl | nutzt `linear.x`, `angular.z` |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | Odom v1 | Basis für spätere Bridge zu `/odom` |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | Status | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | IO | LED/MOSFET |

### 5.2 Semantik (vereinbartes Verhalten)

- `/cmd_vel.linear.x` in \(\mathrm{m/s}\), `/cmd_vel.angular.z` in \(\mathrm{rad/s}\)
- `/odom_raw` liefert Pose im lokalen Odometrie-Frame (noch ohne TF)

---

## 6. Konfiguration (relevante Systemparameter)

### 6.1 Zeit- und Safety-Parameter

| Parameter | Wert | Bedeutung |
|----------|------|-----------|
| `LOOP_RATE_HZ` | 100 | Control-Zyklus |
| `ODOM_PUBLISH_HZ` | 20 | Odom-Publish (Sollwert) |
| `FAILSAFE_TIMEOUT_MS` | 2000 | Timeout bis Motorstopp |

### 6.2 Motor/Mechanik

| Parameter | Wert | Bedeutung |
|----------|------|-----------|
| `MOTOR_PWM_FREQ` | 20000 | PWM-Frequenz |
| `MOTOR_PWM_BITS` | 8 | Auflösung 0–255 |
| `PWM_DEADZONE` | 35 | Mindest-PWM |
| `WHEEL_DIAMETER` | 0.065 m | Geometrie |
| `WHEEL_BASE` | 0.178 m | Geometrie |

### 6.3 Control-Mode

- Feedforward aktiv (`feedforward_gain = 2.0`)
- PID deaktiviert (`PID_KP = PID_KI = PID_KD = 0.0`)

---

## 7. Hardware-Schnittstellen (I/O)

### 7.1 Motor- und Encoder-I/O

| Signal | Pin | Modus | Zielhardware |
|--------|-----|------|-------------|
| Motor L A/B | D0/D1 | PWM | MDD3A |
| Motor R A/B | D2/D3 | PWM | MDD3A |
| Encoder L A | D6 | IRQ | JGA25-370 |
| Encoder R A | D7 | IRQ | JGA25-370 |
| LED/MOSFET | D10 | GPIO | IRLZ24N |

### 7.2 Reservierte Schnittstellen

- I²C (D4/D5) für IMU (zukünftig)
- Servo PWM (D8/D9) (zukünftig)

---

## 8. Verifikation (Systemnachweis auf Verhaltensebene)

> Ziel: Nachweise beschreiben **was** geprüft wurde und **welches Ergebnis** erwartet wird.
> (Build/Flash-Anleitungen gehören in Entwicklerdoku/Runbook.)

| Prüffall | Erwartetes Systemverhalten |
|---------|-----------------------------|
| Agent-Verbindung | Verbindung stabil; Reconnect nach Reset reproduzierbar |
| Heartbeat | Zähler steigt ~1×/s |
| Motion vor/zurück/drehen | Motoren folgen Vorzeichen von `linear.x` / `angular.z` |
| Failsafe | Nach Ausbleiben von Kommandos: Motoren stop nach ~`FAILSAFE_TIMEOUT_MS` |
| Odometrie plausibel | `x` steigt bei Vorwärtsfahrt; `theta` ändert bei Rotation |

---

## 9. Einschränkungen (bekannte Systemgrenzen)

1. **Open-Loop Control:** keine geschlossene Geschwindigkeitsregelung (PID deaktiviert)
2. **Encoder A-only:** Richtungsinformation wird aus Sollwert abgeleitet
3. **Odom-Rate effektiv geringer:** durch Serial-Transport typisch < Sollwert
4. **Kein TF/URDF in dieser Version:** Integration folgt in Phase 4

---

## 10. Änderungsverlauf (System-Release-Notizen)

### v3.2.0 (2025-12-20)

- Motor-Richtung durch PWM-Kanaltausch korrigiert
- Feedforward als stabiler Control-Mode, PID deaktiviert
- Failsafe Timeout auf \(2000\,\mathrm{ms}\) gesetzt
- Funktionsnachweise (Motion, Heartbeat, Odom-Plausibilität, Failsafe) erbracht

---

## 11. Ausblick / Systemintegration (nächste Systemschicht)

- **Phase 4:** URDF + TF (REP-105) + Odom-Bridge (`Pose2D` → `nav_msgs/Odometry`) + optional EKF
- Ziel: Kompatibilität für SLAM/Nav2 über standardisierte Frames und `/odom`
