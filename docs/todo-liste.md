# ToDo-Liste AMR-Projekt

> **Stand:** 2025-12-12 | **Aktuelle Phase:** 2 (Odometrie)

---

## 📊 Phasen-Übersicht

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 0 | Fundament (OS, Docker, Hailo) | ✅ Abgeschlossen |
| Phase 1 | Motor-Test + Teleop | ✅ **Abgeschlossen** |
| Phase 2 | Encoder + Odometrie | ◄── **AKTUELL** |
| Phase 3 | LiDAR + SLAM | ⬜ |
| Phase 4 | Navigation | ⬜ |
| Phase 5 | Kamera + AI | ⬜ |
| Phase 6 | Integration | ⬜ |

---

## ✅ Phase 1: Abgeschlossen (2025-12-12)

### Erreichte Ziele

- [x] ESP32 Serial-Bridge Firmware v0.3.0
- [x] Differential Drive Kinematik
- [x] Deadzone-Kompensation
- [x] Failsafe (500ms Timeout)
- [x] ROS 2 Serial Bridge Node
- [x] Docker Integration
- [x] Teleop Tastatursteuerung
- [x] Git-Workflow Mac ↔ GitHub ↔ Pi

### Workaround dokumentiert

- micro-ROS Build scheitert an Python 3.13
- **Lösung:** Serial-Bridge statt micro-ROS Agent

---

## 🎯 Phase 2: Odometrie (AKTUELL)

### 2.1 Encoder-Kalibrierung

- [ ] Kalibrierungs-Sketch auf ESP32 flashen
- [ ] Linkes Rad: 10 Umdrehungen drehen, Ticks zählen
- [ ] Rechtes Rad: 10 Umdrehungen drehen, Ticks zählen
- [ ] `TICKS_PER_REV_LEFT` in config.h eintragen
- [ ] `TICKS_PER_REV_RIGHT` in config.h eintragen

### 2.2 ESP32 Firmware erweitern

- [ ] Encoder-ISR implementieren (D6, D7)
- [ ] Odometrie-Berechnung (x, y, theta)
- [ ] Serial-Protokoll erweitern: `ODOM:left,right,x,y,theta\n`
- [ ] Tick-Counter zurücksetzen bei Reset

### 2.3 ROS 2 Bridge erweitern

- [ ] Odometrie parsen
- [ ] `/odom` Topic publizieren (nav_msgs/Odometry)
- [ ] TF-Broadcast: `odom` → `base_link`

### 2.4 Validierung

- [ ] 1 m vorwärts fahren → Odometrie zeigt ~1 m (±5%)
- [ ] 360° drehen → Odometrie zeigt ~360° (±10%)
- [ ] RViz2: Odometrie-Pfad visualisieren

---

## 📋 Nächste Phasen (Vorschau)

### Phase 3: SLAM

- [ ] RPLIDAR A1 in ROS 2 integrieren
- [ ] slam_toolbox konfigurieren
- [ ] Erste Karte erstellen
- [ ] Karte speichern

### Phase 4: Navigation

- [ ] Nav2 Stack konfigurieren
- [ ] AMCL Lokalisierung
- [ ] Autonome Punkt-zu-Punkt Navigation

### Phase 5: Kamera + AI

- [ ] IMX296 Global Shutter integrieren
- [ ] YOLOv8 auf Hailo-8L
- [ ] Personen-Erkennung → Stopp-Verhalten

### Phase 6: Integration

- [ ] Sensor Fusion (EKF)
- [ ] Systemstart automatisieren
- [ ] Demo vorbereiten

---

## 📚 Dokumentation

| Datei | Inhalt | Status |
|-------|--------|--------|
| `01-Pi-OS-flashen.md` | OS-Installation, SSH, Docker | ✅ |
| `02-hailo-setup.md` | HailoRT 4.23.0, Benchmark | ✅ |
| `03-ros2-docker.md` | Container-Setup, URDF | ✅ |
| `04-esp32-firmware.md` | PlatformIO Firmware | ✅ |
| `08-entwicklerdoku-status.md` | Projektstatus | ✅ Aktualisiert |
| `AMR_Implementierungsplan.md` | Phasenplan | ✅ |
| `Industriestandards-AMR.md` | REP-103, REP-105 | ✅ |

---

## 📅 Zeitplan

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

## ✅ Checkliste pro Phase

Jede Phase ist erst abgeschlossen, wenn:

- [x] Die definierten Tests bestanden sind
- [x] Der Code committet und dokumentiert ist
- [x] Die Konfigurationsdateien versioniert sind
- [x] Ein kurzes Protokoll die Ergebnisse festhält
- [x] Der nächste Schritt klar ist

**Phase 1:** Alle Punkte erfüllt ✅

---

## 🔧 Aktuelle Software-Versionen

| Komponente | Version |
|------------|---------|
| ESP32 Firmware | v0.3.0-serial |
| Serial Bridge | v0.3.0 |
| Docker Stack | perception + serial_bridge |
| Git Repo | unger-robotics/amr-platform |

---

*Aktualisiert: 2025-12-12 | Phase 1 abgeschlossen*
