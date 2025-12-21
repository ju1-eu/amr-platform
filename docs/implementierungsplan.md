# AMR Implementierungsplan – vom Drivebase-Prototyp zur autonomen Navigation

> **Version:** 2.1 | **Stand:** 2025-12-21 | **Firmware:** v3.2.0
> **Ist-Stand:** Phase 1–3 ✅ | **Nächster Fokus:** Phase 4 (URDF/TF/EKF)

---

## 0) Ziel und Regel dieses Implementierungsplans

### Ziel

- **Arbeitspakete** (Tasks) je Phase
- **Reihenfolge & Abhängigkeiten**
- **Artefakte** (Code/Configs/Launch-Files)
- **Definition of Done (DoD)** + **Verifikationschecks**
- **Zeitplan** (Timeboxing mit Meilensteinen)

### Regel (Scope-Grenze)

- Planung und Umsetzung (Was/Wie/Wann), DoD, Abhängigkeiten, Risiken.

---

## 1) Phasenstatus (Ist)

| Phase | Beschreibung | Status |
|------:|--------------|:------:|
| 1 | micro-ROS auf ESP32-S3 | ✅ |
| 2 | Docker-Infrastruktur | ✅ |
| 3 | RPLidar A1 Integration | ✅ |
| 4 | URDF + TF + EKF | ◄── **als Nächstes** |
| 5 | SLAM (slam_toolbox) | ⬜ |
| 6 | Nav2 Autonome Navigation | ⬜ |

---

## 2) Arbeitsprinzip: Vertikale Scheiben (inkrementell, testbar)

Jede Phase liefert ein **lauffähiges Teilsystem** inklusive **DoD + Messchecks**, bevor die nächste Komplexitätsschicht hinzukommt.

**Konsequenz für die Planung:**
Phase \(n\) gilt erst als „done“, wenn:

1) Artefakte im Repo liegen (Code/Configs/Launch)
2) Mess-/Funktionschecks dokumentiert sind
3) Abhängigkeiten für Phase \(n+1\) erfüllt sind

---

## 3) Phasenplan (Arbeitspakete + DoD)

### Phase 1 ✅ – micro-ROS Drivebase (abgeschlossen)

**Lieferumfang:** ESP32-S3 micro-ROS Client, `/cmd_vel`, `/odom_raw`, Failsafe, Heartbeat.
**DoD:** Richtungen getestet, Failsafe greift deterministisch, Topics stabil.

---

### Phase 2 ✅ – Docker-Infrastruktur (abgeschlossen)

**Lieferumfang:** `amr_agent` + `amr_dev`, reproduzierbarer Start, ROS-Tools im Container.
**DoD:** Agent verbindet stabil, Workspace verfügbar.

---

### Phase 3 ✅ – RPLidar A1 (abgeschlossen)

**Lieferumfang:** LiDAR-Treiber, `/scan` stabil (~\(7{,}6\,\mathrm{Hz}\)), `frame_id` definiert.
**DoD:** `/scan` plausibel, wiederholbarer Start.

---

### Phase 4 ◄── als Nächstes – URDF + TF + EKF (robot_localization)

#### Ziel

ROS-Standard-Integration herstellen: **URDF/TF-Baum** + **standardisiertes `/odom`** als Basis für SLAM/Nav2.

#### Abhängigkeiten (erfüllt)

- `/cmd_vel` wirkt, `/odom_raw` vorhanden (Phase 1)
- `/scan` vorhanden (Phase 3)
- Docker/ROS-Tooling steht (Phase 2)

#### Arbeitspakete (geordnet)

1) **URDF-Paket anlegen** (`ros2_ws/src/amr_description`)
   - Frames festlegen: `base_footprint`, `base_link`, `laser_frame` (Projektstandard: einheitlicher Name)
   - Statische Transforms: `base_link -> laser_frame`
2) **robot_state_publisher** integrieren
   - Launch-File + Parametrik
   - RViz-Check: Links/Frames sichtbar
3) **Odom-Bridge bereitstellen** (`ros2_ws/src/amr_bridge`)
   - Node: `/odom_raw (Pose2D)` → `/odom (nav_msgs/Odometry)`
   - TF publizieren: `odom -> base_footprint` (oder `odom -> base_link`, aber konsistent)
4) **EKF (robot_localization) hinzufügen**
   - EKF-Config (Covariances + Inputs)
   - Output: `/odometry/filtered` + TF (je nach Setup)
   - Start/Stop über Launch kontrollierbar

#### DoD Phase 4 (messbar)

- [ ] `robot_state_publisher` läuft, TF enthält `base_footprint`, `base_link`, `laser_frame`
- [ ] TF-Kette ist konsistent (keine Loops): `odom -> base_footprint -> base_link -> laser_frame`
- [ ] `/odom` existiert als `nav_msgs/Odometry` und wird publiziert (Rate dokumentiert)
- [ ] EKF läuft stabil (keine TF-Fehler/Time jumps), `/odometry/filtered` vorhanden
- [ ] RViz: Robot Model + LaserScan im korrekten Frame visualisierbar

---

### Phase 5 – SLAM (slam_toolbox)

#### Ziel

Eine **Karte** erzeugen und speichern (Artefakt: `map.yaml` + `map.pgm`/`map.png`).

#### Abhängigkeiten (müssen aus Phase 4 kommen)

- TF-Baum korrekt (inkl. LiDAR Frame)
- `/scan` stabil
- `/odom` oder `/odometry/filtered` stabil

#### Arbeitspakete (geordnet)

1) slam_toolbox integrieren + Param-Datei anlegen
2) Online SLAM starten (async) + RViz Setup
3) Kartierungsfahrten (Testbereich) durchführen
4) Map speichern und versionieren (inkl. Parameterstand)

#### DoD Phase 5 (messbar)

- [ ] SLAM läuft ohne TF/Time-Fehler
- [ ] Karte ist speicherbar und reproduzierbar ladbar
- [ ] „Loop closure/Drift“ qualitativ dokumentiert (kurzes Protokoll: Bereich, Dauer, Auffälligkeiten)

---

### Phase 6 – Nav2 (autonome Navigation)

#### Ziel

Auf einer gespeicherten Karte Zielpunkte setzen und autonom fahren (Planen + Ausführen + Hindernisbehandlung).

#### Abhängigkeiten (aus Phase 5)

- Karte vorhanden
- Localization-Setup (z. B. AMCL) möglich
- `/scan` + `/tf` + `/odom` stabil

#### Arbeitspakete (geordnet)

1) Nav2-Stack integrieren (bringup) + Basis-Konfig
2) Localization (AMCL) konfigurieren + initial pose workflow
3) Costmaps (global/local) konfigurieren (Footprint/Inflation/Obstacle layer)
4) Planner/Controller Parameter iterativ tunen
5) End-to-end Szenario: Goal setzen → Fahrt → Stop/Recovery

#### DoD Phase 6 (messbar)

- [ ] AMCL stabil (Pose springt nicht, TF konsistent)
- [ ] Goal-Navigation funktioniert in Testbereich (mind. 3 erfolgreiche Läufe)
- [ ] Recovery/Stop-Verhalten ist dokumentiert (z. B. bei blockierter Route)

---

## 4) Zeitplan (aktualisiert ab 2025-12-22)

**Ist:** Phase 1–3 sind abgeschlossen (bis 2025-12-20).
**Plan:** Timeboxing für die Restphasen (6 Wochen).

```

Kalender:  22.12–28.12  29.12–04.01  05.01–11.01  12.01–18.01  19.01–25.01  26.01–01.02
───────────  ───────────  ───────────  ───────────  ───────────  ───────────
Phase 4    ███████████  ███████████  (URDF/TF/Odom/EKF)
Phase 5                            ███████████  ███████████  (SLAM + Map)
Phase 6                                                     ███████████  ███████████  (Nav2)

```

**Pufferregel:** Nav2-Tuning ist typischerweise der größte Zeitfresser → wenn etwas rutscht, zuerst Phase 6 verlängern, nicht Phase 4 kürzen.

---

## 5) Globale Abschlusskriterien (für jede Phase)

- [ ] Artefakte im Repo (Code/Config/Launch) + klare Pfade
- [ ] DoD-Checks ausgeführt und dokumentiert (kurzes Protokoll reicht)
- [ ] Reproduzierbarer Start (Container/Launch)
- [ ] Offene Einschränkungen/Technische Schulden als „Known Limits“ notiert
