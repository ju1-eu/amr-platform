# Projektkonzept: AMR Platform (Differential-Drive)

**Stand:** 2025-12-21 | **Projektstatus:** Phase 1–3 ✅, Phase 4 🔜 als Nächstes

Das Ziel ist ein **Autonomous Mobile Robot (AMR)**, der sich in Innenräumen **selbstständig lokalisieren, kartieren und navigieren** kann – mit klaren Datenverträgen (Topics/TF) und reproduzierbarem Bringup (Docker).

---

## 1. Der Anwendungsfall (Use Case): Mehr als nur Fahren

Das Szenario ist ein kompakter AMR für Werkstatt-/Lager-Umgebungen (Indoor):

1. **Autonomie:** Zielpunkte anfahren (Nav2), später auch Patrol-Routen.
2. **Perzeption:** Hindernisse primär über **2D-LiDAR** (RPLidar A1). Kamera/AI ist optional und später.
3. **Sicherheit:** **Failsafe** bei Kommunikationsverlust (Stop nach Timeout) und klare Betriebsgrenzen (z. B. langsame Geschwindigkeiten beim Mapping).

Der Fokus ist „**Engineering first**“: erst stabile Odom/TF/Scan → dann SLAM → dann Navigation.

---

## 2. Die Entwicklungs-Roadmap: Sechs Phasen zur Navigation

Wir entwickeln in **inkrementellen, testbaren Teilsystemen**. Aktueller Stand:

| Phase | Inhalt                                               | Status |
| ----: | ---------------------------------------------------- | :----: |
|     1 | micro-ROS auf ESP32-S3 (Dual-Core, Control + Safety) |    ✅   |
|     2 | ROS 2 Humble auf Pi 5 via Docker + micro-ROS Agent   |    ✅   |
|     3 | RPLidar A1 → `/scan` stabil                          |    ✅   |
|     4 | **URDF + TF + Odom-Bridge + EKF**           |   🔜   |
|     5 | SLAM (slam_toolbox) → `/map` + `map→odom`            |    ⬜   |
|     6 | Nav2 autonome Navigation auf Karte                   |    ⬜   |

---

## 3. Die Systemarchitektur (Schichtenmodell)

Die Architektur trennt **Echtzeit (Motor/Encoder)** von **ROS-Compute (Perzeption/Planung)**:

```
┌──────────────────────────────────────────────────────────── ┐
│  4. Navigation / Verhalten                                  │
│  Nav2: Goals, Planner/Controller, Recovery                  │
├─────────────────────────────────────────────────────────────┤
│  3. Perzeption                                              │
│  2D LiDAR: /scan → Costmaps / SLAM                          │
├─────────────────────────────────────────────────────────────┤
│  2. Lokalisierung                                           │
│  TF-Kette + Odom (/odom) → später EKF/IMU optional          │
├─────────────────────────────────────────────────────────────┤
│  1. Echtzeit-Drivebase (ESP32-S3)                           │
│  100 Hz Control Task, Encoder-ISR, Odom-Integration, Failsafe│
└─────────────────────────────────────────────────────────────┘
```

**Kommunikation:**

| Verbindung              | Protokoll/Medium                           | Typischer Effekt                                  |
| ----------------------- | ------------------------------------------ | ------------------------------------------------- |
| ESP32 ↔ Pi 5            | USB-CDC, micro-ROS (XRCE-DDS), 921600 Baud | ROS-Topics für `/cmd_vel`, `/odom_raw`, Heartbeat |
| Pi 5 ↔ LiDAR            | USB `/dev/ttyUSB0` (cp210x)                | `/scan` ~7.6 Hz                                   |
| Pi 5 ↔ Hailo (optional) | PCIe (M.2)                                 | spätere AI-Perzeption                             |

---

## 4. Ingenieurs-Realität: Wo es kritisch wird

Drei Punkte entscheiden, ob Phase 4–6 sauber funktionieren:

**(1) TF- und Topic-Verträge sind „hart“**

* Nav2/SLAM benötigen eine stabile Kette: `map → odom → base_* → laser`.
* **Regel:** Genau **eine** Quelle pro TF-Kante (kein Doppel-Publishing).

**(2) Odometrie ist nur so gut wie Mechanik + Encoder-Signal**

* Aktuell A-only Encoder: Richtung wird aus Sollwert/Heuristik abgeleitet → für „präzises Closed-Loop“ begrenzt.
* Konsequenz: konservative Fahrprofile, saubere Kalibrierwerte, später Upgrade (Quadratur oder validierte Richtungslogik).

**(3) Safety ist nicht optional**

* Failsafe bleibt aktiv (Stop nach Timeout), auch wenn später Nav2 `/cmd_vel` liefert.
* Jede Phase endet erst, wenn Smoke-Tests reproduzierbar grün sind.

---

## Projektstand

| Bereich                                           | Status |
| ------------------------------------------------- | :----: |
| Drivebase (Motor/Encoder, 100 Hz, Failsafe)       |    ✅   |
| ROS-Compute via Docker (Humble) + micro-ROS Agent |    ✅   |
| LiDAR `/scan` stabil                              |    ✅   |
| TF/URDF + `/odom`-Bridge (+ EKF)         |   🔜   |
| SLAM `/map` + `map→odom`                          |    ⬜   |
| Nav2 Navigation                                   |    ⬜   |
| Hailo & Kamera                          |    ⬜   |
