# Theoretisches Konzept: Die vier Ebenen der Autonomie

**Stand:** 2025-12-20 | **Projekt:** AMR Platform | **Status:** Phase 1–3 ✅, Phase 4 🔜 (URDF/TF/Odom-Bridge/EKF)

Autonomie entsteht nicht „in einer Kette“, sondern durch parallel laufende Ebenen mit klaren Datenverträgen (Topics/TF) und definierten Sicherheitsgrenzen.

---

## Ebene 1: Reflex und Basis-Sicherheit (Low-Level Control)

**Zweck:** Der Roboter bleibt beherrschbar – auch bei Kommunikationsverlust.

**Ist-Stand (Phase 1):**
Der **ESP32-S3** ist der Echtzeit-Controller:

* Motoransteuerung + Encoder-ISR in **100 Hz** Control-Loop (Core 0)
* Odometrie-Integration intern, Publishing über micro-ROS
* **Failsafe:** Stop, wenn länger als **$2000,\mathrm{ms}$** kein gültiges Kommando ankommt

**Wichtige Klarstellung:**
Ein „LiDAR-Notstopp < $0{,}30,\mathrm{m}$“ läuft aktuell **nicht** auf dem ESP32, weil der LiDAR am Pi hängt. Die Reflex-Ebene ist derzeit primär **Comms-Failsafe** + Motor-Stop-Logik.

**Grenze:** Open-Loop/Feedforward (PID aktuell deaktiviert) → keine echte Drehzahlregelung, Richtung bei A-only Encodern heuristisch.

---

## Ebene 2: Gedächtnis (Frames, Odometrie-Vertrag, Lokalisierung)

**Zweck:** „Wo bin ich?“ – konsistente Pose über TF.

**Ist-Stand (Phase 3):**

* `/scan` ist stabil (RPLidar A1, ca. **$7{,}6,\mathrm{Hz}$**, frame_id: `laser`)
* `/odom_raw` existiert (Pose2D) – aber **noch kein sauberer `/odom` (nav_msgs/Odometry)** + konsistente TF-Kette für SLAM

**Nächster Schritt (Phase 4):**
Wir bauen den TF-/Odom-Kontrakt, damit SLAM/Nav2 überhaupt zuverlässig arbeiten kann:

* Statische Frames aus URDF: `base_footprint → base_link → laser`
* Odom-Bridge: `/odom_raw` → `/odom` + TF `odom → base_footprint`
* Optional danach: EKF (robot_localization), sobald Sensorbasis sinnvoll ist (später IMU)

**Grenze:** Ohne saubere TF-Kette werden Map/Costmaps „inactive“ oder „driften“.

---

## Ebene 3: Strategie (Navigation / Nav2)

**Zweck:** „Wie komme ich von A nach B?“ – global planen, lokal ausweichen.

**Geplante Umsetzung (Phase 6):**

* Nav2 erzeugt `/cmd_vel` aus Karte + Costmaps
* Der Roboter folgt `/cmd_vel` über micro-ROS (Pi → ESP32)

**Voraussetzungen (müssen vorher stabil sein):**

* TF: `map → odom → base_* → laser`
* Topics: `/scan`, `/odom`, `/tf`, `/tf_static`
* Karte: `/map` (aus SLAM oder Map-Server)

**Grenze:** Ohne korrekt kalibrierte Odometrie/Frames wird Nav2 instabil (Fehlpose, falsche Hindernisse, falscher Drehsinn).

---

## Ebene 4: Kognition (Semantik / Vision / AI)

**Zweck:** „Was ist das?“ – Objekte und Situationen klassifizieren (Mensch/Werkzeug/Hindernis).

**Projektstand:** Optional/später. (Hailo-8L + Kamera sind aktuell nicht Teil des kritischen Pfades für Phase 4–6.)

**Grenze:** Semantik bringt erst Nutzen, wenn Ebene 2–3 robust laufen (sonst reagiert das System auf falsche Positionen).

---

## Gesamtbild als Datenfluss

```
Ebene 4 (später):   Vision/AI (Hailo/Kamera) ──► Semantik/Behavior
Ebene 3 (später):   Nav2 ──► /cmd_vel
Ebene 2 (Phase 4):  TF + /odom + (später EKF) ──► SLAM/Map
Ebene 1 (Phase 1):  ESP32 Control + Failsafe ──► Motoren
```

**Kommunikation (Ist):**

| Verbindung         | Technik                           | Wert / Hinweis                  |
| ------------------ | --------------------------------- | ------------------------------- |
| ESP32 ↔ Pi 5       | micro-ROS (XRCE-DDS) über USB-CDC | $921600,\mathrm{baud}$          |
| Pi 5 ↔ LiDAR       | USB-Serial `/dev/ttyUSB0`         | `/scan` ca. $7{,}6,\mathrm{Hz}$ |
| Pi 5 ↔ (AI später) | PCIe (M.2)                        | optional                        |

---

## Konsequenz für „Phase 4 als Nächstes“

Phase 4 ist die **Scharnierstelle**: Sie macht aus „Sensoren liefern Daten“ ein **konsistentes Robot-Modell** (URDF/TF) plus **nutzbare Odometrie** (`/odom`). Erst danach sind SLAM (Phase 5) und Nav2 (Phase 6) technisch sauber aufsetzbar.
