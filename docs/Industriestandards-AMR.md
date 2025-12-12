# Standards für AMR-Entwicklung

Für dein Master-Projekt und die Thesis ist es entscheidend, nicht nur „irgendwie“ zu programmieren, sondern sich an etablierte Industriestandards zu halten. Das unterscheidet eine Bastelarbeit von einer Ingenieursleistung.

Für deinen AMR (Raspberry Pi 5 + ESP32 + ROS 2) sind folgende Standards **verbindlich**, wenn du willst, dass Navigation (Nav2) und SLAM später reibungslos funktionieren.

### 1. Der ROS-Standard: REPs (ROS Enhancement Proposals)

Das sind die „Gesetze“ der ROS-Welt. Wenn du diese verletzt, wird der Roboter in der Simulation (RViz) herumspringen oder gegen Wände fahren.

* **REP-103: Einheiten & Koordinaten**
  * **Einheiten:** Wir rechnen **immer** in SI-Einheiten.
    * Distanz: **Meter** (nicht Millimeter! Deswegen `0.178` statt `178` in deiner Config).
    * Winkel: **Radiant** (nicht Grad!).
    * Zeit: **Sekunden**.
  * **Koordinatensystem:**
    * **X:** Nach vorne (Vorwärtsfahrt).
    * **Y:** Nach links.
    * **Z:** Nach oben.
    * **Rotation:** Rechte-Hand-Regel (Gegen den Uhrzeigersinn ist positiv).
  * *Konsequenz für dich:* Dein ESP32 muss die Encoder-Ticks in Meter umrechnen, bevor er sie an den Raspberry Pi sendet.

* **REP-105: Koordinaten-Frames (Der TF-Tree)**
    Das ist das Rückgrat der Navigation. Dein System muss folgende Kette einhalten:
  * `map` (Globale Karte, Fixpunkt)
  * `odom` (Startpunkt der Fahrt, driftet mit der Zeit)
  * `base_link` (Der Roboter selbst, meistens die Mitte der Radachse)
  * `laser_frame` (Wo der LiDAR montiert ist)
  * *Konsequenz für dich:* Der ESP32 publiziert die Transformation `odom -> base_link`. Der Raspberry Pi (SLAM) publiziert `map -> odom`.

### 2. Sicherheits-Standards (Safety)

Bei einem autonomen Roboter darf die Software niemals „einschlafen“, während der Motor noch läuft.

* **Heartbeat / Dead Man's Switch:**
    Der ESP32 darf nicht blind fahren. Er muss regelmäßig (z.B. alle 100ms) eine Nachricht vom Master (Pi 5) erhalten.
  * *Regel:* Bleibt die Nachricht für > 500ms aus (z.B. WLAN weg, ROS-Node abgestürzt), müssen die Motoren **sofort stoppen** (PWM = 0).
  * *Implementierung:* Das bauen wir direkt in die Firmware ein.

### 3. Architektur-Standard: Hybrid Master-Slave

Du hast dies bereits in deiner Thesis definiert, aber hier ist die technische Umsetzung:

* **Echtzeit-Trennung:**
  * **Hard Real-Time (ESP32):** Alles, was physikalische Schäden verursachen kann oder präzises Timing braucht (Motor-PWM, Encoder-Zählen, Not-Stopp).
  * **Soft Real-Time (Pi 5):** Alles, was „denkt“ (Pfadplanung, Bilderkennung). Wenn der Pi 5 mal 100ms hängt, darf der Roboter nicht crashen (siehe Heartbeat).

### 4. Code-Qualität (Best Practices)

Für die Benotung der Thesis wichtig:

* **Non-Blocking Code (ESP32):**
    Verwende niemals `delay()` in deiner Hauptschleife. Das blockiert die Kommunikation. Wir nutzen `millis()`-Timer oder FreeRTOS Tasks.
* **Topic Naming:**
    Halte dich an ROS 2 Konventionen:
  * `/cmd_vel` für Steuerbefehle.
  * `/odom` für Odometrie.
  * `/scan` für LiDAR-Daten.
  * `/diagnostics` für Status (Batterie, Fehler).

---

### Zusammenfassung für dein Projekt

Wenn wir jetzt den Code schreiben, halten wir uns strikt an:

1. **SI-Einheiten** (Meter, Radiant).
2. **REP-105 Frames** (`odom` -> `base_link`).
3. **Failsafe-Mechanismus** (Kein Heartbeat = Stopp).
4. **Non-Blocking I/O** (keine `delay()` im Loop).
