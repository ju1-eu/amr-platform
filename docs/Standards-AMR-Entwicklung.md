# Standards für AMR-Entwicklung (klar getrennt nach Standard-Typ)

Ziel: Die Doku soll sauber unterscheiden zwischen **Industrienormen**, **ROS-De-facto-Standards**, **guter Engineering-Praxis** und **Projektstandards**.

---

## 1) ROS-De-facto-Standards (Interoperabilität in ROS 2)

Das sind keine ISO-Normen, aber in ROS praktisch „Pflicht“, weil RViz, Nav2, SLAM und TF darauf aufbauen.

### REP-103: Einheiten & Koordinaten (ROS-Konvention)

- **Einheiten:** SI
  - Distanz in **Meter**
  - Winkel in **Radiant**
  - Zeit in **Sekunden**
- **Koordinatenrahmen:**
  - \(x\): vorwärts, \(y\): links, \(z\): oben
  - Rotation nach **Rechte-Hand-Regel** (CCW positiv)

**Konsequenz:** Alle kinematischen Parameter und Odometrie müssen in SI laufen (z. B. `WHEEL_BASE=0.178` in \(\mathrm{m}\), nicht `178` in \(\mathrm{mm}\)).

### REP-105: Frames/TF-Tree (ROS-Konvention)

Typische Kette:

- `map` → `odom` → `base_link` → `laser_frame`

**Konsequenz:**

- Drivebase/Odom (z. B. ESP32 oder Pi-Node) liefert `odom -> base_*`.
- SLAM/Localization liefert `map -> odom`.
- URDF/Static TF liefert `base_link -> laser_frame`.

---

## 2) Industrienormen (Safety/Compliance-Orientierung)

Das sind externe, veröffentlichte Normen/Standards. Für ein Uni-Projekt meist **nicht zertifiziert**, aber du kannst dich daran **orientieren** (Begriffe: Risikobeurteilung, Sicherheitsfunktionen, Verifikation).

Typische Referenzen (je nach Scope):

- **Maschinensicherheit / Risikobeurteilung:** ISO 12100
- **Safety von fahrerlosen Fahrzeugen/AMR/AGV im industriellen Umfeld:** ISO 3691-4
- **Sicherheitsbezogene Steuerungen:** ISO 13849-1 / IEC 62061
- **Elektrische Ausrüstung von Maschinen:** IEC 60204-1
- **Flotten-/Leitsystem-Kommunikation:** VDA 5050 (branchenweiter Schnittstellenstandard)

**Konsequenz:** Du formulierst Safety-Anforderungen und Tests so, dass sie „normnah“ sind (z. B. Betriebsarten, Stopp-Verhalten, Risikoargumentation), ohne zu behaupten, du seist zertifiziert.

---

## 3) Gute Engineering-Praxis (robust, prüfbar, thesis-tauglich)

Das sind bewährte technische Maßnahmen, die Standards typischerweise unterstützen, aber als „Praxisregel“ im Projekt konkretisiert werden.

### Safety/Robustheit im Embedded-Teil

- **Dead-Man / Heartbeat + Timeout → Motorstopp**
  - Regel: Wenn Kommandos/Heartbeat ausbleiben, dann **lokal** (ESP32) Motoren auf \(0\) setzen.
  - Timeout-Wert ist **projektspezifisch** (z. B. \(500\,\mathrm{ms}\) bis \(2000\,\mathrm{ms}\)), muss begründet und getestet werden.
- **Watchdog / Fail-safe Defaults**
- **Deterministischer Control-Loop** (z. B. \(100\,\mathrm{Hz}\)) und klare Prioritäten (RT-Task vs. Comms)

### Echtzeit-Trennung (Master–Slave)

- **Hard/firm real-time (ESP32):** PWM, Encoder, Not-Halt, Failsafe
- **Soft real-time (Pi 5):** SLAM, Nav2, Logging, UI

---

## 4) Projektstandards (intern, konsistent, wartbar)

Das sind Regeln, die ihr im Repo festlegt, damit Team/du selbst in 3 Monaten noch schnell reinkommt.

Beispiele:

- **Doku-Template pro Phase** (Frontmatter, DoD, Smoke-Tests, bekannte Einschränkungen)
- **Definition of Done (DoD)** + Messpunkte (`ros2 topic hz`, RViz-Check, Reconnect-Test)
- **Topic-Namenskonventionen** (ROS-üblich, aber die exakten Prefixe/Namensräume sind Projektentscheidung)
- **Non-Blocking Code-Regeln** (kein `delay()` im Control-Pfad; Timer/Tasks)
- **Versionierung / Changelog / Verzeichnisstruktur**

---

## Minimal-Set (Daumenregel für „funktioniert später mit Nav2/SLAM“)

Wenn du nur 4 Dinge „hart“ festzurren willst:

1. **REP-103/105 einhalten** (Einheiten + TF-Tree)
2. **Failsafe lokal auf der Drivebase** (Timeout → Stopp)
3. **Nicht-blockierendes Timing** (Control & Comms getrennt)
4. **DoD + Smoke-Tests pro Phase** (messbar, reproduzierbar)
