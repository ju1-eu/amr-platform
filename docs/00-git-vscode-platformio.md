# Git, VS Code & PlatformIO

**Stand:** 2025-12-20 | **Firmware:** v3.2.0 | **Architektur:** micro-ROS (Dual-Core, USB-CDC)

Ziel dieser Datei: reproduzierbarer Workflow, um **Firmware zu bauen/flashen/debuggen** und Änderungen **sauber zu versionieren**.

---

## Konzepte & Definitionen

### DIY-Projekt (Do It Yourself)

Du verantwortest Planung, Aufbau, Code und Doku selbst (statt „fertig kaufen“).

- **Typisch im AMR:** Verdrahtung + Firmware + ROS-Stack + Troubleshooting
- **Ziel:** Lernen, Kontrolle, Reproduzierbarkeit

### IoT (Internet of Things) im AMR-Kontext

Ein Gerät wird „IoT“, sobald es vernetzt Telemetrie/Kommandos austauscht.

- **Hier konkret:** ESP32-S3 als micro-ROS Client (USB-CDC) ↔ micro-ROS Agent auf dem Pi (Docker) ↔ ROS 2 Topics (`/cmd_vel`, `/odom_raw`, …)

### STEAM (kurz)

- **Engineering:** Mechanik, Verkabelung, Power
- **Technology:** ROS 2, Docker, micro-ROS
- **Mathematics:** Diff-Drive Kinematik, Odometrie, Regelung/Feedforward
- **Art:** Doku/Design (nachvollziehbar, testbar)

### Lizenz (MIT)

- Freie Nutzung/Änderung/Weitergabe, Urheberhinweis bleibt, keine Garantie.

---

## Git-Versionierung

### Regel

Git ist hier nicht „Backup“, sondern **Engineering-Protokoll**: *Welche Änderung führte zu welchem Verhalten (Test/DoD)?*

### Setup (einmalig, wenn noch nicht gesetzt)

```bash
git config --global user.name "Jan Unger"
git config --global user.email "<deine-mail>"
```

### Repo holen / aktualisieren

```bash
# Clone (einmalig)
git clone git@github.com:unger-robotics/amr-platform.git
cd amr-platform

# Update (regelmäßig)
git pull --rebase origin main
```

### Standard-Arbeitszyklus („The Loop“)

```bash
git status
git add .
git commit -m "feat: feedforward-only control (PID disabled), failsafe 2000ms"
git push origin main
```

### Commit-Konvention (empfohlen)

| Präfix      | Bedeutung                    |
| ----------- | ---------------------------- |
| `feat:`     | neue Funktion                |
| `fix:`      | Bugfix                       |
| `docs:`     | Doku                         |
| `refactor:` | Umbau ohne Funktionsänderung |
| `test:`     | Tests                        |
| `chore:`    | Build/Deps/Tooling           |

Beispiele (passend zu v3.2.0):

```bash
git commit -m "feat: dual-core separation control/comm (100Hz + micro-ROS spin)"
git commit -m "fix: swap PWM channels A<->B for correct direction"
git commit -m "docs: update phase docs to v3.2.0 (Humble + 921600)"
```

---

## VS Code (zwei Umgebungen)

### 1) MCU/Firmware (ESP32-S3)

- **Extension:** PlatformIO IDE
- **Aufgabe:** Build/Upload/Monitor der Firmware im Ordner `firmware/`

### 2) Pi/ROS (Raspberry Pi 5)

- **Extension:** Remote - SSH
- **Aufgabe:** Docker/ROS-Stack starten, `ros2` Tools, Logs, TF/Topics prüfen

Empfohlene Extensions:

| Extension      | Zweck                      |
| -------------- | -------------------------- |
| PlatformIO IDE | ESP32 Build/Flash/Monitor  |
| Remote - SSH   | Arbeiten direkt auf dem Pi |
| C/C++          | IntelliSense Firmware      |
| Python         | ROS Tools/Nodes/Skripte    |
| Docker         | Compose/Container          |
| GitLens        | History/Blame/Review       |
| XML (Red Hat)  | URDF/Xacro (ab Phase 4)    |

---

## PlatformIO (Firmware)

### Regel

Alle `pio`-Befehle laufen im Ordner `firmware/`. Aktives Environment: **`seeed_xiao_esp32s3`**.

### platformio.ini (Ist-Stand)

- Environment: `[env:seeed_xiao_esp32s3]`
- micro-ROS: `micro_ros_platformio`, Transport: `serial`
- Upload: `upload_speed = 921600`

**Wichtig (Konsistenz):** In `main.cpp` läuft `Serial.begin(921600)`. Daher sollte auch der Monitor auf **921600** stehen.

Empfohlene Anpassung in `firmware/platformio.ini`:

```ini
monitor_speed = 921600
```

### Build / Upload / Monitor (Terminal)

```bash
cd firmware

# Build
pio run -e seeed_xiao_esp32s3

# Upload
pio run -e seeed_xiao_esp32s3 -t upload

# Serial Monitor (explizit)
pio device monitor -b 921600
```

### PlatformIO in VS Code (GUI)

1. Unten in der Statusleiste Environment wählen: `seeed_xiao_esp32s3`
2. Aktionen:

   - **Build** (✓)
   - **Upload** (→)
   - **Monitor** (Stecker)

---

## Projektstruktur (relevant für Tooling)

```text
amr-platform/
├── firmware/                 # ESP32-S3 Firmware (micro-ROS, Dual-Core)
│   ├── include/config.h
│   ├── src/main.cpp
│   ├── platformio.ini
│   └── extra_script.py
├── docker/                   # Pi 5 Docker Stack (Agent + Dev)
│   ├── docker-compose.yml
│   └── Dockerfile
├── ros2_ws/                  # ROS 2 Workspace (Bridge/Description/Bringup)
│   └── src/
├── docs/                     # System- + Entwicklerdoku, Phasen
└── scripts/                  # Helper (Deploy, Setup, Templates)
```

---

## Schnelle Referenz

### Firmware flashen (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform/firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

### Pi: Repo aktualisieren + Docker prüfen

```bash
ssh pi@rover
cd ~/amr-platform
git pull --rebase

cd docker
docker compose ps
docker compose logs microros_agent --tail 20
```

---

*Aktualisiert für micro-ROS Architektur (v3.2.0, 921600 Baud, Dual-Core).*
