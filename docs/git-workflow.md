# Git-Workflow: Mac ↔ GitHub ↔ Raspberry Pi

> **Prinzip:** Mac = Entwicklung, GitHub = Single Source of Truth, Pi = Deployment
> **Stand:** 2025-12-12 | **Firmware:** v0.3.0-serial

---

## 1. Übersicht

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│      Mac        │         │     GitHub      │         │   Raspberry Pi  │
│   (Entwicklung) │ ──push──▶│  amr-platform   │◀──pull──│   (Runtime)     │
│                 │◀──pull── │ unger-robotics  │         │                 │
└─────────────────┘         └─────────────────┘         └─────────────────┘
     │                                                        │
     │ firmware_serial/                                       │ ~/amr-platform/
     │ ros2_ws/src/amr_serial_bridge/                         │ docker/
     │ docker/                                                │ ros2_ws/
     │ docs/                                                  │
     └────────────────────────────────────────────────────────┘
```

---

## 2. Initiales Setup

### 2.1 Auf dem Mac (Development)

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform

# 1. Identität setzen (WICHTIG!)
git config --global user.name "Jan Unger"
git config --global user.email "unger.robotics@gmail.com"

# 2. Falls noch nicht initialisiert
git init

# 3. Remote hinzufügen (SSH bevorzugt)
git remote add origin git@github.com:unger-robotics/amr-platform.git

# 4. .gitignore erstellen (WICHTIG!)
cat > .gitignore << 'EOF'
# === Build-Artefakte ===
.pio/
.vscode/
*.o
*.elf
*.bin
*.map

# === Python ===
__pycache__/
*.pyc
.pytest_cache/

# === ROS 2 Build ===
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/

# === Docker ===
*.log

# === Hailo (große Dateien!) ===
*.hef
hailo_models/

# === macOS ===
.DS_Store
*.swp

# === Secrets (niemals committen!) ===
*.key
*.pem
secrets/

# === IDE ===
*.code-workspace
.idea/
EOF

# 5. Ersten Commit & Upload
git add .
git commit -m "feat: Serial-Bridge Firmware v0.3.0"
git branch -M main
git push -u origin main
```

### 2.2 Auf dem Raspberry Pi (Runtime)

```bash
# 1. Identität setzen (Damit Hotfixes zugeordnet werden)
git config --global user.name "Jan Unger"
git config --global user.email "unger.robotics@gmail.com"

# 2. Backup der aktuellen Konfiguration
cp -r ~/amr-platform ~/amr_backup_$(date +%Y%m%d)

# 3. Repository klonen
cd ~
git clone git@github.com:unger-robotics/amr-platform.git

# 4. Symlinks für Kompatibilität erstellen (optional)
ln -s ~/amr-platform ~/amr
```

---

## 3. Täglicher Workflow

### 3.1 Auf dem Mac entwickeln

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform

# 1. Vor der Arbeit: Aktuellen Stand holen
git pull origin main

# 2. Arbeiten (Code schreiben, testen)
# ... Änderungen an firmware_serial/src/main.cpp ...

# 3. Status prüfen
git status

# 4. Änderungen stagen
git add firmware_serial/src/main.cpp
# Oder alles: git add .

# 5. Commit mit aussagekräftiger Nachricht
git commit -m "fix: Deadzone-Kompensation für kleine Geschwindigkeiten"

# 6. Hochladen
git push origin main
```

### 3.2 Auf dem Pi deployen

```bash
cd ~/amr-platform

# 1. Änderungen holen
git pull origin main

# 2. Docker neu starten
cd docker
docker compose down
docker compose up -d

# 3. Logs prüfen
docker compose logs -f serial_bridge
```

---

## 4. Commit-Konventionen

| Präfix | Verwendung | Beispiel |
|--------|------------|----------|
| `feat:` | Neue Funktion | `feat: Encoder-ISR für Phase 2` |
| `fix:` | Bugfix | `fix: Failsafe-Timeout auf 500ms` |
| `docs:` | Dokumentation | `docs: README aktualisiert` |
| `refactor:` | Code-Umbau | `refactor: HAL in separate Datei` |
| `test:` | Tests | `test: Motor-Kalibrierung Sketch` |
| `chore:` | Build/Tooling | `chore: PlatformIO auf 6.1.15` |

**Beispiele für gute Commits:**

```bash
git commit -m "feat(firmware): Serial-Bridge Protokoll V:x,W:y"
git commit -m "fix(docker): serial_bridge Container Device-Mapping"
git commit -m "docs: Phase 1 abgeschlossen"
```

---

## 5. Branching-Strategie (optional)

Für ein Bachelor-Projekt reicht oft `main`. Falls du experimentieren willst:

```bash
# Neuen Feature-Branch erstellen
git checkout -b feature/encoder-odometry

# Arbeiten, committen...
git add .
git commit -m "feat: Odometrie-Publisher implementiert"

# Branch hochladen
git push -u origin feature/encoder-odometry

# Zurück zu main und mergen
git checkout main
git merge feature/encoder-odometry
git push origin main

# Branch löschen (lokal + remote)
git branch -d feature/encoder-odometry
git push origin --delete feature/encoder-odometry
```

---

## 6. Synchronisation Mac ↔ Pi

### Schneller Weg: Nur Firmware (ohne Git)

```bash
# Von Mac aus (rsync über SSH)
rsync -avz --progress \
    /Users/jan/daten/start/IoT/AMR/amr-platform/firmware_serial/ \
    pi@rover:~/amr-platform/firmware_serial/
```

### Empfohlener Weg: Über GitHub

```
Mac: git push  →  GitHub (unger-robotics)  →  Pi: git pull
```

Das ist sauberer, dokumentiert alle Änderungen und sichert den Code extern.

### Ein-Zeilen-Deploy (Mac → Pi)

```bash
# Committen + Pushen + Pi updaten + Container neustarten
git add . && git commit -m "fix: Beschreibung" && git push && \
ssh pi@rover "cd ~/amr-platform && git pull && cd docker && docker compose up -d"
```

---

## 7. Häufige Szenarien

### "Ich habe auf dem Pi etwas getestet und will es behalten"

```bash
# Auf dem Pi
cd ~/amr-platform
git add docker/docker-compose.yml
git commit -m "tune: Serial-Bridge Timeout angepasst"
git push origin main

# Auf dem Mac (später)
git pull origin main
```

### "Ich habe lokale Änderungen, will aber den neuesten Stand"

```bash
# Option A: Änderungen temporär speichern (Stash)
git stash
git pull origin main
git stash pop  # Änderungen wieder anwenden

# Option B: Änderungen verwerfen (Hard Reset - VORSICHT!)
git checkout -- .
git pull origin main
```

### "Merge-Konflikt!"

```bash
# Git zeigt an: CONFLICT in firmware_serial/src/main.cpp

# 1. Datei öffnen, Konflikte manuell lösen
#    Suche nach: <<<<<<< HEAD ... ======= ... >>>>>>>

# 2. Gelöste Datei stagen
git add firmware_serial/src/main.cpp

# 3. Merge abschließen
git commit -m "merge: Konflikt in main.cpp gelöst"
```

---

## 8. Nützliche Aliase (optional)

In `~/.gitconfig` hinzufügen:

```ini
[alias]
    st = status
    co = checkout
    br = branch
    ci = commit
    lg = log --oneline --graph --decorate --all
    sync = !git pull && git push
```

---

## 9. Backup-Strategie

### Lokales Backup (Mac)

```bash
# Bare-Repository als Backup
git clone --bare . ~/Backups/amr-platform.git
```

### Automatisches Backup (Pi)

```bash
# Cron-Job für tägliches Backup
crontab -e

# Eintrag hinzufügen:
0 3 * * * cd ~/amr-platform && git add -A && git commit -m "auto: Daily backup" && git push origin main 2>/dev/null || true
```

---

## 10. Projekt-Struktur (aktuell)

```
amr-platform/
├── firmware/                # micro-ROS (nicht verwendet)
├── firmware_serial/         # Serial-Bridge Firmware ✅
│   ├── platformio.ini
│   ├── include/config.h
│   └── src/main.cpp
├── firmware_test/           # Hardware-Test
├── ros2_ws/
│   └── src/
│       ├── amr_description/
│       ├── amr_bringup/
│       └── amr_serial_bridge/  # ROS 2 Serial Bridge ✅
├── docker/
│   ├── docker-compose.yml   # perception + serial_bridge
│   └── perception/
├── docs/
├── scripts/
│   └── deploy.sh
├── LICENSE
└── README.md
```

---

## 11. Checkliste: Git richtig nutzen

| Regel | Warum |
|-------|-------|
| **Vor der Arbeit: `git pull`** | Konflikte vermeiden |
| **Kleine Commits** | Leichter nachvollziehbar |
| **Aussagekräftige Nachrichten** | Hilft beim Debuggen |
| **Secrets nie committen** | Sicherheit (keine Passwörter!) |
| **Identität prüfen** | `git config user.email` checken |
| **firmware_serial/ für ESP32** | Nicht firmware/ (micro-ROS defekt) |

---

## Zusammenfassung: Der goldene Pfad

```bash
# Jeden Tag, jedes Mal:

# 1. Updaten
git pull origin main

# 2. Arbeiten
# ... code, test, debug ...

# 3. Committen
git add .
git commit -m "feat: Beschreibung der Änderung"

# 4. Synchronisieren
git push origin main

# 5. Auf Pi deployen
ssh pi@rover "cd ~/amr-platform && git pull && cd docker && docker compose up -d"
```

---

## Quick Reference Card

| Aktion | Mac | Pi |
|--------|-----|-----|
| **ESP32 flashen** | `cd firmware_serial && pio run -t upload` | — |
| **Serial Monitor** | `pio device monitor` | `screen /dev/ttyACM0 115200` |
| **Docker starten** | — | `docker compose up -d` |
| **Logs anzeigen** | — | `docker compose logs -f serial_bridge` |
| **Git sync** | `git pull && git push` | `git pull` |
| **Teleop testen** | — | `docker exec -it amr_perception ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

---

*Aktualisiert: 2025-12-12 | Projekt: amr-platform | Maintainer: unger-robotics*
