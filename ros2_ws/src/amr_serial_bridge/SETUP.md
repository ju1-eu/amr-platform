# Serial Bridge auf Raspberry Pi einrichten

> **Ziel:** ROS 2 `/cmd_vel` → Serial → ESP32 → Motoren
> **Voraussetzung:** ESP32 mit Serial-Bridge Firmware geflasht

---

## Schritt 1: Package auf Pi kopieren

### Option A: Via Git (empfohlen)

```bash
# Auf Mac: Committen
cd /Users/jan/daten/start/IoT/AMR/amr-platform
git add ros2_ws/src/amr_serial_bridge/
git commit -m "feat: ROS 2 Serial Bridge Package"
git push origin main

# Auf Pi: Pullen
ssh pi@rover
cd ~/amr-platform
git pull origin main
```

### Option B: Via SCP

```bash
# Von Mac aus
scp -r ros2_ws/src/amr_serial_bridge pi@rover:~/amr-platform/ros2_ws/src/
```

---

## Schritt 2: pyserial installieren

```bash
ssh pi@rover

# Im Docker-Container oder auf Host
pip3 install pyserial --break-system-packages
```

---

## Schritt 3: Package bauen

```bash
cd ~/amr-platform/ros2_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Bauen
colcon build --packages-select amr_serial_bridge

# Sourcing
source install/setup.bash
```

---

## Schritt 4: ESP32 verbinden

```bash
# USB-Verbindung prüfen
ls -la /dev/ttyACM*

# Erwartung: /dev/ttyACM0
```

---

## Schritt 5: Bridge starten

### Direkt starten

```bash
ros2 run amr_serial_bridge serial_bridge
```

### Via Launch-File

```bash
ros2 launch amr_serial_bridge serial_bridge.launch.py
```

### Mit anderem Port

```bash
ros2 launch amr_serial_bridge serial_bridge.launch.py serial_port:=/dev/ttyUSB0
```

---

## Schritt 6: Teleop testen

In einem zweiten Terminal:

```bash
# Teleop installieren (falls nicht vorhanden)
sudo apt install ros-jazzy-teleop-twist-keyboard

# Starten
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:**

| Taste | Aktion |
|-------|--------|
| `i` | Vorwärts |
| `,` | Rückwärts |
| `j` | Links drehen |
| `l` | Rechts drehen |
| `k` | Stopp |
| `q` | Geschwindigkeit + |
| `z` | Geschwindigkeit - |

---

## Schritt 7: Docker-Integration (optional)

### docker-compose.yml erweitern

```yaml
services:
  serial_bridge:
    image: ros:jazzy-ros-base
    container_name: amr_serial_bridge
    network_mode: host
    privileged: true
    volumes:
      - /dev/ttyACM0:/dev/ttyACM0
      - ../ros2_ws:/ros2_ws:ro
      - /dev/shm:/dev/shm
    environment:
      - ROS_DOMAIN_ID=0
    working_dir: /ros2_ws
    command: >
      bash -c "
        pip3 install pyserial &&
        source /opt/ros/jazzy/setup.bash &&
        colcon build --packages-select amr_serial_bridge &&
        source install/setup.bash &&
        ros2 launch amr_serial_bridge serial_bridge.launch.py
      "
    restart: unless-stopped
```

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d serial_bridge
docker compose logs -f serial_bridge
```

---

## Debugging

### Node läuft?

```bash
ros2 node list
# Erwartung: /serial_bridge
```

### Topic aktiv?

```bash
ros2 topic list
ros2 topic info /cmd_vel
# Erwartung: Subscription count: 1
```

### Manuell cmd_vel senden

```bash
# Vorwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Drehen
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Stopp
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### Serial Monitor auf Pi

```bash
# Direkt (ohne ROS)
screen /dev/ttyACM0 115200

# Oder mit minicom
minicom -D /dev/ttyACM0 -b 115200
```

---

## Checkliste

- [ ] ESP32 per USB am Pi angeschlossen
- [ ] `/dev/ttyACM0` existiert
- [ ] pyserial installiert
- [ ] Package gebaut (`colcon build`)
- [ ] Bridge gestartet (zeigt "ESP32 bereit")
- [ ] Teleop: Roboter fährt vorwärts/rückwärts
- [ ] Teleop: Roboter dreht links/rechts
- [ ] Failsafe: Teleop beenden → Motoren stoppen

---

*Stand: 2025-12-12 | Version: 0.3.0*
