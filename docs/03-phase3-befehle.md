# Phase 3: RPLidar A1 – Befehlsreferenz

**Status:** ✅ Abgeschlossen (2025-12-20)

---

## Quick Start

### 1. Container starten (nach Pi Reboot)

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose ps
```

### 2. RPLidar starten (Terminal 1)

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### 3. Smoke-Tests (zweites Terminal)

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

# Topics prüfen
ros2 topic list

# Frequenz prüfen (~7.6 Hz)
ros2 topic hz /scan

# Daten prüfen
ros2 topic echo /scan --once
```

---

## Erwartete Topics

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/odom_raw
/scan
/parameter_events
/rosout
```

---

## Scan-Daten Referenz

| Feld | Erwartung |
|------|-----------|
| `frame_id` | `laser` |
| `angle_min` | -3.14 rad |
| `angle_max` | +3.14 rad |
| `range_min` | 0.05 m |
| `range_max` | 12.0 m |
| Frequenz | ~7.6 Hz |

---

## Troubleshooting

### "Operation timeout"

```bash
# Port blockiert? Prüfen:
sudo fuser /dev/ttyUSB0

# Falls belegt, Prozesse beenden:
sudo kill <PID>
```

### Device nicht vorhanden

```bash
ls -l /dev/ttyUSB*
dmesg | grep -i usb | tail -10
```

### Container sieht Device nicht

docker-compose.yml prüfen:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
```

---

## Hardware-Info

| Sensor | Wert |
|--------|------|
| Modell | RPLidar A1 |
| S/N | 74A5FA89C7E19EC8BCE499F0FF725670 |
| Firmware | 1.29 |
| Hardware Rev | 7 |
| Scan Mode | Sensitivity |
| Sample Rate | 8 kHz |
| Scan Frequency | ~7.6 Hz |
