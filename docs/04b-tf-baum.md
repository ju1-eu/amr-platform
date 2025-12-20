---
title: "Phase 4 – TF-Baum für SLAM"
status: "active"
updated: "2025-12-20"
version: "3.0"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker) ✅"
  - "Phase 3 (RPLidar A1) ✅"
next:
  - "Phase 5 (SLAM)"
---

# Phase 4: TF-Baum für SLAM

## Zielbild & Definition of Done

### Zielbild (TF-Baum)

```
odom → base_footprint → base_link → laser
```

### DoD (prüfbar)

- [ ] `odom_converter.py` publiziert `/odom` + TF `odom → base_footprint`
- [ ] `robot_state_publisher` publiziert statische TFs aus URDF
- [ ] `ros2 run tf2_ros tf2_echo odom laser` funktioniert
- [ ] RViz2 zeigt LaserScan korrekt relativ zum Roboter

---

## 1) Übersicht

### 1.1 Was wir haben

| Topic | Typ | Quelle |
|-------|-----|--------|
| `/odom_raw` | `geometry_msgs/Pose2D` | ESP32 (micro-ROS) |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar A1 |
| `/cmd_vel` | `geometry_msgs/Twist` | Steuerung |

### 1.2 Was wir brauchen

| Topic/TF | Typ | Quelle |
|----------|-----|--------|
| `/odom` | `nav_msgs/Odometry` | odom_converter.py |
| TF: `odom → base_footprint` | Dynamisch | odom_converter.py |
| TF: `base_footprint → base_link` | Statisch | robot_state_publisher |
| TF: `base_link → laser` | Statisch | robot_state_publisher |

### 1.3 Warum kein EKF?

EKF (robot_localization) macht ohne IMU wenig Sinn – es würde nur die Encoder-Odometrie durchreichen. Wenn später IMU (MPU6050) hinzukommt, können wir EKF nachrüsten.

---

## 2) Komponente 1: odom_converter.py

### 2.1 Funktion

Konvertiert `/odom_raw` (Pose2D) → `/odom` (Odometry) + TF

### 2.2 Code

Datei: `ros2_ws/src/amr_bridge/amr_bridge/odom_converter.py`

```python
#!/usr/bin/env python3
"""
odom_converter.py - Konvertiert /odom_raw (Pose2D) zu /odom (Odometry) + TF
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Pose2D,
            '/odom_raw',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Vorherige Werte für Velocity-Berechnung
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.prev_time = self.get_clock().now()
        
        self.get_logger().info('odom_converter started')

    def odom_callback(self, msg: Pose2D):
        current_time = self.get_clock().now()
        
        # Zeit-Delta berechnen
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.05  # Fallback
        
        # Velocity berechnen (Differenz)
        vx = (msg.x - self.prev_x) / dt
        vy = (msg.y - self.prev_y) / dt
        vtheta = (msg.theta - self.prev_theta) / dt
        
        # Odometry Message erstellen
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (Quaternion aus theta)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)
        
        # Velocity (im body frame)
        odom.twist.twist.linear.x = vx * math.cos(msg.theta) + vy * math.sin(msg.theta)
        odom.twist.twist.linear.y = -vx * math.sin(msg.theta) + vy * math.cos(msg.theta)
        odom.twist.twist.angular.z = vtheta
        
        # Covariance (einfache Werte)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # theta
        
        self.odom_pub.publish(odom)
        
        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Werte speichern
        self.prev_x = msg.x
        self.prev_y = msg.y
        self.prev_theta = msg.theta
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.3 Package-Struktur

```
ros2_ws/src/amr_bridge/
├── amr_bridge/
│   ├── __init__.py
│   └── odom_converter.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.4 setup.py

```python
from setuptools import setup

package_name = 'amr_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Unger',
    maintainer_email='jan@unger.de',
    description='AMR Bridge Nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_converter = amr_bridge.odom_converter:main',
        ],
    },
)
```

---

## 3) Komponente 2: URDF

### 3.1 Messungen (TODO: anpassen!)

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `base_link_z` | 0.05 m | Höhe Chassis über Boden |
| `laser_x` | 0.10 m | LiDAR vor base_link |
| `laser_y` | 0.00 m | LiDAR mittig |
| `laser_z` | 0.10 m | LiDAR über base_link |

### 3.2 URDF-Datei

Datei: `ros2_ws/src/amr_description/urdf/amr.urdf`

```xml
<?xml version="1.0"?>
<robot name="amr">

  <!-- base_footprint: Projektion auf Boden -->
  <link name="base_footprint"/>

  <!-- base_link: Chassis -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.20 0.15 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- laser: LiDAR -->
  <link name="laser">
    <visual>
      <geometry>
        <cylinder radius="0.035" length="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Joint: base_footprint → base_link -->
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Joint: base_link → laser -->
  <joint name="base_link_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0.10 0 0.10" rpy="0 0 0"/>
  </joint>

</robot>
```

### 3.3 Package-Struktur

```
ros2_ws/src/amr_description/
├── urdf/
│   └── amr.urdf
├── launch/
│   └── description.launch.py
├── package.xml
└── CMakeLists.txt
```

---

## 4) Komponente 3: Launch-File

Datei: `ros2_ws/src/amr_description/launch/description.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('amr_description'),
        'urdf',
        'amr.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Robot State Publisher (statische TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Odom Converter (dynamische TF)
        Node(
            package='amr_bridge',
            executable='odom_converter',
            name='odom_converter',
            output='screen'
        ),
    ])
```

---

## 5) Installation & Build

### 5.1 Im Container

```bash
cd ~/amr-platform/docker
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

# Packages bauen
cd /root/ros2_ws
colcon build --packages-select amr_bridge amr_description
source install/setup.bash
```

### 5.2 Dependencies installieren (falls nötig)

```bash
apt-get update
apt-get install -y ros-humble-robot-state-publisher ros-humble-tf2-tools
```

---

## 6) Start & Test

### 6.1 Alle Nodes starten

**Terminal 1: RPLidar**
```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 2: TF-Nodes**
```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch amr_description description.launch.py
```

### 6.2 Smoke-Tests

**Terminal 3:**
```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash

# Topics prüfen
ros2 topic list | grep odom

# TF prüfen
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom laser

# TF-Baum visualisieren
ros2 run tf2_tools view_frames
# Erzeugt frames.pdf
```

---

## 7) Erwartete Ergebnisse

### 7.1 Topics

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/odom_raw
/odom           # NEU
/scan
/tf             # NEU
/tf_static      # NEU
```

### 7.2 TF-Baum

```
odom
└── base_footprint
    └── base_link
        └── laser
```

---

## 8) Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| TF disconnected | robot_state_publisher fehlt | Launch prüfen |
| Kein /odom | odom_converter fehlt | Node starten |
| Laser schwebt | laser_z falsch | URDF anpassen |
| TF springt | Zwei TF-Quellen | Nur eine Quelle erlaubt |

---

## 9) Nächste Schritte

Nach erfolgreichem TF-Baum:

1. **Phase 5: SLAM** mit slam_toolbox
2. **Optional: EKF** wenn IMU (MPU6050) vorhanden

---

## 10) Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v3.0 | 2025-12-20 | Vereinfacht ohne EKF, odom_converter.py, Minimal-URDF |
| v2.0 | 2025-12-20 | Mit EKF (überholt) |
| v1.0 | 2025-12-19 | Initiale Version |
