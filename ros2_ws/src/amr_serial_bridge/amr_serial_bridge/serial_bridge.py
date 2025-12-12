#!/usr/bin/env python3
"""
AMR Serial Bridge Node
======================

Brücke zwischen ROS 2 /cmd_vel und ESP32 Serial-Protokoll.

Protokoll zum ESP32:
    Senden:   "V:<linear>,W:<angular>\n"
    Empfang:  "OK:<linear>,<angular>\n" oder "ERR:..." oder "FAILSAFE:..."

Autor: Jan Unger | FH Aachen
Version: 0.3.0
Datum: 2025-12-12
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time


class SerialBridge(Node):
    """ROS 2 Node für Serial-Kommunikation mit ESP32."""

    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameter deklarieren
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_rate', 20.0)  # Hz
        
        # Parameter lesen
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.send_rate = self.get_parameter('send_rate').value
        
        # Aktuelle Geschwindigkeitswerte
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_cmd_time = time.time()
        self.lock = threading.Lock()
        
        # Serial Port öffnen
        self.serial = None
        self.connect_serial()
        
        # Subscriber für /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer für regelmäßiges Senden (Heartbeat)
        period = 1.0 / self.send_rate
        self.timer = self.create_timer(period, self.send_to_esp32)
        
        # Thread für Serial-Empfang
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()
        
        self.get_logger().info(
            f'Serial Bridge gestartet: {self.port} @ {self.baud} baud'
        )

    def connect_serial(self):
        """Verbindung zum Serial Port herstellen."""
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.1
                )
                self.get_logger().info(f'Serial Port {self.port} geöffnet')
                time.sleep(2)  # ESP32 Reset abwarten
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f'Serial Port Fehler (Versuch {attempt + 1}/{max_retries}): {e}'
                )
                time.sleep(1)
        
        self.get_logger().error(f'Konnte Serial Port {self.port} nicht öffnen!')
        self.serial = None

    def cmd_vel_callback(self, msg: Twist):
        """Callback für /cmd_vel Topic."""
        with self.lock:
            self.linear_vel = msg.linear.x
            self.angular_vel = msg.angular.z
            self.last_cmd_time = time.time()

    def send_to_esp32(self):
        """Sendet aktuelle Geschwindigkeit an ESP32 (Heartbeat)."""
        if self.serial is None or not self.serial.is_open:
            self.connect_serial()
            return
        
        with self.lock:
            v = self.linear_vel
            w = self.angular_vel
            
            # Timeout-Check: Nach 0.5s ohne /cmd_vel → Stopp senden
            if time.time() - self.last_cmd_time > 0.5:
                v = 0.0
                w = 0.0
        
        # Befehl formatieren und senden
        cmd = f'V:{v:.3f},W:{w:.3f}\n'
        try:
            self.serial.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial Schreibfehler: {e}')
            self.serial = None

    def serial_reader(self):
        """Hintergrund-Thread für Serial-Empfang."""
        while self.running:
            if self.serial is None or not self.serial.is_open:
                time.sleep(1)
                continue
            
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self.process_response(line)
            except (serial.SerialException, UnicodeDecodeError) as e:
                self.get_logger().debug(f'Serial Lesefehler: {e}')
            
            time.sleep(0.01)

    def process_response(self, line: str):
        """Verarbeitet Antworten vom ESP32."""
        if line.startswith('OK:'):
            # Bestätigung - optional loggen
            self.get_logger().debug(f'ESP32: {line}')
        elif line.startswith('ERR:'):
            self.get_logger().warn(f'ESP32 Fehler: {line}')
        elif line.startswith('FAILSAFE:'):
            self.get_logger().warn(f'ESP32 Failsafe aktiv: {line}')
        elif line == 'READY':
            self.get_logger().info('ESP32 bereit')
        else:
            self.get_logger().debug(f'ESP32: {line}')

    def destroy_node(self):
        """Cleanup beim Beenden."""
        self.running = False
        
        # Stopp-Befehl senden
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(b'V:0.000,W:0.000\n')
                time.sleep(0.1)
                self.serial.close()
                self.get_logger().info('Serial Port geschlossen')
            except Exception:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
