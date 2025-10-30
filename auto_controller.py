#!/usr/bin/env python3
"""
IBB_Car Auto Controller Node
=============================
Dieser Node implementiert die automatische Fahrlogik:
- Fährt geradeaus
- Stoppt 100cm vor einem Hindernis
- Lenkt nach links ein

Autor: RPi IBB Team
Datum: 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time


class AutoController(Node):
    """
    Automatischer Fahrcontroller für das RC-Auto

    Subscribed Topics:
    ------------------
    /distance (Float32): Ultraschall-Distanzmessung in cm

    Published Topics:
    -----------------
    /cmd_drive (String): Steuerbefehle an Arduino (z.B. "M1600", "S1300")

    Parameter:
    ----------
    stop_distance: Abstand in cm, bei dem das Auto anhält (default: 100.0)
    drive_speed: PWM-Wert für Vorwärtsfahrt (default: 1600)
    stop_speed: PWM-Wert für Stopp (default: 1500)
    steer_left: PWM-Wert für Linkslenkung (default: 1300)
    steer_center: PWM-Wert für Geradeausfahrt (default: 1500)
    """

    def __init__(self):
        super().__init__('auto_controller')

        # ============================================
        # Parameter deklarieren und initialisieren
        # ============================================
        self.declare_parameter('stop_distance', 100.0)  # Stoppabstand in cm
        self.declare_parameter('drive_speed', 1600)  # Vorwärts PWM
        self.declare_parameter('stop_speed', 1500)  # Neutral/Stopp PWM
        self.declare_parameter('steer_left', 1300)  # Links PWM
        self.declare_parameter('steer_center', 1500)  # Geradeaus PWM

        # Parameter auslesen
        self.stop_distance = self.get_parameter('stop_distance').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.stop_speed = self.get_parameter('stop_speed').value
        self.steer_left = self.get_parameter('steer_left').value
        self.steer_center = self.get_parameter('steer_center').value

        # ============================================
        # Zustandsvariablen
        # ============================================
        self.current_distance = None  # Aktuelle gemessene Distanz
        self.state = "IDLE"  # Fahrzustand: IDLE, DRIVING, STOPPED, TURNING

        # ============================================
        # ROS2 Publisher einrichten
        # ============================================
        # Publisher für Fahrbefehle an serial_bridge
        self.cmd_pub = self.create_publisher(
            String,
            '/cmd_drive',
            10
        )

        # ============================================
        # ROS2 Subscriber einrichten
        # ============================================
        # Subscriber für Ultraschall-Distanzmessungen
        self.distance_sub = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10
        )

        # ============================================
        # Timer für Steuerungslogik
        # ============================================
        # Steuerungslogik mit 10 Hz ausführen (alle 0.1 Sekunden)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚗 Auto Controller gestartet')
        self.get_logger().info(f'   Stoppabstand: {self.stop_distance} cm')
        self.get_logger().info(f'   Fahrgeschwindigkeit: {self.drive_speed} µs PWM')

    def distance_callback(self, msg):
        """
        Callback-Funktion für eingehende Distanzmessungen

        Parameter:
        ----------
        msg (Float32): Distanz in cm vom Ultraschallsensor

        Diese Funktion wird automatisch aufgerufen, wenn eine neue
        Distanzmessung auf dem Topic /distance veröffentlicht wird.
        """
        self.current_distance = msg.data

        # Optional: Debug-Ausgabe (kann bei Bedarf auskommentiert werden)
        # self.get_logger().debug(f'📏 Distanz empfangen: {self.current_distance} cm')

    def send_motor_command(self, pwm_value):
        """
        Sendet einen Motor-Steuerbefehl an den Arduino

        Parameter:
        ----------
        pwm_value (int): PWM-Wert in Mikrosekunden (1000-2000)
                        1500 = Neutral/Stopp
                        >1500 = Vorwärts
                        <1500 = Rückwärts
        """
        cmd = String()
        cmd.data = f"M{pwm_value}"  # Format: "M1600"
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'🚙 Motor: {pwm_value} µs')

    def send_steering_command(self, pwm_value):
        """
        Sendet einen Lenk-Steuerbefehl an den Arduino

        Parameter:
        ----------
        pwm_value (int): PWM-Wert in Mikrosekunden (1000-2000)
                        1500 = Geradeaus
                        <1500 = Links
                        >1500 = Rechts
        """
        cmd = String()
        cmd.data = f"S{pwm_value}"  # Format: "S1300"
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'🎯 Lenkung: {pwm_value} µs')

    def control_loop(self):
        """
        Hauptsteuerungsschleife - wird alle 0.1 Sekunden aufgerufen

        Implementiert eine State-Machine mit folgenden Zuständen:
        - IDLE: Wartet auf gültige Distanzmessung
        - DRIVING: Fährt geradeaus
        - STOPPED: Hat gestoppt, wartet kurz
        - TURNING: Lenkt nach links ein
        """

        # Wenn noch keine Distanzmessung vorhanden, warten
        if self.current_distance is None:
            if self.state == "IDLE":
                self.get_logger().info('⏳ Warte auf erste Distanzmessung...')
            return

        # ============================================
        # State Machine
        # ============================================

        if self.state == "IDLE":
            # Initialer Zustand: Auto starten
            self.get_logger().info('🟢 Starte Fahrt...')
            self.send_steering_command(self.steer_center)  # Geradeaus
            self.send_motor_command(self.drive_speed)  # Los fahren
            self.state = "DRIVING"

        elif self.state == "DRIVING":
            # Fahrzustand: Überprüfe Distanz
            if self.current_distance <= self.stop_distance:
                # Hindernis erkannt - stoppen
                self.get_logger().info(f'🛑 Hindernis bei {self.current_distance:.1f} cm - STOPP!')
                self.send_motor_command(self.stop_speed)  # Motor stoppen
                self.state = "STOPPED"
                self.stop_time = time.time()  # Zeitpunkt des Stopps merken
            else:
                # Weiter geradeaus fahren
                # Optional: Debug-Ausgabe alle paar Messungen
                pass

        elif self.state == "STOPPED":
            # Gestoppt: Kurz warten (z.B. 1 Sekunde)
            if time.time() - self.stop_time > 1.0:
                self.get_logger().info('↩️ Lenke nach links ein...')
                self.send_steering_command(self.steer_left)  # Nach links lenken
                self.state = "TURNING"
                self.turn_time = time.time()  # Zeitpunkt der Lenkung merken

        elif self.state == "TURNING":
            # Lenkvorgang: Nach 2 Sekunden wieder geradeaus
            if time.time() - self.turn_time > 2.0:
                self.get_logger().info('✅ Lenkvorgang abgeschlossen - zurück zu Geradeaus')
                self.send_steering_command(self.steer_center)  # Zurück zu Geradeaus
                # Optionen für danach:
                # 1. Wieder losfahren: self.state = "DRIVING"
                # 2. Mission beendet: self.state = "IDLE"
                self.state = "IDLE"  # Für dieses Beispiel: Mission beendet
                self.get_logger().info('🏁 Mission abgeschlossen. Node läuft weiter.')


def main(args=None):
    """
    Hauptfunktion zum Starten des Nodes
    """
    # ROS2 initialisieren
    rclpy.init(args=args)

    # Node-Instanz erstellen
    node = AutoController()

    try:
        # Node laufen lassen (blockiert hier)
        # Callbacks werden automatisch aufgerufen
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Sauberes Beenden bei Ctrl+C
        node.get_logger().info('🛑 Beende Auto Controller...')
    finally:
        # Sicherheitshalber: Auto stoppen
        node.send_motor_command(1500)
        node.send_steering_command(1500)

        # Aufräumen
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()