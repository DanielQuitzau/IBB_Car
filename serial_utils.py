# IBB_Car/utils/serial_utils.py
import serial
import time

class SerialConnection:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        """Öffnet die serielle Verbindung"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Arduino-Reset warten
            print(f"✅ Verbunden mit {self.port} @ {self.baudrate} Baud")
        except serial.SerialException as e:
            print(f"❌ Konnte {self.port} nicht öffnen: {e}")

    def read_line(self):
        """Liest eine Zeile vom Arduino"""
        if self.ser:
            try:
                line = self.ser.readline().decode("utf-8").strip()
                if line:  # Nur zurückgeben wenn nicht leer
                    return line
            except Exception as e:
                print(f"Fehler beim Lesen: {e}")
        return None

    def send(self, message):
        """Sendet eine Nachricht an den Arduino"""
        if self.ser:
            self.ser.write((message + "\n").encode("utf-8"))

    def close(self):
        if self.ser:
            self.ser.close()
            print("🔌 Serielle Verbindung geschlossen")
